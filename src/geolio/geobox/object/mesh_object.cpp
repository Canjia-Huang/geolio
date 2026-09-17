//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/19.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "mesh_object.h"
#include <geogram_gfx/imgui_ext/imgui_ext.h>
#include <geogram_gfx/third_party/imgui/imgui.h>
#include <geogram/mesh/mesh_geometry.h>
#include <geogram_gfx/GLUP/GLUP.h>
// glupPrivateVertex*()/glupPrivateTexCoord*(): the immediate-mode entry points
// used by MeshGfx::draw_vertex() and MeshGfx::draw_attribute_as_tex_coord(),
// which the cell-facet passes below reproduce (see GeoGram PR #386).
#include <geogram_gfx/GLUP/GLUP_private.h>
#include <geogram_gfx/basic/GL.h>

#include "geolio/common/log.h"

namespace geolio::geobox
{
    MeshObject::MeshObject(
        const std::string& name,
        const std::vector<ColormapInfo>& colormaps,
        const GEO::Mesh& mesh
        ) : BaseObject(name),
            colormaps_(colormaps)
    {
        mesh_.copy(mesh);
        mesh_gfx_.set_mesh(&mesh_);

        if (mesh_.edges.nb() == 0 && mesh_.facets.nb() == 0 && mesh_.cells.nb() == 0)
            show_vertices_ = true;
        // Seed the absolute (model-space) marker size from the mesh extent:
        // one hundredth of the bounding-box diagonal roughly reproduces the
        // on-screen size of the former fixed pixel points at the initial
        // framing, whatever the scale of the model.
        if (mesh_.vertices.nb() != 0) {
            if (const float diag = bbox_diagonal();
                diag > 0.0f)
                vertices_size_ = 0.01f * diag;
        }

        if (mesh_.edges.nb() == 0)
            show_edges_ = false;
        if (mesh_.facets.nb() == 0)
            show_surface_mesh_ = false;
        if (mesh_.cells.nb() == 0)
            show_volume_ = false;

        set_attribute(attribute_);
    }

    void MeshObject::draw_object_properties(
        ) {
        ImGui::PushID(this);

        if (ImGui::Button("Reload", ImVec2(-1, 0)))
            reload();
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("%s", filepath_.c_str());

        ImGui::Separator();
        if (ImGui::CollapsingHeader("Info")) {
            /* == Element count ==================================================================================== */
            if (ImGui::BeginTable(
                "##InfoTable", 2,
                ImGuiTableFlags_Borders | ImGuiTableFlags_SizingStretchSame)
                ) {
                ImGui::TableSetupColumn("Element");
                ImGui::TableSetupColumn("Count");
                ImGui::TableHeadersRow();

                // Clicking a count copies its value to the clipboard.
                const auto count_cell = [](const char* element, const GEO::index_t count) {
                    const std::string text =
                        std::to_string(static_cast<unsigned int>(count));
                    // The element name disambiguates the ImGui ID in case two
                    // counts happen to be equal.
                    const std::string label = text + "##" + element;
                    if (ImGui::Selectable(label.c_str(), false))
                        ImGui::SetClipboardText(text.c_str());
                    // if (ImGui::IsItemHovered())
                    //     ImGui::SetTooltip("Click to copy");
                };

                if (mesh_.vertices.nb() > 0) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("vertices");
                    ImGui::TableSetColumnIndex(1);
                    count_cell("vertices", mesh_.vertices.nb());
                }
                if (mesh_.edges.nb() > 0) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("edges");
                    ImGui::TableSetColumnIndex(1);
                    count_cell("edges", mesh_.edges.nb());
                }
                if (mesh_.facets.nb() > 0) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("facets");
                    ImGui::TableSetColumnIndex(1);
                    count_cell("facets", mesh_.facets.nb());
                }
                if (mesh_.cells.nb() > 0) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("cells");
                    ImGui::TableSetColumnIndex(1);
                    count_cell("cells", mesh_.cells.nb());
                }

                ImGui::EndTable();
            }
        }

        if (ImGui::CollapsingHeader("Render", ImGuiTreeNodeFlags_DefaultOpen)) {
            const auto s = static_cast<float>(ImGui::scaling());

            /* == Attributes ======================================================================================= */
            ImGui::Checkbox("attributes", &show_attributes_);
            if (show_attributes_) {
                if (attribute_min_ == 0.0f && attribute_max_ == 0.0f)
                    autorange();

                if (ImGui::Button(
                    (attribute_ + "##Attribute").c_str(),
                    ImVec2(-1, 0)))
                    ImGui::OpenPopup("##Attributes");

                if (ImGui::BeginPopup("##Attributes")) {
                    std::vector<std::string> attributes;
                    GEO::String::split_string(attribute_names(), ';', attributes);

                    for (const auto& attribute : attributes) {
                        if (ImGui::Button(attribute.c_str())) {
                            set_attribute(attribute);
                            ImGui::CloseCurrentPopup();
                        }
                    }
                    ImGui::EndPopup();
                }

                ImGui::InputFloat("min", &attribute_min_);
                ImGui::InputFloat("max", &attribute_max_);
                if (ImGui::Button("autorange", ImVec2(-1, 0)))
                    autorange();

                if (ImGui::ImageButton(
                    "choose_colormap",
                    static_cast<ImTextureID>(colormaps_[current_colormap_index_].texture),
                    ImVec2(0.95f * ImGui::GetContentRegionAvail().x, 8.0f*s))
                    ) {
                    ImGui::OpenPopup("##Colormap");
                }
                if (ImGui::BeginPopup("##Colormap")) {
                    for (GEO::index_t i = 0; i < colormaps_.size(); ++i) {
                        if (ImGui::ImageButton(
                            colormaps_[i].name.c_str(),
			                static_cast<ImTextureID>(colormaps_[i].texture),
                            ImVec2(100.0f*s, 8.0f*s))
                            ) {
                            current_colormap_index_   = i;
                            ImGui::CloseCurrentPopup();
                        }
                    }
                    ImGui::EndPopup();
                }
            }

            /* == Vertices ========================================================================================= */
            ImGui::Separator();
            ImGui::Checkbox("##VertOnOff", &show_vertices_);
            ImGui::SameLine();
            ImGui::ColorEdit3WithPalette("Vert.", vertices_color_.data());

            if (show_vertices_) {
                ImGui::Indent();

                // The size is an absolute (model-space) value: markers are
                // discs of this diameter in the model, so their on-screen
                // size follows the camera distance (near large, far small).
                // The range goes up to the model's bounding-box diagonal and
                // the drag speed is scaled accordingly, so both coarse and
                // fine adjustments are convenient.
                const float diag = bbox_diagonal();
                ImGui::DragFloat(
                    "sz.##vertices", &vertices_size_,
                    diag * 0.0001f,
                    0.0f, diag > 0.0f ? diag : 1.0f, "%.6g");
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip(
                        "Marker diameter in model coordinates (absolute "
                        "size). The on-screen size therefore varies with "
                        "the camera distance.");
                ImGui::SliderFloat("trsp.##vertices", &vertices_transparency_, 0.0f, 1.0f, "%.2f");

                ImGui::Unindent();
            }

            /* == Edges (explicit mesh.edges) ========================================================================= */
            if (mesh_.edges.nb() != 0) {
                ImGui::Separator();
                ImGui::Checkbox("##EdgeOnOff", &show_edges_);
                ImGui::SameLine();
                ImGui::ColorEdit3WithPalette("Edge.", edges_color_.data());
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip(
                        "Explicit 1-D edges stored in the mesh (mesh.edges), "
                        "e.g. feature edges. Independent from the facet and "
                        "cell wireframes.");
                if (show_edges_)
                    ImGui::SliderFloat("wid.##edges", &edges_width_, 0.1f, 2.0f, "%.1f");
            }

            /* == Facets =========================================================================================== */
            if (mesh_.facets.nb() != 0) {
                ImGui::Separator();
                ImGui::Checkbox("##SurfOnOff", &show_surface_);
                ImGui::SameLine();
                ImGui::ColorEdit3WithPalette("Surf.", surface_color_.data());

                if (show_surface_) {
                    ImGui::Indent();

                    ImGui::Checkbox("##SidesOnOff", &show_surface_sides_);
                    ImGui::SameLine();
                    ImGui::ColorEdit3WithPalette("2sided", surface_color_2_.data());

                    ImGui::SliderFloat("trsp.##surface", &surface_transparency_, 0.0f, 1.0f, "%.2f");

                    ImGui::Checkbox("##MeshOnOff", &show_surface_mesh_);
                    ImGui::SameLine();
                    ImGui::ColorEdit3WithPalette("mesh", surface_mesh_color_.data());
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip(
                            "Wireframe of the surface facets "
                            "(the edges of mesh.facets).");
                    if (show_surface_mesh_)
                        ImGui::SliderFloat("wid.##mesh", &surface_mesh_width_, 0.1f, 2.0f, "%.1f");

                    ImGui::Checkbox("##BordersOnOff", &show_surface_borders_);
                    ImGui::SameLine();
                    ImGui::ColorEdit3WithPalette("borders", surface_borders_color_.data());
                    if (show_surface_borders_)
                        ImGui::SliderFloat("wid.##borders", &surface_borders_width_, 0.1f, 2.0f, "%.1f");

                    ImGui::Unindent();
                }
            }

            /* == Cells ============================================================================================ */
            if (mesh_.cells.nb() != 0) {
                ImGui::Separator();
                ImGui::Checkbox("##VolumeOnOff", &show_volume_);
                ImGui::SameLine();
                ImGui::ColorEdit3WithPalette("Volume", volume_color_.data());

                if (show_volume_) {
                    ImGui::Indent();

                    ImGui::Checkbox("##CellMeshOnOff", &show_volume_mesh_);
                    ImGui::SameLine();
                    ImGui::ColorEdit3WithPalette("cell mesh", volume_mesh_color_.data());
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip(
                            "Wireframe of the volume cells "
                            "(the edges of mesh.cells).");
                    if (show_volume_mesh_)
                        ImGui::SliderFloat("wid.##cellmesh", &volume_mesh_width_, 0.1f, 2.0f, "%.1f");

                    ImGui::SliderFloat("shrk.", &cells_shrink_, 0.0f, 1.0f, "%.2f");
                    if (!mesh_.cells.are_simplices()) {
                        ImGui::Checkbox("colored cells", &show_colored_cells_);
                        ImGui::Checkbox("hexes", &show_hexes_);
                    }

                    ImGui::Unindent();
                }
            }
        }

        ImGui::PopID();
    }

    void MeshObject::draw_scene(
        const bool lighting
        ) {
        if (mesh_gfx_.mesh() == nullptr)
            return;

        mesh_gfx_.set_lighting(lighting);

        if (show_attributes_) {
            mesh_gfx_.set_scalar_attribute(
                attribute_subelements_,
                attribute_name_,
                static_cast<double>(attribute_min_),
                static_cast<double>(attribute_max_),
                colormaps_[current_colormap_index_].texture,
                1);
        }
        else
            mesh_gfx_.unset_scalar_attribute();

        // Opaque geometry first, then the points on top: transparent points do
        // not write depth, so drawing them first would let the surface occlude
        // them; drawing them last keeps them visible on the surface.
        draw_surface();
        draw_edges();
        draw_volume(lighting);
        draw_points();
    }

    void MeshObject::reload(
        ) {
        if (GEO::Mesh mesh;
            mesh.load(filepath_)) {
            mesh_.copy(mesh);
            bbox_diag_ = -1.0f; // invalidate the cached bounding-box diagonal
        }
    }

    void MeshObject::get_bbox(
        double* xyzmin,
        double* xyzmax
        ) const {
        if (mesh_.vertices.dimension() == 2) {
            for(GEO::coord_index_t c = 0; c < 2; c++) {
                xyzmin[c] = GEO::Numeric::max_float64();
                xyzmax[c] = GEO::Numeric::min_float64();
            }
            xyzmin[2] = 0;
            xyzmax[2] = 0;
            for(const GEO::vec2& p: mesh_.vertices.points<2>()) {
                for(GEO::coord_index_t c = 0; c < 2; c++) {
                    xyzmin[c] = std::min(xyzmin[c], p[c]);
                    xyzmax[c] = std::max(xyzmax[c], p[c]);
                }
            }
        }
        else
            GEO::get_bbox(mesh_, xyzmin, xyzmax);
    }

    float MeshObject::bbox_diagonal(
        ) const {
        if (bbox_diag_ < 0.0f) {
            double xyzmin[3] = {0.0, 0.0, 0.0};
            double xyzmax[3] = {0.0, 0.0, 0.0};
            get_bbox(xyzmin, xyzmax);
            double d2 = 0.0;
            for (GEO::coord_index_t c = 0; c < 3; ++c) {
                const double d = xyzmax[c] - xyzmin[c];
                d2 += d * d;
            }
            bbox_diag_ = static_cast<float>(std::sqrt(d2));
        }
        return bbox_diag_;
    }

    void MeshObject::draw_points(
        ) {
        if (!show_vertices_ || mesh_.vertices.nb() == 0 ||
            vertices_size_ <= 0.0f)
            return;

        // Vertex markers have an absolute (model-space) diameter
        // (vertices_size_), so their on-screen size must follow the camera
        // distance (near large, far small). MeshGfx::draw_vertices() cannot
        // express that: GLUP points rasterize with a single point-sprite size
        // for the whole draw call, i.e. a constant size on screen. Instead,
        // each vertex is drawn as a camera-facing disc of the requested world
        // diameter using GLUP's sphere impostor primitive (per-vertex radius
        // in the w component of glupVertex4d), which gives the round marker
        // silhouette with correct perspective scaling.
        const bool blended = vertices_transparency_ != 0.0f;
        if (blended) {
            glDepthMask(GL_FALSE);
            glEnable(GL_BLEND);
            glBlendEquation(GL_FUNC_ADD);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        }

        // Render flat markers: without shading, the impostor spheres look
        // like round discs. Restore the previous lighting state afterwards
        // so the next object's surface pass is unaffected.
        const bool lighting = glupIsEnabled(GLUP_LIGHTING) != GL_FALSE;
        glupDisable(GLUP_LIGHTING);

        // Vertices sit exactly on the surface, so the default GL_LESS depth
        // test would cull them at equal depth; use an inclusive test so the
        // markers stay visible on top of the model.
        glDepthFunc(GL_LEQUAL);

        const double radius = 0.5 * static_cast<double>(vertices_size_);

        // When a scalar attribute is displayed on the vertices, color each
        // marker through GLUP's 1-D colormap texturing (the same state
        // MeshGfx::begin_attributes() sets up), so the markers keep the
        // attribute coloring; otherwise use the plain vertex color.
        GEO::ReadOnlyScalarAttributeAdapter scalar_attribute;
        bool textured = false;
        if (show_attributes_ &&
            attribute_subelements_ == GEO::MESH_VERTICES) {
            scalar_attribute.bind_if_is_defined(
                mesh_.vertices.attributes(), attribute_name_
            );
            if (scalar_attribute.is_bound()) {
                textured = true;
                glupEnable(GLUP_TEXTURING);
                glupTextureMode(GLUP_TEXTURE_REPLACE);
                glupTextureType(GLUP_TEXTURE_1D);
                glActiveTexture(GL_TEXTURE0 + GLUP_TEXTURE_1D_UNIT);
                glBindTexture(
                    GL_TEXTURE_2D,
                    colormaps_[current_colormap_index_].texture
                );
                GEO::glupMapTexCoords1d(
                    static_cast<double>(attribute_min_),
                    static_cast<double>(attribute_max_),
                    1
                );
                glupSetColor3f(
                    GLUP_FRONT_AND_BACK_COLOR, 1.0f, 1.0f, 1.0f
                );
            }
        }
        if (!textured) {
            const float rgba[4] = {
                vertices_color_.x,
                vertices_color_.y,
                vertices_color_.z,
                1.0f - vertices_transparency_
            };
            glupSetColor4fv(GLUP_FRONT_COLOR, rgba);
        }

        glupBegin(GLUP_SPHERES);
        {
            const bool single_precision = mesh_.vertices.single_precision();
            const GEO::coord_index_t dim = mesh_.vertices.dimension();
            const GEO::index_t nb = mesh_.vertices.nb();
            for (GEO::index_t v = 0; v < nb; ++v) {
                double p[3] = {0.0, 0.0, 0.0};
                if (single_precision) {
                    const float* pp =
                        mesh_.vertices.single_precision_point_ptr(v);
                    for (GEO::coord_index_t c = 0; c < dim && c < 3; ++c)
                        p[c] = static_cast<double>(pp[c]);
                }
                else {
                    const double* pp = mesh_.vertices.point_ptr(v);
                    for (GEO::coord_index_t c = 0; c < dim && c < 3; ++c)
                        p[c] = pp[c];
                }
                if (textured)
                    glupTexCoord1d(scalar_attribute[v]);
                glupVertex4d(p[0], p[1], p[2], radius);
            }
        }
        glupEnd();

        if (textured) {
            glupDisable(GLUP_TEXTURING);
            // Reset the texture matrix, as MeshGfx::end_attributes() does.
            glupMatrixMode(GLUP_TEXTURE_MATRIX);
            glupLoadIdentity();
            glupMatrixMode(GLUP_MODELVIEW_MATRIX);
        }
        if (lighting)
            glupEnable(GLUP_LIGHTING);

        glDepthFunc(GL_LESS);

        if (blended) {
            glDisable(GL_BLEND);
            glDepthMask(GL_TRUE);
        }
    }

    void MeshObject::draw_surface(
        ) {
        mesh_gfx_.set_mesh_color(0.0, 0.0, 0.0);

        const float alpha = 1.0f - surface_transparency_;
        if (surface_transparency_ != 0.0f) {
            glDepthMask(GL_FALSE);
            glEnable(GL_BLEND);
            glBlendEquation(GL_FUNC_ADD);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        }

        mesh_gfx_.set_surface_color(
            surface_color_.x, surface_color_.y, surface_color_.z, alpha);
        if (show_surface_sides_) {
            mesh_gfx_.set_backface_surface_color(
                surface_color_2_.x, surface_color_2_.y, surface_color_2_.z, alpha);
        }

        mesh_gfx_.set_show_mesh(show_surface_mesh_);
        mesh_gfx_.set_mesh_color(
            surface_mesh_color_.x, surface_mesh_color_.y, surface_mesh_color_.z);
        mesh_gfx_.set_mesh_width(
            static_cast<GEO::index_t>(surface_mesh_width_ * 10.0f));

        if (show_surface_) {
            const float specular_backup = glupGetSpecular();
            glupSetSpecular(0.4f);
            mesh_gfx_.draw_surface();
            glupSetSpecular(specular_backup);

            if (show_surface_borders_) {
                mesh_gfx_.set_mesh_color(
                    surface_borders_color_.x,
                    surface_borders_color_.y,
                    surface_borders_color_.z);
                mesh_gfx_.set_mesh_border_width(
                    static_cast<GEO::index_t>(surface_borders_width_ * 10.0f));
                mesh_gfx_.draw_surface_borders();

                // The border pass above leaves MeshGfx's mesh color set to the border
                // color; restore it so later passes (edges, volume wireframe) use the
                // surface mesh color.
                mesh_gfx_.set_mesh_color(
                    surface_mesh_color_.x, surface_mesh_color_.y, surface_mesh_color_.z);
            }
        }

        if (surface_transparency_ != 0.0f) {
            glDisable(GL_BLEND);
            glDepthMask(GL_TRUE);
        }
    }

    void MeshObject::draw_edges(
        ) {
        if (!show_edges_ || mesh_.edges.nb() == 0)
            return;

        // MeshGfx::draw_edges() renders only the explicit mesh.edges store and
        // picks up the color/width from its mesh_color_/mesh_width_ members;
        // borrow them for this pass, then restore them so the facet and cell
        // wireframes keep their own styles.
        mesh_gfx_.set_mesh_color(
            edges_color_.x, edges_color_.y, edges_color_.z);
        mesh_gfx_.set_mesh_width(
            static_cast<GEO::index_t>(edges_width_ * 10.0f));
        mesh_gfx_.draw_edges();
        mesh_gfx_.set_mesh_color(
            surface_mesh_color_.x, surface_mesh_color_.y, surface_mesh_color_.z);
        mesh_gfx_.set_mesh_width(
            static_cast<GEO::index_t>(surface_mesh_width_ * 10.0f));
    }

    void MeshObject::draw_volume(
        const bool lighting
        ) {
        if (show_volume_) {
            if (glupIsEnabled(GLUP_CLIPPING) &&
                glupGetClipMode() == GLUP_CLIP_SLICE_CELLS)
                mesh_gfx_.set_lighting(false);

            mesh_gfx_.set_shrink(static_cast<double>(cells_shrink_));
            mesh_gfx_.set_draw_cells(GEO::MESH_HEX, show_hexes_);
            mesh_gfx_.set_draw_cells(GEO::MESH_CONNECTOR, show_connectors_);

            if(show_colored_cells_)
                mesh_gfx_.set_cells_colors_by_type();
            else
                mesh_gfx_.set_cells_color(
                    volume_color_.x, volume_color_.y, volume_color_.z);

            // The cell wireframe is emitted by GLUP itself (GLUP_DRAW_MESH),
            // whose state MeshGfx derives from its show_mesh_/mesh_color_/
            // mesh_width_ members at the start of draw_volume(); override
            // them for this pass and restore them afterwards, so the cell
            // wireframe is independent from the facet wireframe and from the
            // explicit mesh.edges.
            const bool saved_show_mesh = mesh_gfx_.get_show_mesh();
            mesh_gfx_.set_show_mesh(show_volume_mesh_);
            mesh_gfx_.set_mesh_color(
                volume_mesh_color_.x, volume_mesh_color_.y, volume_mesh_color_.z);
            mesh_gfx_.set_mesh_width(
                static_cast<GEO::index_t>(volume_mesh_width_ * 10.0f));

            // A scalar attribute defined on the cell facets is attached to the
            // half-facets of the cells, so MeshGfx::draw_volume() cannot render
            // it with whole-cell primitives: the facets are drawn one by one
            // instead (see draw_volume_cell_facets_attribute(), GeoGram PR #386).
            if (cell_facets_attribute_active())
                draw_volume_cell_facets_attribute();
            else
                mesh_gfx_.draw_volume();

            mesh_gfx_.set_show_mesh(saved_show_mesh);
            mesh_gfx_.set_mesh_color(
                surface_mesh_color_.x, surface_mesh_color_.y, surface_mesh_color_.z);
            mesh_gfx_.set_mesh_width(
                static_cast<GEO::index_t>(surface_mesh_width_ * 10.0f));

            mesh_gfx_.set_lighting(lighting);
        }
    }

    bool MeshObject::cell_facets_attribute_active(
        ) const {
        // GeoGram PR #386 also requires MeshGfx's picking mode to be MESH_NONE;
        // GeoBox never picks in the mesh (MeshGfx::set_picking_mode() is not
        // called anywhere in the application), so there is nothing to test here.
        return show_attributes_ &&
               attribute_subelements_ == GEO::MESH_CELL_FACETS;
    }

    double MeshObject::cells_shrink_factor(
        ) const {
        // GLUP shrinks a primitive by moving each of its vertices toward the
        // average of the primitive's vertices (see
        // GLUP::Context::shrink_cells_in_immediate_buffers()), and it skips the
        // shrink entirely when the cells are cut by a slice, which the clip
        // modes below test. The facet passes apply the same shrink to the cells
        // the facets belong to, so they have to skip it in the same cases.
        if (cells_shrink_ == 0.0f)
            return 0.0;
        if (glupIsEnabled(GLUP_CLIPPING) &&
            glupGetClipMode() == GLUP_CLIP_SLICE_CELLS)
            return 0.0;
        return static_cast<double>(cells_shrink_);
    }

    void MeshObject::draw_volume_cell_facets_attribute(
        ) {
        // GeoBox implementation of the GeoGram PR #386 ("render volume-mesh
        // cell_facets attributes") drawing paths, i.e. of what
        // MeshGfx::draw_tets_immediate_attrib() and
        // MeshGfx::draw_hybrid_immediate_attrib() do when the displayed
        // attribute is on the cell facets.
        //
        // The pass cannot be delegated to MeshGfx::draw_volume(): the helpers
        // it is built on (begin_attributes(), draw_vertex(),
        // draw_sequences_if(), draw_volume_vertex_with_attribute()...) are
        // protected members of MeshGfx, and MeshObject owns a MeshGfx instead of
        // deriving from it. The GLUP state that MeshGfx::draw_volume() sets up
        // is therefore reproduced here:
        //   MeshGfx::set_GLUP_parameters() -> GLUP_DRAW_MESH, GLUP_MESH_COLOR,
        //     GLUP_MESH_WIDTH and GLUP_LIGHTING;
        //   MeshGfx::draw_tets()/draw_hybrid() -> the cells color;
        //   MeshGfx::draw_volume() -> glupSetCellsShrink(shrink_), which is
        //     replaced by a per facet vertex shrink (see
        //     draw_volume_facet_vertex()).

        // == MeshGfx::set_GLUP_parameters() ==================================
        if (mesh_gfx_.get_show_mesh())
            glupEnable(GLUP_DRAW_MESH);
        else
            glupDisable(GLUP_DRAW_MESH);

        // The mesh color/width were set by draw_volume() for this pass (they are
        // restored right after), so they are read back from MeshGfx rather than
        // duplicated here.
        float mesh_color[4] = {0.0f, 0.0f, 0.0f, 1.0f};
        mesh_gfx_.get_mesh_color(
            mesh_color[0], mesh_color[1], mesh_color[2], mesh_color[3]);
        glupSetColor4fv(GLUP_MESH_COLOR, mesh_color);
        glupSetMeshWidth(GLUPint(mesh_gfx_.get_mesh_width()));

        if (mesh_gfx_.get_lighting())
            glupEnable(GLUP_LIGHTING);
        else
            glupDisable(GLUP_LIGHTING);

        // Cells color, as MeshGfx::draw_tets()/draw_hybrid() set it from their
        // cells_color_ table before emitting the cells. It is only visible when
        // no attribute is bound, since the GLUP_TEXTURE_REPLACE mode set up by
        // begin_attributes() replaces it.
        glupSetColor4fv(GLUP_FRONT_AND_BACK_COLOR, volume_color_.data());

        // == MeshGfx::draw_volume() ==========================================
        // GLUP shrinks whole-cell primitives only, and the facets are drawn as
        // plain triangles/quads: disable GLUP's shrink and replicate it per
        // facet vertex (see draw_volume_facet_vertex()).
        const GLUPfloat saved_shrink = GLUPfloat(cells_shrink_);
        glupSetCellsShrink(0.0f);

        begin_attributes();

        if (mesh_.cells.are_simplices()) {
            // MeshGfx::draw_tets_immediate_attrib(): all the cells are
            // tetrahedra, so all their facets are triangles.
            if (mesh_gfx_.get_draw_cells(GEO::MESH_TET)) {
                draw_volume_cell_facets(0, mesh_.cells.nb(), GLUP_TRIANGLES);
                draw_volume_cell_facets(0, mesh_.cells.nb(), GLUP_QUADS);
            }
        } else {
            // MeshGfx::draw_hybrid_immediate_attrib(): one pass per cell type,
            // for the types that are displayed and present in the mesh.
            bool has_cells[GEO::MESH_NB_CELL_TYPES] = {};
            for (GEO::index_t c = 0; c < mesh_.cells.nb(); ++c)
                has_cells[mesh_.cells.type(c)] = true;

            for (GEO::index_t type = GEO::MESH_TET;
                 type < GEO::MESH_NB_CELL_TYPES;
                 ++type) {
                if (!mesh_gfx_.get_draw_cells(
                        static_cast<GEO::MeshCellType>(type)) ||
                    !has_cells[type])
                    continue;

                // Equivalent of MeshGfx::draw_sequences_if(): the cells of a
                // given type are drawn by runs of consecutive cells of that
                // type (a hybrid mesh usually stores its types in blocks).
                GEO::index_t c = 0;
                while (c < mesh_.cells.nb()) {
                    while (c < mesh_.cells.nb() &&
                           GEO::index_t(mesh_.cells.type(c)) != type)
                        ++c;
                    const GEO::index_t begin_c = c;
                    while (c < mesh_.cells.nb() &&
                           GEO::index_t(mesh_.cells.type(c)) == type)
                        ++c;
                    if (begin_c == c)
                        break;
                    draw_volume_cell_facets(begin_c, c, GLUP_TRIANGLES);
                    draw_volume_cell_facets(begin_c, c, GLUP_QUADS);
                }
            }
        }

        end_attributes();

        glupSetCellsShrink(saved_shrink);
    }

    void MeshObject::draw_volume_cell_facets(
        const GEO::index_t begin_c,
        const GEO::index_t end_c,
        const GLUPprimitive prim
        ) {
        glupBegin(prim);
        for (GEO::index_t c = begin_c; c < end_c; ++c) {
            // Center of the cell, toward which its facets are pulled when they
            // are shrunk; only needed (and only computed) when shrink is set.
            double centroid[3] = {0.0, 0.0, 0.0};
            if (cells_shrink_factor() != 0.0) {
                const GEO::index_t nb_vertices = mesh_.cells.nb_vertices(c);
                for (GEO::index_t lv = 0; lv < nb_vertices; ++lv) {
                    double p[3] = {0.0, 0.0, 0.0};
                    get_vertex_position(mesh_.cells.vertex(c, lv), p);
                    centroid[0] += p[0];
                    centroid[1] += p[1];
                    centroid[2] += p[2];
                }
                centroid[0] /= double(nb_vertices);
                centroid[1] /= double(nb_vertices);
                centroid[2] /= double(nb_vertices);
            }

            const GEO::index_t nb_facets = mesh_.cells.nb_facets(c);
            for (GEO::index_t lf = 0; lf < nb_facets; ++lf) {
                // Index of the half-facet (cell, facet) in mesh.cell_facets: the
                // attribute of the facet is looked up with it.
                const GEO::index_t cell_facet = mesh_.cells.facet(c, lf);
                const GEO::index_t nb_vertices =
                    mesh_.cells.facet_nb_vertices(c, lf);
                if (prim == GLUP_QUADS) {
                    if (nb_vertices == 4) {
                        for (GEO::index_t lv = 0; lv < 4; ++lv)
                            draw_volume_facet_vertex(
                                c, lf, lv, cell_facet, centroid);
                    }
                }
                else if (nb_vertices == 3) {
                    for (GEO::index_t lv = 0; lv < 3; ++lv)
                        draw_volume_facet_vertex(
                            c, lf, lv, cell_facet, centroid);
                }
                else if (nb_vertices > 4) {
                    // Defensive fan for unexpected facet sizes (a cell facet is
                    // a triangle or a quad in GeoGram's cell descriptors).
                    for (GEO::index_t lv = 1; lv + 1 < nb_vertices; ++lv) {
                        for (const GEO::index_t k :
                             {GEO::index_t(0), lv, lv + 1})
                            draw_volume_facet_vertex(
                                c, lf, k, cell_facet, centroid);
                    }
                }
            }
        }
        glupEnd();
    }

    void MeshObject::draw_volume_facet_vertex(
        const GEO::index_t cell,
        const GEO::index_t lf,
        const GEO::index_t lv,
        const GEO::index_t cell_facet,
        const double* centroid
        ) {
        // The color comes from the attribute of the cell facet, so the two
        // facets shared by two adjacent cells can be colored differently.
        draw_attribute_as_tex_coord(cell_facet);

        const GEO::index_t v = mesh_.cells.facet_vertex(cell, lf, lv);

        // Without shrink, the vertex is emitted as-is, so that the facets fit
        // the mesh vertices exactly (and 2-D meshes are handled).
        const double shrink = cells_shrink_factor();
        if (shrink == 0.0) {
            draw_vertex(v);
            return;
        }

        // With shrink, the vertex is moved toward the center of its cell by the
        // same amount as GLUP applies to whole-cell primitives.
        double p[3] = {0.0, 0.0, 0.0};
        if (mesh_gfx_.get_animate() && mesh_.vertices.dimension() >= 6) {
            // Position of the vertex at the current animation time: the first
            // three coordinates are the position at t=0, the last three at t=1
            // (see MeshGfx::set_animate()).
            const double t = mesh_gfx_.get_time();
            const double s = 1.0 - t;
            for (GEO::coord_index_t coords = 0; coords < 3; ++coords) {
                if (mesh_.vertices.single_precision()) {
                    const float* q =
                        mesh_.vertices.single_precision_point_ptr(v);
                    p[coords] = s * double(q[coords]) + t * double(q[coords + 3]);
                }
                else {
                    const double* q = mesh_.vertices.point_ptr(v);
                    p[coords] = s * q[coords] + t * q[coords + 3];
                }
            }
        }
        else
            get_vertex_position(v, p);

        glupPrivateVertex3d(
            (1.0 - shrink) * p[0] + shrink * centroid[0],
            (1.0 - shrink) * p[1] + shrink * centroid[1],
            (1.0 - shrink) * p[2] + shrink * centroid[2]
        );
    }

    void MeshObject::draw_vertex(
        const GEO::index_t v
        ) {
        // Same as MeshGfx::draw_vertex(): when animation is active, the vertex
        // moves between the two positions stored in its six coordinates.
        if (mesh_gfx_.get_animate() && mesh_.vertices.dimension() >= 6) {
            const double t = mesh_gfx_.get_time();
            const double s = 1.0 - t;
            if (mesh_.vertices.single_precision()) {
                const float* p = mesh_.vertices.single_precision_point_ptr(v);
                glupPrivateVertex3f(
                    GLUPfloat(s * double(p[0]) + t * double(p[3])),
                    GLUPfloat(s * double(p[1]) + t * double(p[4])),
                    GLUPfloat(s * double(p[2]) + t * double(p[5]))
                );
            }
            else {
                const double* p = mesh_.vertices.point_ptr(v);
                glupPrivateVertex3d(
                    s * p[0] + t * p[3],
                    s * p[1] + t * p[4],
                    s * p[2] + t * p[5]
                );
            }
        }
        else if (mesh_.vertices.single_precision()) {
            if (mesh_.vertices.dimension() < 3)
                glupPrivateVertex2fv(
                    mesh_.vertices.single_precision_point_ptr(v));
            else
                glupPrivateVertex3fv(
                    mesh_.vertices.single_precision_point_ptr(v));
        }
        else {
            if (mesh_.vertices.dimension() < 3)
                glupPrivateVertex2dv(mesh_.vertices.point_ptr(v));
            else
                glupPrivateVertex3dv(mesh_.vertices.point_ptr(v));
        }
    }

    void MeshObject::get_vertex_position(
        const GEO::index_t v,
        double* p
        ) const {
        // 2-D meshes have no z coordinate (see get_bbox()).
        const GEO::coord_index_t dim = mesh_.vertices.dimension();
        p[0] = p[1] = p[2] = 0.0;
        if (mesh_.vertices.single_precision()) {
            const float* q = mesh_.vertices.single_precision_point_ptr(v);
            for (GEO::coord_index_t c = 0; c < dim && c < 3; ++c)
                p[c] = static_cast<double>(q[c]);
        }
        else {
            const double* q = mesh_.vertices.point_ptr(v);
            for (GEO::coord_index_t c = 0; c < dim && c < 3; ++c)
                p[c] = q[c];
        }
    }

    void MeshObject::draw_attribute_as_tex_coord(
        const GEO::index_t element
        ) {
        // GeoBox only displays scalar (1-D) attributes: draw_scene() always
        // calls MeshGfx::set_scalar_attribute(), which sets attribute_dim_ = 1.
        glupPrivateTexCoord1d(scalar_attribute_[element]);
    }

    void MeshObject::begin_attributes(
        ) {
        // Same as MeshGfx::begin_attributes() (a protected member there), for
        // the 1-D scalar attributes displayed by GeoBox.
        if (attribute_subelements_ == GEO::MESH_NONE)
            return;

        const GEO::MeshSubElementsStore& subelements =
            mesh_.get_subelements_by_type(attribute_subelements_);
        scalar_attribute_.bind_if_is_defined(
            subelements.attributes(), attribute_name_);
        if (!scalar_attribute_.is_bound()) {
            // No attribute to display: keep the plain element color.
            return;
        }

        glupEnable(GLUP_TEXTURING);
        glupTextureMode(GLUP_TEXTURE_REPLACE);
        glupTextureType(GLUP_TEXTURE_1D);

        // The colormap selected in the UI is a 1-D texture (stored as a 2-D
        // texture with a single row, see GLUP_TEXTURE_1D_TARGET).
        glActiveTexture(GL_TEXTURE0 + GLUP_TEXTURE_1D_UNIT);
        glBindTexture(
            GLUP_TEXTURE_1D_TARGET,
            colormaps_[current_colormap_index_].texture
        );

        // Rescale the attribute range [attribute_min_, attribute_max_] to [0,1].
        GEO::glupMapTexCoords1d(
            static_cast<double>(attribute_min_),
            static_cast<double>(attribute_max_),
            1
        );

        if (!glupIsEnabled(GLUP_NORMAL_MAPPING))
            glupSetColor3f(GLUP_FRONT_AND_BACK_COLOR, 1.0f, 1.0f, 1.0f);
    }

    void MeshObject::end_attributes(
        ) {
        // Same as MeshGfx::end_attributes().
        if (scalar_attribute_.is_bound()) {
            glupDisable(GLUP_TEXTURING);
            scalar_attribute_.unbind();
        }
        glupMatrixMode(GLUP_TEXTURE_MATRIX);
        glupLoadIdentity();
        glupMatrixMode(GLUP_MODELVIEW_MATRIX);
    }

    void MeshObject::autorange(
        ) {
        if (attribute_subelements_ == GEO::MESH_NONE)
            return;

        const GEO::MeshSubElementsStore& subelements =
            mesh_.get_subelements_by_type(attribute_subelements_);
            GEO::ReadOnlyScalarAttributeAdapter attribute(
            subelements.attributes(), attribute_name_
            );

        attribute_min_ = 0.0;
        attribute_max_ = 0.0;
        if (attribute.is_bound()) {
            attribute_min_ = GEO::Numeric::max_float32();
            attribute_max_ = GEO::Numeric::min_float32();
            for (GEO::index_t i = 0; i < subelements.nb(); ++i) {
                attribute_min_ =
                    std::min(attribute_min_, static_cast<float>(attribute[i]));
                attribute_max_ =
                    std::max(attribute_max_, static_cast<float>(attribute[i]));
            }
        }
    }

    void MeshObject::set_attribute(
        const std::string& attribute
        ) {
        attribute_ = attribute;
        std::string subelements_name;
        GEO::String::split_string(
            attribute_, '.',
            subelements_name,
            attribute_name_);

        attribute_subelements_ = GEO::Mesh::name_to_subelements_type(subelements_name);

        // if (attribute_min_ == 0.0f && attribute_max_ == 0.0f)
        autorange();
    }
}
