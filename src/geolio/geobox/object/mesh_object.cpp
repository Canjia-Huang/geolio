//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/19.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "mesh_object.h"
#include <geogram_gfx/imgui_ext/imgui_ext.h>
#include <geogram_gfx/third_party/imgui/imgui.h>
#include <geogram/mesh/mesh_geometry.h>

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

        set_attribute(attribute_);
    }

    void MeshObject::draw_object_properties(
        ) {
        ImGui::PushID(this);

        if (ImGui::Button("Reload", ImVec2(-1, 0)))
            reload();

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

                if (mesh_.vertices.nb() > 0) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("vertices");
                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text("%u", static_cast<unsigned int>(mesh_.vertices.nb()));
                }
                if (mesh_.edges.nb() > 0) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("edges");
                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text("%u", static_cast<unsigned int>(mesh_.edges.nb()));
                }
                if (mesh_.facets.nb() > 0) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("facets");
                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text("%u", static_cast<unsigned int>(mesh_.facets.nb()));
                }
                if (mesh_.cells.nb() > 0) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("cells");
                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text("%u", static_cast<unsigned int>(mesh_.cells.nb()));
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

                ImGui::SliderFloat("sz.", &vertices_size_, 0.1f, 5.0f, "%.1f");
                ImGui::SliderFloat("trsp.##vertices", &vertices_transparency_, 0.0f, 1.0f, "%.2f");

                ImGui::Unindent();
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

                    ImGui::Checkbox("##MeshOnOff", &show_mesh_);
                    ImGui::SameLine();
                    ImGui::ColorEdit3WithPalette("mesh", mesh_color_.data());
                    if (show_mesh_)
                        ImGui::SliderFloat("wid.##mesh", &mesh_width_, 0.1f, 2.0f, "%.1f");

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
            mesh.load(filepath_))
            mesh_.copy(mesh);
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

    void MeshObject::draw_points(
        ) {
        if(show_vertices_) {
            if(vertices_transparency_ != 0.0f) {
                glDepthMask(GL_FALSE);
                glEnable(GL_BLEND);
                glBlendEquation(GL_FUNC_ADD);
                glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            }
            mesh_gfx_.set_points_color(
                vertices_color_.x, vertices_color_.y, vertices_color_.z,
                1.0f - vertices_transparency_
            );
            mesh_gfx_.set_points_size(vertices_size_);

            // Vertices sit exactly on the surface, so the default GL_LESS
            // depth test would cull them at equal depth; use an inclusive test
            // so they stay visible on top of the model.
            glDepthFunc(GL_LEQUAL);
            mesh_gfx_.draw_vertices();
            glDepthFunc(GL_LESS);

            if(vertices_transparency_ != 0.0f) {
                glDisable(GL_BLEND);
                glDepthMask(GL_TRUE);
            }
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

        mesh_gfx_.set_show_mesh(show_mesh_);
        mesh_gfx_.set_mesh_color(mesh_color_.x, mesh_color_.y, mesh_color_.z);
        mesh_gfx_.set_mesh_width(static_cast<GEO::index_t>(mesh_width_ * 10.0f));

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
                // mesh color.
                mesh_gfx_.set_mesh_color(mesh_color_.x, mesh_color_.y, mesh_color_.z);
            }
        }

        if (surface_transparency_ != 0.0f) {
            glDisable(GL_BLEND);
            glDepthMask(GL_TRUE);
        }
    }

    void MeshObject::draw_edges(
        ) {
        if (show_mesh_)
            mesh_gfx_.draw_edges();
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

            mesh_gfx_.draw_volume();

            mesh_gfx_.set_lighting(lighting);
        }
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
