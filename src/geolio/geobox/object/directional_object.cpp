//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/31.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "directional_object.h"

#include <algorithm>
#include <cmath>
#include <geogram/mesh/mesh_geometry.h>
#include <geogram_gfx/GLUP/GLUP.h>
#include <geogram_gfx/imgui_ext/imgui_ext.h>
#include <geogram_gfx/imgui_ext/icon_font.h>

#include <geolio/common/log.h>
#include <geolio/geobox/object/mesh_object.h>
#include <geolio/io/fra_io.h>

namespace geolio::geobox
{
    namespace
    {
        /** @brief Colors the axes of the field are drawn with by default. */
        const GEO::vec4f AXIS_COLORS[] = {
            GEO::vec4f(0.90f, 0.10f, 0.10f, 1.0f),
            GEO::vec4f(0.10f, 0.70f, 0.10f, 1.0f),
            GEO::vec4f(0.10f, 0.30f, 0.90f, 1.0f)
        };

        /** @brief Number of default colors above. */
        constexpr GEO::index_t NB_AXIS_COLORS =
            sizeof(AXIS_COLORS) / sizeof(AXIS_COLORS[0]);

        /** @brief Name of the elements a field can be attached to. */
        const char* subelements_label(const GEO::MeshElementsFlags subelements) {
            switch (subelements) {
                case GEO::MESH_VERTICES:
                    return "vertices";
                case GEO::MESH_FACETS:
                    return "facets";
                case GEO::MESH_CELLS:
                    return "cells";
                default:
                    return "nothing";
            }
        }
    }

    DirectionalObject::DirectionalObject(
        std::string name,
        std::string filepath,
        ApplicationObjects& objects,
        const GEO::index_t vectors_nb_per_element,
        const std::vector<double>& vectors
        ) : BaseObject(std::move(name), std::move(filepath)),
            vectors_(vectors),
            vectors_nb_per_element_(vectors_nb_per_element),
            objects_(objects)
    {
        // One style per axis, the axes being told apart by their color.
        axis_styles_.resize(vectors_nb_per_element_);
        for (GEO::index_t axis = 0; axis < vectors_nb_per_element_; ++axis)
            axis_styles_[axis].color = AXIS_COLORS[axis % NB_AXIS_COLORS];

        // Nothing is drawn until the field is attached to a mesh object (see
        // attach_to()): the .fra format holds the vectors only.
        build_visualization_mesh();
    }

    GEO::index_t DirectionalObject::nb_frames(
        ) const {
        if (vectors_nb_per_element_ == 0)
            return 0;
        return GEO::index_t(vectors_.size() / (3 * vectors_nb_per_element_));
    }

    std::vector<GEO::MeshElementsFlags> DirectionalObject::attachable_subelements(
        const MeshObject& mesh_object
        ) const {
        std::vector<GEO::MeshElementsFlags> kinds;
        const GEO::index_t nb = nb_frames();
        if (nb == 0)
            return kinds;

        const GEO::Mesh& mesh = mesh_object.mesh();
        if (mesh.vertices.nb() == nb)
            kinds.push_back(GEO::MESH_VERTICES);
        if (mesh.facets.nb() == nb)
            kinds.push_back(GEO::MESH_FACETS);
        if (mesh.cells.nb() == nb)
            kinds.push_back(GEO::MESH_CELLS);
        return kinds;
    }

    bool DirectionalObject::attached(
        ) const {
        return subelements_ != GEO::MESH_NONE &&
               !centers_.empty() &&
               !mesh_object_.expired();
    }

    bool DirectionalObject::attach_to(
        const std::shared_ptr<MeshObject>& mesh_object,
        const GEO::MeshElementsFlags subelements
        ) {
        if (mesh_object == nullptr || subelements == GEO::MESH_NONE) {
            mesh_object_.reset();
            subelements_ = GEO::MESH_NONE;
            centers_.clear();
            build_visualization_mesh();
            return false;
        }

        // The frames need one element to sit on each.
        const GEO::Mesh& mesh = mesh_object->mesh();
        GEO::index_t nb_subelements = 0;
        switch (subelements) {
            case GEO::MESH_VERTICES:
                nb_subelements = mesh.vertices.nb();
                break;
            case GEO::MESH_FACETS:
                nb_subelements = mesh.facets.nb();
                break;
            case GEO::MESH_CELLS:
                nb_subelements = mesh.cells.nb();
                break;
            default:
                break;
        }
        if (nb_subelements != nb_frames()) {
            LOG::ERROR(
                "`{}` has {} {} for the {} frames of `{}`: the field is not "
                "attached to it.",
                mesh_object->name(),
                nb_subelements,
                subelements_label(subelements),
                nb_frames(),
                name_);
            return false;
        }

        mesh_object_ = mesh_object;
        subelements_ = subelements;
        read_centers();
        build_visualization_mesh();
        return true;
    }

    void DirectionalObject::read_centers(
        ) {
        centers_.clear();
        elements_bbox_diagonal_ = 1.0f;

        const auto mesh_object = mesh_object_.lock();
        if (mesh_object == nullptr)
            return;
        const GEO::Mesh& mesh = mesh_object->mesh();

        // The frames are drawn from the center of the element they sit on.
        const GEO::index_t nb = nb_frames();
        centers_.reserve(nb);
        switch (subelements_) {
            case GEO::MESH_VERTICES:
                for (GEO::index_t v = 0; v < nb; ++v)
                    centers_.push_back(mesh.vertices.point(v));
                break;
            case GEO::MESH_FACETS:
                for (GEO::index_t f = 0; f < nb; ++f)
                    centers_.push_back(GEO::Geom::mesh_facet_center(mesh, f));
                break;
            case GEO::MESH_CELLS:
                for (GEO::index_t c = 0; c < nb; ++c)
                    centers_.push_back(GEO::Geom::mesh_cell_center(mesh, c));
                break;
            default:
                break;
        }

        // The size of the mesh, to interpret the length of the vectors (see
        // length()).
        double xyzmin[3] = {0.0, 0.0, 0.0};
        double xyzmax[3] = {0.0, 0.0, 0.0};
        bool first = true;
        for (GEO::index_t v = 0; v < mesh.vertices.nb(); ++v) {
            const double* p = mesh.vertices.point_ptr(v);
            for (GEO::coord_index_t c = 0; c < 3; ++c) {
                if (first || p[c] < xyzmin[c])
                    xyzmin[c] = p[c];
                if (first || p[c] > xyzmax[c])
                    xyzmax[c] = p[c];
            }
            first = false;
        }
        double d2 = 0.0;
        for (GEO::coord_index_t c = 0; c < 3; ++c) {
            const double d = xyzmax[c] - xyzmin[c];
            d2 += d * d;
        }
        elements_bbox_diagonal_ =
            d2 > 0.0 ? static_cast<float>(std::sqrt(d2)) : 1.0f;
    }

    void DirectionalObject::build_visualization_mesh(
        ) {
        mesh_.clear();
        mesh_.vertices.set_dimension(3);

        const GEO::index_t nb = nb_elements();
        // Nothing to draw when the field does not describe the mesh it was given
        // (the constructor empties it in that case).
        if (nb == 0 || vectors_nb_per_element_ == 0 ||
            vectors_.size() != 3 * vectors_nb_per_element_ * nb) {
            return;
        }

        // One edge per vector: from the center of the element, along the unit
        // direction of the vector. The length is applied while drawing (see
        // draw_scene()), so that it can be changed without touching the mesh.
        // The vectors are stored element by element -- the N vectors of an
        // element are 3*N consecutive coordinates -- while the edges are stored
        // axis by axis, so that drawing an axis reads a contiguous range.
        const GEO::index_t nb_edges = nb * vectors_nb_per_element_;
        mesh_.vertices.create_vertices(2 * nb_edges);
        mesh_.edges.create_edges(nb_edges);
        for (GEO::index_t axis = 0; axis < vectors_nb_per_element_; ++axis) {
            for (GEO::index_t e = 0; e < nb; ++e) {
                const double* v =
                    &vectors_[3 * (e * vectors_nb_per_element_ + axis)];
                const double norm =
                    std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
                const double inv_norm = norm > 0.0 ? 1.0 / norm : 0.0;

                const GEO::index_t edge = axis * nb + e;
                const GEO::index_t start = 2 * edge;
                mesh_.edges.set_vertex(edge, 0, start);
                mesh_.edges.set_vertex(edge, 1, start + 1);

                const GEO::vec3& center = centers_[e];
                mesh_.vertices.point(start) = center;
                mesh_.vertices.point(start + 1) = GEO::vec3(
                    center[0] + inv_norm * v[0],
                    center[1] + inv_norm * v[1],
                    center[2] + inv_norm * v[2]);
            }
        }
    }

    double DirectionalObject::length_in_model_units(
        ) const {
        return static_cast<double>(length_) * elements_bbox_diagonal_;
    }

    void DirectionalObject::draw_object_properties(
        ) {
        ImGui::PushID(this);

        // The file the vectors were read from, if any (see
        // BaseObject::draw_object_properties()).
        BaseObject::draw_object_properties();

        // The mesh object pickers are the way to give the field something to be
        // drawn on: they are open by default while nothing is attached, and
        // folded away once the field is drawn.
        if (ImGui::CollapsingHeader("Info", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Text(
                "%u vectors per element, %u frames",
                static_cast<unsigned>(vectors_nb_per_element_),
                static_cast<unsigned>(nb_frames()));

            ImGui::Separator();

            // Where the frames come from: the mesh object they are attached to, and
            // which of its elements they sit on (see attach_to()).
            draw_attachment_pickers();
        }

        // Nothing is drawn while the field is not attached, so there is nothing
        // to style either.
        if (attached() &&
            ImGui::CollapsingHeader("Render", ImGuiTreeNodeFlags_DefaultOpen)) {
            // Whether the vectors are drawn in both directions: a, -a, b, -b, ...
            // With it, the axes of a frame are lines through the center of their
            // element instead of being oriented.
            ImGui::Checkbox("draw opposite", &draw_opposite_);
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip(
                    "Draw each vector in both directions: a and -a, b and -b, ...");
            }

            // The length and the width are the same for all the axes.
            ImGui::DragFloat("len.", &length_, 0.001f, 0.0f, 1e8, "%.3f");
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip(
                    "Length of the vectors, as a fraction of the diagonal of the mesh the field is defined on (%.3f).",
                    elements_bbox_diagonal_);
            }

            ImGui::DragFloat("wid.", &width_, 0.01f, 0.0f, 1e8, "%.2f");
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip(
                    "Width of the lines the vectors are drawn with.");
            }

            ImGui::Separator();

            // One row per axis: whether it is drawn, and its color.
            for (GEO::index_t axis = 0; axis < vectors_nb_per_element_; ++axis) {
                ImGui::PushID(static_cast<int>(axis));

                AxisStyle& style = axis_styles_[axis];
                ImGui::Checkbox("##visible", &style.visible);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Show the vectors of this axis.");

                ImGui::SameLine();

                ImGui::ColorEdit3WithPalette("#color", style.color.data());
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Color of the vectors of this axis.");

                ImGui::SameLine();

                ImGui::Text("Axis %u", static_cast<unsigned>(axis));

                ImGui::PopID();
            }
        }

        ImGui::PopID();
    }

    bool DirectionalObject::draw_attachment_pickers(
        ) {
        bool attached = false;

        // The objects the field can be attached to: those with as many vertices,
        // facets or cells as it has frames.
        std::vector<std::shared_ptr<MeshObject>> candidates;
        for (const auto& object : objects_) {
            const auto mesh_object = std::dynamic_pointer_cast<MeshObject>(object);
            if (mesh_object == nullptr ||
                attachable_subelements(*mesh_object).empty()) {
                continue;
            }
            candidates.push_back(mesh_object);
        }

        if (candidates.empty()) {
            ImGui::TextDisabled(
                "No matching mesh object.",
                static_cast<unsigned>(nb_frames()));
            return attached;
        }

        // A single mesh object, with a single kind of element to attach to, is
        // picked automatically: the field shows up as soon as it is created.
        auto selected = mesh_object_.lock();
        if (selected == nullptr && candidates.size() == 1) {
            const auto kinds = attachable_subelements(*candidates[0]);
            if (kinds.size() == 1) {
                attached = attach_to(candidates[0], kinds[0]);
                selected = candidates[0];
            }
        }

        // The mesh object: its name, or a placeholder while none is attached.
        std::string preview =
            selected != nullptr ? selected->name() : "Select a mesh object";
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted("Mesh");
        ImGui::SameLine();
        if (ImGui::BeginCombo("##attach_mesh", preview.c_str())) {
            for (const auto& candidate : candidates) {
                const bool is_selected = candidate == selected;
                if (ImGui::Selectable(candidate->name().c_str(), is_selected)) {
                    // Keep the kind of element of the previous mesh when it
                    // still works, so switching between two meshes of the same
                    // kind does not lose the choice.
                    const auto kinds = attachable_subelements(*candidate);
                    const auto current = std::ranges::find(kinds, subelements_);
                    attached = attach_to(
                        candidate,
                        current != kinds.end() ? subelements_ : kinds.front());
                    selected = candidate;
                }
                if (is_selected)
                    ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }

        // The kind of element the frames sit on. It is picked when the mesh
        // offers several kinds with the right number of elements, and shown --
        // there is nothing to pick -- when it offers a single one, so that the
        // panel always says where the frames sit.
        if (selected != nullptr) {
            const auto kinds = attachable_subelements(*selected);
            if (kinds.size() == 1) {
                ImGui::AlignTextToFramePadding();
                ImGui::TextUnformatted("Attach to");
                ImGui::SameLine();
                ImGui::TextDisabled(
                    "%s (%u)",
                    subelements_label(kinds.front()),
                    static_cast<unsigned>(nb_frames()));
            }
            if (kinds.size() > 1) {
                ImGui::AlignTextToFramePadding();
                ImGui::TextUnformatted("Attach to");
                ImGui::SameLine();
                std::string kind_preview = subelements_label(subelements_);
                if (ImGui::BeginCombo("##attach_kind", kind_preview.c_str())) {
                    for (const auto kind : kinds) {
                        const bool is_selected = kind == subelements_;
                        if (ImGui::Selectable(subelements_label(kind), is_selected)) {
                            attached = attach_to(selected, kind);
                        }
                        if (is_selected)
                            ImGui::SetItemDefaultFocus();
                    }
                    ImGui::EndCombo();
                }
            }
        }

        return attached;
    }

    void DirectionalObject::draw_scene(
        const bool lighting
        ) {
        (void)lighting; // the vectors are lines: they are not lit, as the
                        // wireframes of the mesh objects
        // The frames sit on the elements of a mesh object: without one (or once
        // it was removed from the scene) there is nowhere to draw them.
        if (!attached() || !renderable() || mesh_.edges.nb() == 0)
            return;

        const GEO::index_t nb = nb_elements();

        // Same GLUP state as the wireframe passes of the mesh objects: thick
        // lines come from the mesh width, with the mesh drawing enabled.
        glupDisable(GLUP_LIGHTING);
        glupEnable(GLUP_DRAW_MESH);

        const double length = length_in_model_units();

        for (GEO::index_t axis = 0; axis < vectors_nb_per_element_; ++axis) {
            const AxisStyle& style = axis_styles_[axis];
            if (!style.visible)
                continue;

            // The width of the lines comes from the mesh width, which GLUP only
            // applies while drawing the mesh: the lines are then painted with the
            // *mesh* color, so both colors are set -- setting the front color
            // only left the axes in the color of whatever was drawn before.
            glupSetColor4fv(GLUP_FRONT_COLOR, style.color.data());
            glupSetColor4fv(GLUP_MESH_COLOR, style.color.data());
            glupSetMeshWidth(static_cast<GEO::index_t>(width_ * 10.0f));

            glupBegin(GLUP_LINES);
            for (GEO::index_t e = 0; e < nb; ++e) {
                const GEO::index_t edge = axis * nb + e;
                const double* start = mesh_.vertices.point_ptr(
                    mesh_.edges.vertex(edge, 0));
                const double* direction = mesh_.vertices.point_ptr(
                    mesh_.edges.vertex(edge, 1));

                // The stored edge is one unit long: the axis length scales it.
                const double end[3] = {
                    start[0] + length * (direction[0] - start[0]),
                    start[1] + length * (direction[1] - start[1]),
                    start[2] + length * (direction[2] - start[2])
                };
                glupVertex3dv(start);
                glupVertex3dv(end);
                if (draw_opposite_) {
                    // The same vector, from the center the other way: the line
                    // drawn for one axis goes through the center.
                    const double opposite[3] = {
                        2.0 * start[0] - end[0],
                        2.0 * start[1] - end[1],
                        2.0 * start[2] - end[2]
                    };
                    glupVertex3dv(start);
                    glupVertex3dv(opposite);
                }
            }
            glupEnd();
        }

        glupDisable(GLUP_DRAW_MESH);
    }

    void DirectionalObject::get_bbox(
        double* xyzmin,
        double* xyzmax
        ) const {
        for (GEO::coord_index_t c = 0; c < 3; ++c) {
            xyzmin[c] = 0.0;
            xyzmax[c] = 0.0;
        }
        if (!attached() || mesh_.vertices.nb() == 0)
            return;

        // The extent of the vectors that are displayed, with the length they are
        // displayed with. With draw_opposite() they also extend on the other
        // side of the center of their element.
        const double length = length_in_model_units();
        const int nb_sides = draw_opposite_ ? 2 : 1;

        bool first = true;
        const GEO::index_t nb = nb_elements();
        for (GEO::index_t axis = 0; axis < vectors_nb_per_element_; ++axis) {
            if (!axis_styles_[axis].visible)
                continue;
            for (GEO::index_t e = 0; e < nb; ++e) {
                const GEO::index_t edge = axis * nb + e;
                const double* start = mesh_.vertices.point_ptr(
                    mesh_.edges.vertex(edge, 0));
                const double* direction = mesh_.vertices.point_ptr(
                    mesh_.edges.vertex(edge, 1));
                for (int side = 0; side < nb_sides; ++side) {
                    const double sign = side == 0 ? 1.0 : -1.0;
                    const double end[3] = {
                        start[0] + sign * length * (direction[0] - start[0]),
                        start[1] + sign * length * (direction[1] - start[1]),
                        start[2] + sign * length * (direction[2] - start[2])
                    };
                    for (GEO::coord_index_t c = 0; c < 3; ++c) {
                        const double lo = std::min(start[c], end[c]);
                        const double hi = std::max(start[c], end[c]);
                        if (first || lo < xyzmin[c])
                            xyzmin[c] = lo;
                        if (first || hi > xyzmax[c])
                            xyzmax[c] = hi;
                    }
                }
                first = false;
            }
        }
    }

    void DirectionalObject::reload(
        ) {
        // The .fra format holds the vectors only: the geometry the object was
        // created with is the one they are drawn on.
        if (filepath_.empty()) {
            LOG::WARN("`{}` has no file to reload its vectors from.", name_);
            return;
        }

        GEO::index_t vectors_nb_per_element = 0;
        std::vector<double> vectors;
        if (!fra_load(filepath_, vectors_nb_per_element, vectors)) {
            LOG::ERROR("Could not reload the vectors of `{}`!", name_);
            return;
        }

        // The file has to hold whole frames of as many vectors as the object
        // draws per frame; how many frames there are is free to change, since the
        // mesh the field is attached to can follow (see attach_to()).
        if (vectors_nb_per_element != vectors_nb_per_element_ ||
            (vectors_nb_per_element_ != 0 &&
             vectors.size() % (3 * vectors_nb_per_element_) != 0)) {
            LOG::ERROR(
                "`{}` holds {} values of {} vectors per element, but the object "
                "was created for {} vectors per element.",
                filepath_,
                vectors.size(),
                vectors_nb_per_element,
                vectors_nb_per_element_);
            return;
        }

        vectors_ = std::move(vectors);

        // The frames were read for the mesh they are attached to: they may not
        // describe it any more, in which case the field is left unattached rather
        // than drawn on the wrong elements.
        if (const auto mesh_object = mesh_object_.lock();
            mesh_object != nullptr && !attach_to(mesh_object, subelements_)) {
            mesh_object_.reset();
            subelements_ = GEO::MESH_NONE;
        }
        read_centers();
        build_visualization_mesh();
        LOG::INFO("Reloaded the vectors of `{}`.", name_);
    }

    bool DirectionalObject::save(
        const std::string& filename
        ) {
        // The vectors, with the header and the element/vector counts, which
        // fra_load() understands (see geolio::fra_save()).
        return fra_save(
            filename, vectors_nb_per_element_, vectors_, true, true);
    }

    bool DirectionalObject::can_write_extension(
        const std::string& extension
        ) const {
        return extension == "fra";
    }
}
