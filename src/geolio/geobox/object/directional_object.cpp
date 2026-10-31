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
    }

    DirectionalObject::DirectionalObject(
        std::string name,
        std::string filepath,
        const GEO::Mesh& mesh,
        const GEO::index_t vectors_nb_per_element,
        const std::vector<double>& vectors,
        const GEO::MeshElementsFlags subelements
        ) : BaseObject(std::move(name), std::move(filepath)),
            vectors_(vectors),
            vectors_nb_per_element_(vectors_nb_per_element),
            subelements_(subelements)
    {
        // The vectors are drawn from the center of the element they belong to.
        const bool volumetric = subelements_ == GEO::MESH_CELLS;
        const GEO::index_t nb = volumetric ? mesh.cells.nb() : mesh.facets.nb();
        centers_.reserve(nb);
        for (GEO::index_t e = 0; e < nb; ++e) {
            centers_.push_back(
                volumetric ? GEO::Geom::mesh_cell_center(mesh, e)
                           : GEO::Geom::mesh_facet_center(mesh, e));
        }

        // Size of the elements, to interpret the length of the vectors (see
        // AxisStyle::length).
        {
            double xyzmin[3] = {0.0, 0.0, 0.0};
            double xyzmax[3] = {0.0, 0.0, 0.0};
            get_bbox(xyzmin, xyzmax);
            double d2 = 0.0;
            for (GEO::coord_index_t c = 0; c < 3; ++c) {
                const double d = xyzmax[c] - xyzmin[c];
                d2 += d * d;
            }
            elements_bbox_diagonal_ =
                d2 > 0.0 ? static_cast<float>(std::sqrt(d2)) : 1.0f;
        }

        // One style per axis, the axes being told apart by their color.
        axis_styles_.resize(vectors_nb_per_element_);
        for (GEO::index_t axis = 0; axis < vectors_nb_per_element_; ++axis) {
            AxisStyle& style = axis_styles_[axis];
            style.color = AXIS_COLORS[axis % NB_AXIS_COLORS];
            style.length = 0.03f;
            style.width = 0.3f;
        }

        if (vectors_.size() != 3 * vectors_nb_per_element_ * nb) {
            LOG::ERROR(
                "Directional field `{}`: {} values for {} elements of {} "
                "vectors, expected {}.",
                name_,
                vectors_.size(),
                nb,
                vectors_nb_per_element_,
                3 * vectors_nb_per_element_ * nb);
            vectors_.clear();
        }

        build_visualization_mesh();
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
        const GEO::index_t nb_edges = nb * vectors_nb_per_element_;
        mesh_.vertices.create_vertices(2 * nb_edges);
        mesh_.edges.create_edges(nb_edges);
        for (GEO::index_t axis = 0; axis < vectors_nb_per_element_; ++axis) {
            for (GEO::index_t e = 0; e < nb; ++e) {
                const double* v = &vectors_[3 * (axis * nb + e)];
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

    void DirectionalObject::draw_object_properties(
        ) {
        // The file the vectors were read from, if any (see
        // BaseObject::draw_object_properties()).
        BaseObject::draw_object_properties();

        ImGui::PushID(this);

        ImGui::Text(
            "%u vectors per %s, %u %s",
            unsigned(vectors_nb_per_element_),
            subelements_ == GEO::MESH_CELLS ? "cell" : "facet",
            unsigned(nb_elements()),
            subelements_ == GEO::MESH_CELLS ? "cells" : "facets");

        // One block per axis: its color, the length of its vectors and the width
        // of the lines they are drawn with.
        for (GEO::index_t axis = 0; axis < vectors_nb_per_element_; ++axis) {
            ImGui::PushID(int(axis));
            ImGui::Separator();
            ImGui::Text("Axis %u", unsigned(axis));

            AxisStyle& style = axis_styles_[axis];
            ImGui::ColorEdit3WithPalette("color", style.color.data());
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Color of the vectors of this axis.");

            ImGui::SetNextItemWidth(-10.0f);
            ImGui::SliderFloat(
                "length", &style.length, 0.0f, 0.5f, "%.3f");
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip(
                    "Length of the vectors, as a fraction of the diagonal of the\n"
                    "bounding box of the field (%.3f).",
                    elements_bbox_diagonal_);
            }

            ImGui::SetNextItemWidth(-10.0f);
            ImGui::SliderFloat("width", &style.width, 0.1f, 2.0f, "%.1f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Width of the lines the vectors are drawn with.");

            ImGui::PopID();
        }

        ImGui::PopID();
    }

    void DirectionalObject::draw_scene(
        const bool lighting
        ) {
        (void)lighting; // the vectors are lines: they are not lit, as the
                        // wireframes of the mesh objects
        if (!renderable() || mesh_.edges.nb() == 0)
            return;

        const GEO::index_t nb = nb_elements();

        // Same GLUP state as the wireframe passes of the mesh objects: thick
        // lines come from the mesh width, with the mesh drawing enabled.
        glupDisable(GLUP_LIGHTING);
        glupEnable(GLUP_DRAW_MESH);

        for (GEO::index_t axis = 0; axis < vectors_nb_per_element_; ++axis) {
            const AxisStyle& style = axis_styles_[axis];
            const double length =
                static_cast<double>(style.length) * elements_bbox_diagonal_;

            glupSetColor3fv(GLUP_FRONT_COLOR, style.color.data());
            glupSetMeshWidth(static_cast<GEO::index_t>(style.width * 10.0f));

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
        if (mesh_.vertices.nb() == 0)
            return;

        // The extent of the vectors, with the length they are displayed with.
        double longest = 0.0;
        for (const AxisStyle& style : axis_styles_)
            longest = std::max(longest, double(style.length));
        longest *= elements_bbox_diagonal_;

        bool first = true;
        const GEO::index_t nb = nb_elements();
        for (GEO::index_t axis = 0; axis < vectors_nb_per_element_; ++axis) {
            for (GEO::index_t e = 0; e < nb; ++e) {
                const GEO::index_t edge = axis * nb + e;
                const double* start = mesh_.vertices.point_ptr(
                    mesh_.edges.vertex(edge, 0));
                const double* direction = mesh_.vertices.point_ptr(
                    mesh_.edges.vertex(edge, 1));
                const double end[3] = {
                    start[0] + longest * (direction[0] - start[0]),
                    start[1] + longest * (direction[1] - start[1]),
                    start[2] + longest * (direction[2] - start[2])
                };
                for (GEO::coord_index_t c = 0; c < 3; ++c) {
                    const double lo = std::min(start[c], end[c]);
                    const double hi = std::max(start[c], end[c]);
                    if (first || lo < xyzmin[c])
                        xyzmin[c] = lo;
                    if (first || hi > xyzmax[c])
                        xyzmax[c] = hi;
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

        if (vectors_nb_per_element != vectors_nb_per_element_ ||
            vectors.size() != 3 * vectors_nb_per_element_ * nb_elements()) {
            LOG::ERROR(
                "`{}` holds {} elements of {} vectors, but the object was "
                "created for {} elements of {} vectors.",
                filepath_,
                vectors_nb_per_element != 0
                    ? vectors.size() / (3 * vectors_nb_per_element)
                    : 0,
                vectors_nb_per_element,
                nb_elements(),
                vectors_nb_per_element_);
            return;
        }

        vectors_ = std::move(vectors);
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
