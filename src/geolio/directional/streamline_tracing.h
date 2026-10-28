//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/27.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_STREAMLINE_TRACING_H
#define GEOLIO_STREAMLINE_TRACING_H
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_AABB.h>
#include <cassert>

#include "geolio/common/utils.h"

namespace geolio
{
    class StreamlineTracer {
    public:
        explicit StreamlineTracer(GEO::Mesh& mesh) : attribute_id_(generate_random_string(22)), mesh_(mesh) {}

        virtual ~StreamlineTracer() {
            if (mesh_element_direction_.is_bound())
                mesh_element_direction_.destroy();
        }

        virtual void set_initial_points(GEO::index_t points_nb, const double* points);

        void trace(GEO::index_t iterations);

        const auto& traced_points() { return traced_points_; }

        void append_tracing_points_to_mesh(
            GEO::Mesh& mesh,
            GEO::Attribute<GEO::index_t>* mesh_v_tracing_times = nullptr,
            GEO::Attribute<GEO::index_t>* mesh_e_point_idx = nullptr) const;

    protected:
        virtual void initialize_points_locate_element() = 0;

        GEO::vec3 get_point_direction(GEO::index_t v);

        virtual void trace_point(GEO::index_t v, GEO::vec3& next_point, GEO::index_t& next_element, GEO::index_t& next_local_idx) = 0;

        const std::string attribute_id_;

        GEO::Mesh& mesh_;

        GEO::index_t points_nb_ = 0;
        GEO::index_t traced_times_ = 0;

        struct TracingPoint {
            GEO::vec3 point;
            GEO::index_t element = GEO::NO_INDEX; // f / c
            GEO::index_t local_idx = GEO::NO_INDEX; // lv / lf
        };
        std::vector<TracingPoint> traced_points_; // points_nb * tracing_times

        GEO::index_t N_ = 0;
        GEO::Attribute<double> mesh_element_direction_;
    };

    /* ============================================================================================================= */

    class StreamlineSurficialTracer : public StreamlineTracer {
    public:
        explicit StreamlineSurficialTracer(GEO::Mesh& mesh) : StreamlineTracer(mesh) {
            assert(mesh_.vertices.dimension() > 2);
            assert(mesh_.facets.nb() > 0);
            assert(mesh_.facets.are_simplices());
            mesh_AABB_.initialize(mesh_);

            mesh_f_unit_normal_.bind(mesh_.facets.attributes(), attribute_id_+":unit_normal");
            for (const auto& f : mesh_.facets)
                mesh_f_unit_normal_[f] = GEO::normalize(GEO::Geom::triangle_normal(
                    mesh_.facets.point(f, 0),
                    mesh_.facets.point(f, 1),
                    mesh_.facets.point(f, 2)));
        }

        ~StreamlineSurficialTracer() override {
            if (mesh_f_unit_normal_.is_bound())
                mesh_f_unit_normal_.destroy();
        }

        void set_per_vertex_directions(GEO::index_t N, const double* directions);

        void set_per_facet_directions(GEO::index_t N, const double* directions);

        void smooth(GEO::index_t iterations = 3, double damping = 0.5, bool constraint_on_surface = true);

    protected:
        void initialize_points_locate_element() override;

        void trace_point(GEO::index_t v, GEO::vec3& next_point, GEO::index_t& next_element, GEO::index_t& next_local_idx) override;

        GEO::MeshFacetsAABB mesh_AABB_;
        GEO::Attribute<GEO::vec3> mesh_f_unit_normal_;
    };

    /* ============================================================================================================= */

    class StreamlineVolumetricTracer : public StreamlineTracer {
    public:
        explicit StreamlineVolumetricTracer(GEO::Mesh& mesh) : StreamlineTracer(mesh) {
            assert(mesh_.cells.nb() > 0);
            assert(mesh_.cells.are_simplices());
            mesh_AABB_.initialize(mesh_);
        }

        void set_per_vertex_directions(GEO::index_t N, const double* directions);

        void set_per_cell_directions(GEO::index_t N, const double* directions);

        void smooth(GEO::index_t iterations = 3, double damping = 0.5);

    protected:
        void initialize_points_locate_element() override;

        void trace_point(GEO::index_t v, GEO::vec3& next_point, GEO::index_t& next_element, GEO::index_t& next_local_idx) override;

        GEO::MeshCellsAABB mesh_AABB_;
    };
}

#endif //GEOLIO_STREAMLINE_TRACING_H
