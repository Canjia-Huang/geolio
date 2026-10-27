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
    class StreamlineTracing {
    public:
        explicit StreamlineTracing(GEO::Mesh& mesh) : attribute_id_(generate_random_string(22)), mesh_(mesh) {}

        virtual ~StreamlineTracing() {
            if (mesh_element_direction_.is_bound())
                mesh_element_direction_.destroy();
        }

        virtual void set_initial_points(GEO::index_t points_nb, const double* points);

        void move_forward();

    protected:
        virtual void initialize_points_locate_element() = 0;

        GEO::vec3 get_point_direction(GEO::index_t v);

        virtual void trace_point(GEO::index_t v, double* next_point, GEO::index_t& next_element) = 0;

        const std::string attribute_id_;

        GEO::Mesh& mesh_;

        GEO::index_t points_nb_ = 0;
        GEO::index_t tracing_times_ = 0;
        std::vector<double> points_; // 3*points_nb
        std::vector<GEO::index_t> points_locate_element_; // points_nb, may be GEO::NO_INDEX

        GEO::index_t N_ = 0;
        GEO::Attribute<double> mesh_element_direction_;
    };

    /* ============================================================================================================= */

    class StreamlineSurficialTracing : public StreamlineTracing {
    public:
        explicit StreamlineSurficialTracing(GEO::Mesh& mesh) : StreamlineTracing(mesh) {
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

        ~StreamlineSurficialTracing() override {
            if (mesh_f_unit_normal_.is_bound())
                mesh_f_unit_normal_.destroy();
        }

        void set_per_vertex_directions(GEO::index_t N, const double* directions);

        void set_per_facet_directions(GEO::index_t N, const double* directions);

    protected:
        void initialize_points_locate_element() override;

        void trace_point(GEO::index_t v, double* next_point, GEO::index_t& next_element) override;

        GEO::MeshFacetsAABB mesh_AABB_;
        GEO::Attribute<GEO::vec3> mesh_f_unit_normal_;
    };

    /* ============================================================================================================= */

    class StreamlineVolumetricTracing : public StreamlineTracing {
    public:
        explicit StreamlineVolumetricTracing(GEO::Mesh& mesh) : StreamlineTracing(mesh) {
            assert(mesh_.cells.nb() > 0);
            assert(mesh_.cells.are_simplices());
            mesh_AABB_.initialize(mesh_);
        }

        void set_per_vertex_directions(GEO::index_t N, const double* directions);

        void set_per_cell_directions(GEO::index_t N, const double* directions);

    protected:
        void initialize_points_locate_element() override;

        void trace_point(GEO::index_t v, double* next_point, GEO::index_t& next_element) override;

        GEO::MeshCellsAABB mesh_AABB_;
    };
}

#endif //GEOLIO_STREAMLINE_TRACING_H
