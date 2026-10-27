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

        virtual void set_initial_points(GEO::index_t points_nb, const double* points);

    protected:
        virtual void initialize_points_locate_element() = 0;

        const std::string attribute_id_;

        GEO::Mesh& mesh_;

        std::vector<double> points_; // 3*points_nb
        std::vector<GEO::index_t> points_locate_element_; // points_nb, may be GEO::NO_FACET or GEO::NO_CELL
    };

    /* ============================================================================================================= */

    class StreamlineSurficialTracing : public StreamlineTracing {
    public:
        explicit StreamlineSurficialTracing(GEO::Mesh& mesh) : StreamlineTracing(mesh) {
            assert(mesh_.facets.nb() > 0);
            mesh_AABB_.initialize(mesh_);
        }

        ~StreamlineSurficialTracing() {
            if (mesh_f_direction_.is_bound())
                mesh_f_direction_.destroy();
        }

        void set_per_vertex_directions(GEO::index_t N, const double* directions);

        void set_per_facet_directions(GEO::index_t N, const double* directions);

    protected:
        void initialize_points_locate_element() override;

        GEO::MeshFacetsAABB mesh_AABB_;
        GEO::Attribute<double> mesh_f_direction_;
    };

    /* ============================================================================================================= */

    class StreamlineVolumetricTracing : public StreamlineTracing {
    public:
        explicit StreamlineVolumetricTracing(GEO::Mesh& mesh) : StreamlineTracing(mesh) {
            assert(mesh_.cells.nb() > 0);
            mesh_AABB_.initialize(mesh_);
        }

        ~StreamlineVolumetricTracing() {
            if (mesh_c_direction_.is_bound())
                mesh_c_direction_.destroy();
        }

        void set_per_vertex_directions(GEO::index_t N, const double* directions);

        void set_per_cell_directions(GEO::index_t N, const double* directions);

    protected:
        void initialize_points_locate_element() override;

        GEO::MeshCellsAABB mesh_AABB_;
        GEO::Attribute<double> mesh_c_direction_;
    };
}

#endif //GEOLIO_STREAMLINE_TRACING_H
