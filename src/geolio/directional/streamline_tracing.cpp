//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/27.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "streamline_tracing.h"

namespace geolio
{
    void StreamlineTracing::set_initial_points(
        const GEO::index_t points_nb,
        const double* points
        ) {
        assert(points_nb > 0);

        points_.resize(3*points_nb);
        std::ranges::copy_n(points, 3*points_nb, points_.data());

        initialize_points_locate_element();
    }

    /* ============================================================================================================= */

    void StreamlineSurficialTracing::set_per_vertex_directions(
        const GEO::index_t N,
        const double* directions
        ) {
        // TODO
    }

    void StreamlineSurficialTracing::set_per_facet_directions(
        const GEO::index_t N,
        const double* directions
        ) {
        assert(N > 0);

        if (mesh_f_direction_.is_bound() && mesh_f_direction_.dimension() != 3*N)
            mesh_f_direction_.destroy();
        if (!mesh_f_direction_.is_bound())
            mesh_f_direction_.create_vector_attribute(mesh_.facets.attributes(), attribute_id_+":direction", 3*N);

        std::ranges::copy_n(directions, 3*N*mesh_.facets.nb(), mesh_f_direction_.data());
    }

    void StreamlineSurficialTracing::initialize_points_locate_element(
        ) {
        const GEO::index_t POINTS_NB = points_.size()/3;
        points_locate_element_.assign(POINTS_NB, GEO::NO_FACET);
        for (GEO::index_t v = 0; v < POINTS_NB; ++v) {
            const GEO::vec3 p(points_[3*v], points_[3*v+1], points_[3*v+2]);

            GEO::vec3 nearest_p;
            double sq_dist;
            points_locate_element_[v] = mesh_AABB_.nearest_facet(p, nearest_p, sq_dist);

            points_[3*v] = nearest_p[0];
            points_[3*v+1] = nearest_p[1];
            points_[3*v+2] = nearest_p[2];
        }
    }

    /* ============================================================================================================= */

    void StreamlineVolumetricTracing::set_per_vertex_directions(
        const GEO::index_t N,
        const double* directions
        ) {
        // TODO
    }

    void StreamlineVolumetricTracing::set_per_cell_directions(
        const GEO::index_t N,
        const double* directions
        ) {
        assert(N > 0);

        if (mesh_c_direction_.is_bound() && mesh_c_direction_.dimension() != 3*N)
            mesh_c_direction_.destroy();
        if (!mesh_c_direction_.is_bound())
            mesh_c_direction_.create_vector_attribute(mesh_.cells.attributes(), attribute_id_+":direction", 3*N);

        std::ranges::copy_n(directions, 3*N*mesh_.cells.nb(), mesh_c_direction_.data());
    }

    void StreamlineVolumetricTracing::initialize_points_locate_element(
        ) {
        const GEO::index_t POINTS_NB = points_.size()/3;
        points_locate_element_.assign(POINTS_NB, GEO::NO_FACET);
        for (GEO::index_t v = 0; v < POINTS_NB; ++v) {
            const GEO::vec3 p(points_[3*v], points_[3*v+1], points_[3*v+2]);

            points_locate_element_[v] = mesh_AABB_.containing_tet(p);
        }
    }
}