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

        points_nb_ = points_nb;
        tracing_times_ = 0;

        points_.resize(3*points_nb_);
        std::ranges::copy_n(points, 3*points_nb_, points_.data());

        initialize_points_locate_element();
    }

    void StreamlineTracing::move_forward(
        ) {
        const GEO::index_t p_start_idx = 3*points_nb_*tracing_times_;

        points_.reserve(points_.size()+3*points_nb_);
        for (GEO::index_t v = 0; v < points_nb_; ++v) {
            /* Pre-allocated */
            points_.emplace_back();
            points_.emplace_back();
            points_.emplace_back();
            points_locate_element_.emplace_back();

            /* Trace */
            trace_point(v, &points_[p_start_idx+3*v], points_locate_element_[v]);
        }

        ++tracing_times_;
    }

    GEO::vec3 StreamlineTracing::get_point_direction(
        const GEO::index_t v
        ) {
        assert(v < points_nb_);
        assert(N_ > 0);
        assert(mesh_element_direction_.is_bound());
        assert(mesh_element_direction_.dimension() == 3*N_);

        const auto& ele = points_locate_element_[v];
        if (ele == GEO::NO_INDEX)
            return {0, 0, 0};

        const GEO::index_t fd_idx = 3*N_*ele;

        if (tracing_times_ == 0 || N_ == 1) // Initially, a direction is randomly selected (take the first)
            return {mesh_element_direction_[fd_idx], mesh_element_direction_[fd_idx+1], mesh_element_direction_[fd_idx+2]};

        /* Find the closet direction */
        const GEO::index_t p0_idx = 3*points_nb_*(tracing_times_-1);
        const GEO::index_t p1_idx = 3*points_nb_*tracing_times_;
        const GEO::vec3 p0(points_[p0_idx+3*v], points_[p0_idx+3*v+1], points_[p0_idx+3*v+2]);
        const GEO::vec3 p1(points_[p1_idx+3*v], points_[p1_idx+3*v+1], points_[p1_idx+3*v+2]);
        const GEO::vec3 moving_dir = p1-p0;
        GEO::vec3 closet_dir;
        double closet_angle = M_PI;
        for (GEO::index_t n = 0; n < N_; ++n) {
            const GEO::vec3 dir(mesh_element_direction_[fd_idx+3*n], mesh_element_direction_[fd_idx+3*n+1], mesh_element_direction_[fd_idx+3*n+2]);
            if (const double angle = GEO::Geom::angle(moving_dir, dir);
                angle < closet_angle
                ) {
                closet_dir = dir;
                closet_angle = angle;
            }
        }
        return closet_dir;
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

        N_ = N;

        if (mesh_element_direction_.is_bound() && mesh_element_direction_.dimension() != 3*N_)
            mesh_element_direction_.destroy();
        if (!mesh_element_direction_.is_bound())
            mesh_element_direction_.create_vector_attribute(mesh_.facets.attributes(), attribute_id_+":direction", 3*N_);

        std::ranges::copy_n(directions, 3*N_*mesh_.facets.nb(), mesh_element_direction_.data());
    }

    void StreamlineSurficialTracing::initialize_points_locate_element(
        ) {
        points_locate_element_.assign(points_nb_, GEO::NO_FACET);
        for (GEO::index_t v = 0; v < points_nb_; ++v) {
            const GEO::vec3 p(points_[3*v], points_[3*v+1], points_[3*v+2]);

            GEO::vec3 nearest_p;
            double sq_dist;
            points_locate_element_[v] = mesh_AABB_.nearest_facet(p, nearest_p, sq_dist);

            points_[3*v] = nearest_p[0];
            points_[3*v+1] = nearest_p[1];
            points_[3*v+2] = nearest_p[2];
        }
    }

    void StreamlineSurficialTracing::trace_point(
        const GEO::index_t v,
        double* next_point,
        GEO::index_t& next_element
        ) {
        assert(v < points_nb_);

        const GEO::index_t p_start_idx = 3*points_nb_*tracing_times_;
        const GEO::vec3 p0(points_[p_start_idx], points_[p_start_idx+1], points_[p_start_idx+2]);

        /* Init */
        next_point[0] = p0[0];
        next_point[1] = p0[1];
        next_point[2] = p0[2];
        next_element = GEO::NO_FACET;

        if (const auto& f = points_locate_element_[v];
            f != GEO::NO_FACET
            ) {
            assert(f < mesh_.facets.nb());

            GEO::vec3 d = get_point_direction(v);

            /* Compute tangential direction */
            d = d - GEO::dot(d, mesh_f_unit_normal_[f])*mesh_f_unit_normal_[f];
            const double d_norm = GEO::length(d);
            if (d_norm < 1e-10) // cannot find a valid direction!
                return;
            d /= d_norm;

            /* Find intersecting edge */
            double max_t = -std::numeric_limits<double>::max();
            GEO::index_t max_lv = GEO::NO_INDEX;
            for (GEO::index_t lv = 0; lv < 3; ++lv) {
                const auto& ep0 = mesh_.facets.point(f, lv);
                const auto& ep1 = mesh_.facets.point(f, (lv+1)%3);
                const GEO::vec3 edge = ep1-ep0;

                const double s_d = GEO::dot(GEO::cross(edge, d), mesh_f_unit_normal_[f]);
                if (std::abs(s_d) < 1e-10) // edge is parallel to d -> skip this edge
                    continue;

                if (const double t = GEO::dot(GEO::cross(p0-ep0, edge), mesh_f_unit_normal_[f]) / s_d;
                    t > max_t
                    ) {
                    max_t = t;
                    max_lv = lv;
                }
            }
            assert(max_lv != GEO::NO_INDEX);

            /* Output */
            const GEO::vec3 p1 = p0+max_t*d;
            next_point[0] = p1[0];
            next_point[1] = p1[1];
            next_point[2] = p1[2];
            next_element = mesh_.facets.adjacent(f, max_lv);
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

        N_ = N;

        if (mesh_element_direction_.is_bound() && mesh_element_direction_.dimension() != 3*N_)
            mesh_element_direction_.destroy();
        if (!mesh_element_direction_.is_bound())
            mesh_element_direction_.create_vector_attribute(mesh_.cells.attributes(), attribute_id_+":direction", 3*N_);

        std::ranges::copy_n(directions, 3*N_*mesh_.cells.nb(), mesh_element_direction_.data());
    }

    void StreamlineVolumetricTracing::initialize_points_locate_element(
        ) {
        points_locate_element_.assign(points_nb_, GEO::NO_FACET);
        for (GEO::index_t v = 0; v < points_nb_; ++v) {
            const GEO::vec3 p(points_[3*v], points_[3*v+1], points_[3*v+2]);

            points_locate_element_[v] = mesh_AABB_.containing_tet(p);
        }
    }

    void StreamlineVolumetricTracing::trace_point(
        const GEO::index_t v,
        double* next_point,
        GEO::index_t& next_element
        ) {
        // TODO
    }
}