//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/27.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "streamline_tracing.h"

#include <geogram/basic/geometry_nd.h>

namespace geolio
{
    void StreamlineTracer::set_initial_points(
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

    void StreamlineTracer::trace(
        const GEO::index_t iterations
        ) {
        assert(iterations > 0);

        points_.resize(points_.size() + 3*points_nb_ * iterations);
        points_locate_element_.resize(points_locate_element_.size() + points_nb_ * iterations);
        for (GEO::index_t iter = 0; iter < iterations; ++iter) {
            const GEO::index_t next_p_start_idx = 3*points_nb_*(tracing_times_+1);
            const GEO::index_t next_ele_start_idx = points_nb_*(tracing_times_+1);

            for (GEO::index_t v = 0; v < points_nb_; ++v) {
                /* Trace */
                GEO::vec3 next_point;
                GEO::index_t next_element;
                GEO::index_t next_local_idx;
                trace_point(v, next_point, next_element, next_local_idx);
                points_[next_p_start_idx + 3*v]   = next_point[0];
                points_[next_p_start_idx + 3*v+1] = next_point[1];
                points_[next_p_start_idx + 3*v+2] = next_point[2];
                points_locate_element_[next_ele_start_idx + v].first = next_element;
                points_locate_element_[next_ele_start_idx + v].second = next_local_idx;
            }

            ++tracing_times_;
        }
    }

    void StreamlineTracer::append_tracing_points_to_mesh(
        GEO::Mesh& mesh,
        GEO::Attribute<GEO::index_t>* mesh_v_tracing_times,
        GEO::Attribute<GEO::index_t>* mesh_e_point_idx
        ) const {
        if (mesh.vertices.dimension() < 3)
            mesh.vertices.set_dimension(3);

        GEO::index_t new_v = mesh.vertices.create_vertices(points_nb_*(tracing_times_+1));
        GEO::index_t new_e = mesh.edges.create_edges(points_nb_*tracing_times_);

        if (mesh_v_tracing_times != nullptr) {
            assert(mesh_v_tracing_times->is_bound());
            assert(mesh_v_tracing_times->size() == mesh.vertices.nb());
        }
        if (mesh_e_point_idx != nullptr) {
            assert(mesh_e_point_idx->is_bound());
            assert(mesh_e_point_idx->size() == mesh.edges.nb());
        }

        for (GEO::index_t v = 0; v < points_nb_; ++v) {
            for (GEO::index_t n = 0; n < tracing_times_+1; ++n) {
                double* p = mesh.vertices.point_ptr(new_v+n);
                p[0] = points_[3*points_nb_*n + 3*v];
                p[1] = points_[3*points_nb_*n + 3*v+1];
                p[2] = points_[3*points_nb_*n + 3*v+2];

                if (mesh_v_tracing_times != nullptr)
                    (*mesh_v_tracing_times)[new_v+n] = n;

                if (n < tracing_times_) {
                    mesh.edges.set_vertex(new_e, 0, new_v+n);
                    mesh.edges.set_vertex(new_e, 1, new_v+n+1);

                    if (mesh_e_point_idx != nullptr)
                        (*mesh_e_point_idx)[new_e] = v;

                    ++new_e;
                }
            }
            new_v += tracing_times_+1;
        }
    }

    GEO::vec3 StreamlineTracer::get_point_direction(
        const GEO::index_t v
        ) {
        assert(v < points_nb_);
        assert(N_ > 0);
        assert(mesh_element_direction_.is_bound());
        assert(mesh_element_direction_.dimension() == 3*N_);

        const auto& ele = points_locate_element_[points_nb_*tracing_times_ + v].first;
        if (ele == GEO::NO_INDEX)
            return {0, 0, 0};

        const GEO::index_t fd_idx = 3*N_*ele;

        if (tracing_times_ == 0 || N_ == 1) { // Initially, a direction is randomly selected (take the first valid one)
            for (GEO::index_t n = 0; n < N_; ++n) {
                if (const GEO::vec3 dir(mesh_element_direction_[fd_idx+3*n], mesh_element_direction_[fd_idx+3*n+1], mesh_element_direction_[fd_idx+3*n+2]);
                    GEO::length(dir) > 1e-12)
                    return dir;
            }
            return {0, 0, 0}; // a frame made of degenerate vectors, e.g. a field carrying no size
        }

        /* Find the closet direction */
        const GEO::index_t p0_idx = 3*points_nb_*(tracing_times_-1);
        const GEO::index_t p1_idx = 3*points_nb_*tracing_times_;
        const GEO::vec3 p0(points_[p0_idx+3*v], points_[p0_idx+3*v+1], points_[p0_idx+3*v+2]);
        const GEO::vec3 p1(points_[p1_idx+3*v], points_[p1_idx+3*v+1], points_[p1_idx+3*v+2]);
        const GEO::vec3 moving_dir = p1-p0;
        GEO::vec3 closet_dir;
        double closet_angle = M_PI;
        for (GEO::index_t n = 0; n < N_; ++n) {
            GEO::vec3 dir(mesh_element_direction_[fd_idx+3*n], mesh_element_direction_[fd_idx+3*n+1], mesh_element_direction_[fd_idx+3*n+2]);
            if (GEO::length(dir) < 1e-12) // degenerate direction, e.g. on a flat facet
                continue;

            /* The directions of a frame field are not oriented: a direction and its
             * opposite describe the same line, and neighbouring facets may well store
             * them with opposite signs. Compare the lines, then keep the direction that
             * goes along the motion. */
            if (GEO::dot(moving_dir, dir) < 0.0)
                dir = -dir;

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

    void StreamlineSurficialTracer::set_per_vertex_directions(
        const GEO::index_t N,
        const double* directions
        ) {
        throw std::logic_error("TODO"); // TODO
    }

    void StreamlineSurficialTracer::set_per_facet_directions(
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

    void StreamlineSurficialTracer::smooth(
        const GEO::index_t iterations,
        const double damping,
        const bool constraint_on_surface
        ) {
        assert(iterations > 0);
        if (tracing_times_ < 2)
            return;
        if (std::abs(damping-1) < 1e-10)
            return;

        std::vector<double> new_points = points_; // pre-allocated
        for (GEO::index_t iter = 0; iter < iterations; ++iter) {
            for (GEO::index_t v = 0; v < points_nb_; ++v) {
                for (GEO::index_t n = 1; n < tracing_times_; ++n) {
                    const GEO::vec3 prev_p(
                        points_[3*points_nb_*(n-1) + 3*v],
                        points_[3*points_nb_*(n-1) + 3*v+1],
                        points_[3*points_nb_*(n-1) + 3*v+2]);
                    const GEO::vec3 cur_p(
                        points_[3*points_nb_*n + 3*v],
                        points_[3*points_nb_*n + 3*v+1],
                        points_[3*points_nb_*n + 3*v+2]);
                    const GEO::vec3 next_p(
                        points_[3*points_nb_*(n+1) + 3*v],
                        points_[3*points_nb_*(n+1) + 3*v+1],
                        points_[3*points_nb_*(n+1) + 3*v+2]);

                    GEO::vec3 p = (1-damping) * 0.5*(prev_p + next_p) + damping * cur_p;

                    if (constraint_on_surface) {
                        if (const auto& [loc_f, loc_lv] = points_locate_element_[points_nb_*n + v];
                            loc_f != GEO::NO_FACET
                            ) {
                            assert(loc_f < mesh_.facets.nb());

                            if (loc_lv != GEO::NO_INDEX) {
                                assert(loc_lv < 3);
                                const auto& ep0 = mesh_.facets.point(loc_f, loc_lv);
                                const auto& ep1 = mesh_.facets.point(loc_f, (loc_lv+1)%3);
                                GEO::vec3 edge = ep1-ep0;
                                if (const double edge_length = GEO::length(edge);
                                    edge_length < 1e-10) // degenerate
                                    p = ep0;
                                else {
                                    edge /= edge_length;
                                    if (const double t = GEO::dot(p-ep0, edge);
                                        t < 0)
                                        p = ep0;
                                    else if (t > edge_length)
                                        p = ep1;
                                    else
                                        p = ep0 + t*edge;
                                }
                            }
                            else {
                                GEO::vec3 closest_p;
                                double lambda0, lambda1, lambda2;
                                GEO::Geom::point_triangle_squared_distance(
                                    p,
                                    mesh_.facets.point(loc_f, 0), mesh_.facets.point(loc_f, 1), mesh_.facets.point(loc_f, 2),
                                    closest_p,
                                    lambda0, lambda1, lambda2);
                                p = closest_p;
                            }
                        }
                    }

                    new_points[3*points_nb_*n + 3*v] = p[0];
                    new_points[3*points_nb_*n + 3*v+1] = p[1];
                    new_points[3*points_nb_*n + 3*v+2] = p[2];
                }
            }

            points_.swap(new_points);
        }

        initialize_points_locate_element();
    }

    void StreamlineSurficialTracer::initialize_points_locate_element(
        ) {
        points_locate_element_.assign(points_nb_, std::pair(GEO::NO_FACET, GEO::NO_INDEX));
        for (GEO::index_t v = 0; v < points_nb_; ++v) {
            const GEO::vec3 p(points_[3*v], points_[3*v+1], points_[3*v+2]);

            GEO::vec3 nearest_p;
            double sq_dist;
            points_locate_element_[v].first = mesh_AABB_.nearest_facet(p, nearest_p, sq_dist);

            points_[3*v] = nearest_p[0];
            points_[3*v+1] = nearest_p[1];
            points_[3*v+2] = nearest_p[2];
        }
    }

    void StreamlineSurficialTracer::trace_point(
        const GEO::index_t v,
        GEO::vec3& next_point,
        GEO::index_t& next_element,
        GEO::index_t& next_local_idx
        ) {
        assert(v < points_nb_);

        const GEO::index_t p_start_idx = 3*points_nb_*tracing_times_+3*v;
        const GEO::vec3 p0(points_[p_start_idx], points_[p_start_idx+1], points_[p_start_idx+2]);

        /* Init */
        next_point[0] = p0[0];
        next_point[1] = p0[1];
        next_point[2] = p0[2];
        next_element = GEO::NO_FACET;
        next_local_idx = GEO::NO_INDEX;

        const auto [loc_f, loc_lv] = points_locate_element_[points_nb_*tracing_times_ + v];
        if (loc_f == GEO::NO_FACET)
            return;
        assert(loc_f < mesh_.facets.nb());

        /* Compute tangential direction */
        GEO::vec3 d = get_point_direction(v); // before reset next
        d = d - GEO::dot(d, mesh_f_unit_normal_[loc_f])*mesh_f_unit_normal_[loc_f];
        const double d_norm = GEO::length(d);
        if (d_norm < 1e-10) // cannot find a valid direction!
            return;
        d /= d_norm;

        /* Find intersecting edge: the first one met ahead of p0. An intersection behind
         * p0 (t <= 0) is not an exit, and neither is the edge p0 already lies on, which
         * is where the previous step left it. */
        double max_t = -std::numeric_limits<double>::max();
        GEO::index_t max_lv = GEO::NO_INDEX;
        for (GEO::index_t lv = 0; lv < 3; ++lv) {
            if (lv == loc_lv)
                continue;

            const auto& ep0 = mesh_.facets.point(loc_f, lv);
            const auto& ep1 = mesh_.facets.point(loc_f, (lv+1)%3);
            const GEO::vec3 edge = ep1-ep0;

            const double s_d = GEO::dot(GEO::cross(edge, d), mesh_f_unit_normal_[loc_f]);
            if (std::abs(s_d) < 1e-10) // edge is parallel to d -> skip this edge
                continue;

            const double t = GEO::dot(GEO::cross(p0-ep0, edge), mesh_f_unit_normal_[loc_f]) / s_d;
            if (t < 0) // behind p0
                continue;

            /* The ray meets the line supporting this edge: reject the hit if it falls
             * outside the edge itself. */
            if (const double s = GEO::dot(GEO::cross(p0-ep0, d), mesh_f_unit_normal_[loc_f]) / s_d;
                s < -1e-10 || s > 1.0+1e-10)
                continue;

            if (t > max_t) {
                max_t = t;
                max_lv = lv;
            }
        }
        if (max_lv == GEO::NO_INDEX) // the direction leads out of this facet, no exit ahead
            return;

        /* Output */
        const GEO::vec3 p1 = p0+max_t*d;
        next_point[0] = p1[0];
        next_point[1] = p1[1];
        next_point[2] = p1[2];
        next_element = mesh_.facets.adjacent(loc_f, max_lv);
        if (next_element != GEO::NO_FACET)
            next_local_idx = mesh_.facets.find_vertex(next_element, mesh_.facets.vertex(loc_f, (max_lv+1)%3));
    }

    /* ============================================================================================================= */

    void StreamlineVolumetricTracer::set_per_vertex_directions(
        const GEO::index_t N,
        const double* directions
        ) {
        throw std::logic_error("TODO"); // TODO
    }

    void StreamlineVolumetricTracer::set_per_cell_directions(
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

    void StreamlineVolumetricTracer::smooth(
        const GEO::index_t iterations,
        const double damping
        ) {
        assert(iterations > 0);
        if (tracing_times_ < 2)
            return;
        if (std::abs(damping-1) < 1e-10)
            return;

        std::vector<double> new_points = points_; // pre-allocated
        for (GEO::index_t iter = 0; iter < iterations; ++iter) {
            for (GEO::index_t v = 0; v < points_nb_; ++v) {
                for (GEO::index_t n = 1; n < tracing_times_; ++n) {
                    const GEO::vec3 prev_p(
                        points_[3*points_nb_*(n-1) + 3*v],
                        points_[3*points_nb_*(n-1) + 3*v+1],
                        points_[3*points_nb_*(n-1) + 3*v+2]);
                    const GEO::vec3 cur_p(
                        points_[3*points_nb_*n + 3*v],
                        points_[3*points_nb_*n + 3*v+1],
                        points_[3*points_nb_*n + 3*v+2]);
                    const GEO::vec3 next_p(
                        points_[3*points_nb_*(n+1) + 3*v],
                        points_[3*points_nb_*(n+1) + 3*v+1],
                        points_[3*points_nb_*(n+1) + 3*v+2]);

                    const GEO::vec3 p = (1-damping) * 0.5*(prev_p + next_p) + damping * cur_p;

                    new_points[3*points_nb_*n + 3*v] = p[0];
                    new_points[3*points_nb_*n + 3*v+1] = p[1];
                    new_points[3*points_nb_*n + 3*v+2] = p[2];
                }
            }

            points_.swap(new_points);
        }

        initialize_points_locate_element();
    }

    void StreamlineVolumetricTracer::initialize_points_locate_element(
        ) {
        points_locate_element_.assign(points_nb_, std::pair(GEO::NO_CELL, GEO::NO_INDEX));
        for (GEO::index_t v = 0; v < points_nb_; ++v) {
            const GEO::vec3 p(points_[3*v], points_[3*v+1], points_[3*v+2]);

            points_locate_element_[v].first = mesh_AABB_.containing_tet(p);
        }
    }

    void StreamlineVolumetricTracer::trace_point(
        const GEO::index_t v,
        GEO::vec3& next_point,
        GEO::index_t& next_element,
        GEO::index_t& next_local_idx
        ) {
        assert(v < points_nb_);

        const GEO::index_t p_start_idx = 3*points_nb_*tracing_times_+3*v;
        const GEO::vec3 p0(points_[p_start_idx], points_[p_start_idx+1], points_[p_start_idx+2]);

        /* Init */
        next_point[0] = p0[0];
        next_point[1] = p0[1];
        next_point[2] = p0[2];
        next_element = GEO::NO_CELL;
        next_local_idx = GEO::NO_INDEX;

        const auto [loc_c, loc_lf] = points_locate_element_[v];
        if (loc_c == GEO::NO_CELL)
            return;
        assert(loc_c < mesh_.cells.nb());

        /* Compute direction */
        GEO::vec3 d = get_point_direction(v);
        const double d_norm = GEO::length(d);
        if (d_norm < 1e-10) // cannot find a valid direction!
            return;
        d /= d_norm;

        /* Find intersecting edge */
        double max_t = -std::numeric_limits<double>::max();
        GEO::index_t max_lf = GEO::NO_INDEX;
        for (GEO::index_t lf = 0; lf < 4; ++lf) {
            if (lf == loc_lf)
                continue;

            const auto& fp0 = mesh_.vertices.point(mesh_.cells.facet_vertex(loc_c, lf, 0));
            const auto& fp1 = mesh_.vertices.point(mesh_.cells.facet_vertex(loc_c, lf, 1));
            const auto& fp2 = mesh_.vertices.point(mesh_.cells.facet_vertex(loc_c, lf, 2));
            const auto E1 = fp1-fp0;
            const auto E2 = fp2-fp0;
            const auto T = p0-fp0;
            const auto P = GEO::cross(d, E2);
            const auto det = GEO::dot(E1, P);
            if (std::abs(det) < 1e-10) // d is parallel to this facet, skip
                continue;
            const auto Q = GEO::cross(T, E1);

            const auto u0 = GEO::dot(T, P) / det;
            const auto u1 = GEO::dot(d, Q) / det;
            if (u0 < 0 || u1 < 0 || u0+u1 > 1) // intersecting point is not located on this facet
                continue;

            const auto t = GEO::dot(E2, Q) / det;
            if (t < 0) // behind p0
                continue;
            
            if (t > max_t) {
                max_t = t;
                max_lf = lf;
            }
        }
        if (max_lf == GEO::NO_INDEX) // the direction leads out of this facet, no exit ahead
            return;

        /* Output */
        const GEO::vec3 p1 = p0+max_t*d;
        next_point[0] = p1[0];
        next_point[1] = p1[1];
        next_point[2] = p1[2];
        next_element = mesh_.cells.adjacent(loc_c, max_lf);
        if (next_element != GEO::NO_CELL)
            next_local_idx = mesh_.cells.find_tet_facet(
                next_element,
                mesh_.cells.facet_vertex(loc_c, max_lf, 2),
                mesh_.cells.facet_vertex(loc_c, max_lf, 1),
                mesh_.cells.facet_vertex(loc_c, max_lf, 0));
    }
}
