//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/13.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "mesh_edges_AABB3d.h"
#include <geogram/basic/geometry_nd.h>
#include <geogram/mesh/mesh.h>

namespace
{
    /**
     * @brief Compute nearest point on a mesh edge to a query point.
     *
     * Given a reference to a Geogram mesh and an edge index, finds the closest
     * point on that edge to the query point `p`. Returns both the nearest point
     * on the segment and the squared distance between `p` and that nearest point.
     *
     * @param[in] mesh Reference to the Geogram mesh containing the edges.
     * @param[in] p Query point in 3D.
     * @param[in] e Index of the edge in the mesh to test.
     * @param[out] nearest_p Returned nearest point on edge `e` to `p`.
     * @param[out] squared_dist Returned squared Euclidean distance between `p`
     *             and `nearest_p`.
     */
    void get_point_edge_nearest_point(
        const GEO::Mesh& mesh,
        const GEO::vec3& p,
        const GEO::index_t e,
        GEO::vec3& nearest_p,
        double& squared_dist
        ) {
        squared_dist = GEO::Numeric::max_float64();

        double lambda0, lambda1;
        squared_dist = GEO::Geom::point_segment_squared_distance(
            p,
            mesh.vertices.point(mesh.edges.vertex(e, 0)),
            mesh.vertices.point(mesh.edges.vertex(e, 1)),
            nearest_p,
            lambda0, lambda1);
    }

    /**
     * @brief Squared distance from a point to the center of an axis-aligned box.
     *
     * Computes the squared Euclidean distance between the query point `p` and the
     * center of the axis-aligned box `B`. Used by the AABB traversal heuristic
     * to choose which child to descend first.
     *
     * @param[in] p Query point in 3D.
     * @param[in] B Axis-aligned bounding box.
     * @return Squared distance between `p` and the center of `B`.
     */
    double point_box_center_squared_distance(
        const GEO::vec3& p,
        const GEO::Box3d& B
        ) {
        double result = 0.0;
        for(GEO::coord_index_t c = 0; c < 3; ++c) {
            const double d = p[c] - 0.5 * (B.xyz_min[c] + B.xyz_max[c]);
            result += GEO::geo_sqr(d);
        }
        return result;
    }

    /**
     * @brief Squared distance from an interior point to the nearest box face.
     *
     * Assumes `p` lies inside `B`. Returns the squared distance from `p` to
     * the closest of the six axis-aligned faces of `B`.
     *
     * @pre B.contains(p) must be true.
     * @param[in] p Point guaranteed to be inside `B`.
     * @param[in] B Axis-aligned bounding box.
     * @return Squared distance from `p` to the nearest face of `B`.
     */
    double inner_point_box_squared_distance(
        const GEO::vec3& p,
        const GEO::Box3d& B
        ) {
        geo_debug_assert(B.contains(p));
        double result = GEO::geo_sqr(p[0] - B.xyz_min[0]);
        result = std::min(result, GEO::geo_sqr(p[0] - B.xyz_max[0]));
        for(GEO::coord_index_t c = 1; c < 3; ++c) {
            result = std::min(result, GEO::geo_sqr(p[c] - B.xyz_min[c]));
            result = std::min(result, GEO::geo_sqr(p[c] - B.xyz_max[c]));
        }
        return result;
    }

    /**
     * @brief Signed squared distance from a point to an axis-aligned box.
     *
     * If `p` is outside `B`, returns the squared distance from `p` to `B`.
     * If `p` is inside `B`, returns the negative squared distance from `p`
     * to the nearest face (i.e. -inner_point_box_squared_distance(p,B)).
     * The signed value is useful for pruning during AABB traversal.
     *
     * @param[in] p Query point in 3D.
     * @param[in] B Axis-aligned bounding box.
     * @return Positive squared distance for outside points, negative squared
     *         distance-to-nearest-face for inside points.
     */
    double point_box_signed_squared_distance(
        const GEO::vec3& p,
        const GEO::Box3d& B
        ) {
        bool inside = true;
        double result = 0.0;
        for(GEO::coord_index_t c = 0; c < 3; c++) {
            if(p[c] < B.xyz_min[c]) {
                inside = false;
                result += GEO::geo_sqr(p[c] - B.xyz_min[c]);
            } else if(p[c] > B.xyz_max[c]) {
                inside = false;
                result += GEO::geo_sqr(p[c] - B.xyz_max[c]);
            }
        }
        if(inside) {
            result = -inner_point_box_squared_distance(p, B);
        }
        return result;
    }
}

namespace geolio
{
    void MeshEdgesAABB3d::initialize(
        GEO::Mesh& mesh
        ) {
        mesh_ = &mesh;
        edges_nb_ = mesh_->edges.nb();

        AABB::initialize(
            edges_nb_,
            [&mesh](GEO::Box3d& B, const GEO::index_t e
                ) {
                // Get edge bbox
                const auto& p0 = mesh.vertices.point(mesh.edges.vertex(e, 0));
                const auto& p1 = mesh.vertices.point(mesh.edges.vertex(e, 1));
                for(GEO::coord_index_t c = 0; c < 3; c++) {
                    B.xyz_min[c] = std::min(p0[c], p1[c]);
                    B.xyz_max[c] = std::max(p0[c], p1[c]);

                    if (B.xyz_max[c]-B.xyz_min[c] < 1e-12) {
                        B.xyz_min[c] -= 1e-10;
                        B.xyz_max[c] += 1e-10;
                    }
                }
            });
    }

    void MeshEdgesAABB3d::get_nearest_edge_hint(
        const GEO::vec3& p,
        GEO::index_t& nearest_e, GEO::vec3& nearest_point, double& sq_dist
        ) const {
        // Find a good initial value for nearest_f by traversing
        // the boxes and selecting the child such that the center
        // of its bounding box is nearer to the query point.
        // For a large mesh (20M facets) this gains up to 10%
        // performance as compared to picking nearest_f randomly.
        GEO::index_t b = 0;
        GEO::index_t e = mesh_->edges.nb();
        GEO::index_t n = 1;
        while(e != b + 1) {
            const GEO::index_t m = b + (e - b) / 2;
            GEO::index_t childl = 2 * n;
            GEO::index_t childr = 2 * n + 1;
            if(
                point_box_center_squared_distance(p, bboxes_[childl]) <
                point_box_center_squared_distance(p, bboxes_[childr])
            ) {
                e = m;
                n = childl;
            } else {
                b = m;
                n = childr;
            }
        }
        nearest_e = element_in_leaf(b);

        nearest_point = mesh_->vertices.point(mesh_->edges.vertex(nearest_e, 0));
        sq_dist = GEO::Geom::distance2(p, nearest_point);
    }

    void MeshEdgesAABB3d::nearest_edge_recursive(
        const GEO::vec3& p,
        GEO::index_t& nearest_e, GEO::vec3& nearest_point, double& sq_dist,
        const GEO::index_t n, const GEO::index_t b, const GEO::index_t e
        ) const {
        geo_debug_assert(e > b);

        // If node is a leaf: compute point-edge distance
        // and replace current if nearer
        if(b + 1 == e) {
            const GEO::index_t edge = element_in_leaf(b);
            GEO::vec3 cur_nearest_point;
            double cur_sq_dist;
            get_point_edge_nearest_point(
                *mesh_, p, edge, cur_nearest_point, cur_sq_dist
            );
            if(cur_sq_dist < sq_dist) {
                nearest_e = edge;
                nearest_point = cur_nearest_point;
                sq_dist = cur_sq_dist;
            }
            return;
        }
        const GEO::index_t m = b + (e - b) / 2;
        GEO::index_t childl = 2 * n;
        GEO::index_t childr = 2 * n + 1;

        const double dl = point_box_signed_squared_distance(p, bboxes_[childl]);
        const double dr = point_box_signed_squared_distance(p, bboxes_[childr]);

        // Traverse the "nearest" child first, so that it has more chances
        // to prune the traversal of the other child.
        if(dl < dr) {
            if(dl < sq_dist) {
                nearest_edge_recursive(
                    p,
                    nearest_e, nearest_point, sq_dist,
                    childl, b, m
                );
            }
            if(dr < sq_dist) {
                nearest_edge_recursive(
                    p,
                    nearest_e, nearest_point, sq_dist,
                    childr, m, e
                );
            }
        } else {
            if(dr < sq_dist) {
                nearest_edge_recursive(
                    p,
                    nearest_e, nearest_point, sq_dist,
                    childr, m, e
                );
            }
            if(dl < sq_dist) {
                nearest_edge_recursive(
                    p,
                    nearest_e, nearest_point, sq_dist,
                    childl, b, m
                );
            }
        }
    }

    bool MeshEdgesAABB3d::nearest_edge_less_than_recursive(
        const GEO::vec3& p, const double less_than_sq_dist,
        GEO::index_t& nearest_e, GEO::vec3& nearest_point, double& sq_dist,
        const GEO::index_t n, const GEO::index_t b, const GEO::index_t e
        ) const {
        geo_debug_assert(e > b);

        if (sq_dist < less_than_sq_dist)
            return true;

        // If node is a leaf: compute point-edge distance
        // and replace current if nearer
        if(b + 1 == e) {
            GEO::index_t edge = element_in_leaf(b);
            GEO::vec3 cur_nearest_point;
            double cur_sq_dist;
            get_point_edge_nearest_point(
                *mesh_, p, edge, cur_nearest_point, cur_sq_dist
            );
            if (cur_sq_dist < less_than_sq_dist)
                return true;

            if(cur_sq_dist < sq_dist) {
                nearest_e = edge;
                nearest_point = cur_nearest_point;
                sq_dist = cur_sq_dist;
            }
        }
        else {
            GEO::index_t m = b + (e - b) / 2;
            GEO::index_t childl = 2 * n;
            GEO::index_t childr = 2 * n + 1;

            double dl = point_box_signed_squared_distance(p, bboxes_[childl]);
            double dr = point_box_signed_squared_distance(p, bboxes_[childr]);

            // Traverse the "nearest" child first, so that it has more chances
            // to prune the traversal of the other child.
            if(dl < dr) {
                if(dl < sq_dist) {
                    if (nearest_edge_less_than_recursive(
                        p, less_than_sq_dist,
                        nearest_e, nearest_point, sq_dist,
                        childl, b, m
                        ))
                        return true;
                }
                if(dr < sq_dist) {
                    if (nearest_edge_less_than_recursive(
                        p, less_than_sq_dist,
                        nearest_e, nearest_point, sq_dist,
                        childr, m, e
                        ))
                        return true;
                }
            } else {
                if(dr < sq_dist) {
                    if (nearest_edge_less_than_recursive(
                        p, less_than_sq_dist,
                        nearest_e, nearest_point, sq_dist,
                        childr, m, e
                        ))
                        return true;
                }
                if(dl < sq_dist) {
                    if (nearest_edge_less_than_recursive(
                        p, less_than_sq_dist,
                        nearest_e, nearest_point, sq_dist,
                        childl, b, m
                        ))
                        return true;
                }
            }
        }

        return false;
    }
}