//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/13.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_MESH_EDGES_AABB3D_H
#define GEOLIO_MESH_EDGES_AABB3D_H
#include <geogram/mesh/mesh_AABB.h>

namespace geolio
{
    class MeshEdgesAABB3d : public GEO::MeshAABB3d {
    public:
        MeshEdgesAABB3d() = default;

        explicit MeshEdgesAABB3d(GEO::Mesh& mesh){
            mesh_ = &mesh;
            initialize(mesh);
        }

        /**
         * @brief Initialize the AABB tree using edges from a mesh.
         *
         * Builds the internal AABB structure for fast spatial queries on all
         * edges in the provided Geogram mesh. The caller retains ownership of
         * the mesh; this class stores a non-owning pointer to it.
         *
         * @param[in] mesh Reference to the Geogram mesh whose edges will be indexed
         *            by the AABB tree.
         */
        void initialize(GEO::Mesh& mesh);

        /**
         * @brief Visit all edges whose bounding box intersects a query box.
         *
         * Executes `action(edge_index)` for every edge whose AABB intersects
         * the provided `box_in`.
         *
         * @param[in] box_in Query axis-aligned box.
         * @param[in] action Callback invoked with each intersecting edge index.
         */
        void compute_bbox_cell_bbox_intersections(const GEO::Box3d& box_in, const std::function<void(GEO::index_t)>& action) const {
            bbox_intersect_recursive(
                action, box_in, 1, 0, edges_nb_
            );
        }

        /**
         * @brief Visit all edges whose bounding boxes contain the query point.
         *
         * Calls `action(edge_index)` for each edge whose AABB contains the
         * point `p`.
         *
         * @param[in] p Query point in 3D.
         * @param[in] action Callback invoked with each containing edge index.
         */
        void containing_boxes(const GEO::vec3& p, const std::function<void(GEO::index_t)>& action) const {
            containing_bboxes_recursive(
                action, p, 1, 0, edges_nb_
            );
        }

        /**
         * @brief Compute all pairs of edges whose bounding boxes intersect.
         *
         * For each intersecting pair (i,j) the provided `action(i,j)` callback
         * is invoked. Self-pairs and duplicate pairs are handled by the
         * underlying traversal.
         *
         * @param[in] action Callback invoked for each intersecting edge pair
         *            with arguments (edge_index_i, edge_index_j).
         */
        void compute_edge_bbox_intersections(const std::function<void(GEO::index_t, GEO::index_t)>& action) const {
            self_intersect_recursive(
                action,
                1, 0, edges_nb_,
                1, 0, edges_nb_
            );
        }

        /**
         * @brief Compute intersecting edge bbox pairs in parallel.
         *
         * Same as `compute_edge_bbox_intersections` but attempts to parallelize
         * the recursive traversal to utilize multiple cores. The callback must
         * be thread-safe.
         *
         * @param[in] action Thread-safe callback invoked with (edge_i, edge_j).
         */
        void compute_edge_bbox_intersections_in_parallel(const std::function<void(GEO::index_t, GEO::index_t)>& action) const {
            self_intersect_recursive_in_parallel(
                action,
                1, 0, edges_nb_,
                1, 0, edges_nb_
            );
        }

        /**
         * @brief Compute intersecting bbox pairs between this tree and another.
         *
         * Traverses both AABB trees and invokes `action(i,j)` for each pair
         * of edges (i from this tree, j from `other`) whose bboxes intersect.
         *
         * @param[in] other Pointer to another MeshEdgesAABB3d instance.
         * @param[in] action Callback invoked with (edge_index_this, edge_index_other).
         */
        void compute_other_cell_bbox_intersections(const MeshEdgesAABB3d* other, const std::function<void(GEO::index_t, GEO::index_t)>& action) const {
            other_intersect_recursive(
                action,
                1, 0, edges_nb_,
                other,
                1, 0, other->edges_nb_
            );
        }

        /**
         * @brief Find the nearest edge to a point and return the closest point.
         *
         * Performs an AABB-accelerated nearest-segment search. On return, `nearest_point`
         * holds the closest point on the edge and `sq_dist` contains the squared distance.
         *
         * @param[in] p Query point in 3D.
         * @param[out] nearest_point Closest point on the nearest edge to `p`.
         * @param[out] sq_dist Squared distance between `p` and `nearest_point`.
         * @return Index of the nearest edge in the mesh.
         */
        GEO::index_t nearest_edge(const GEO::vec3& p, GEO::vec3& nearest_point, double& sq_dist) const {
            GEO::index_t nearest_edge;
            get_nearest_edge_hint(p, nearest_edge, nearest_point, sq_dist);
            nearest_edge_recursive(
                p,
                nearest_edge, nearest_point, sq_dist,
                1, 0, edges_nb_
            );
            return nearest_edge;
        }

        /**
         * @brief Convenience overload that returns the index of the nearest edge.
         *
         * Does not return the nearest point or squared distance. Use the
         * three-argument overload when these are required.
         *
         * @param[in] p Query point in 3D.
         * @return Index of the nearest edge.
         */
        [[nodiscard]] GEO::index_t nearest_edge(const GEO::vec3& p) const {
            GEO::vec3 nearest_point;
            double sq_dist;
            return nearest_edge(p, nearest_point, sq_dist);
        }

        /**
         * @brief Return the squared distance from `p` to the closest edge.
         *
         * Convenience wrapper that returns only the squared distance.
         *
         * @param[in] p Query point in 3D.
         * @return Squared distance to the nearest edge.
         */
        [[nodiscard]] double squared_distance(const GEO::vec3& p) const {
            GEO::vec3 nearest_point;
            double result;
            nearest_edge(p, nearest_point, result);
            return result;
        }

        /**
         * @brief Test whether the squared distance to the nearest edge is below a threshold.
         *
         * This function may exit early when it finds any edge closer than the provided
         * threshold; it is more efficient than computing the exact nearest edge.
         *
         * @param[in] p Query point in 3D.
         * @param[in] less_than_sq_dist Squared distance threshold to test against.
         * @return true if there exists an edge at squared distance < less_than_sq_dist.
         */
        [[nodiscard]] bool squared_distance_less_than(const GEO::vec3& p, const double less_than_sq_dist) const {
            GEO::index_t nearest_edge;
            GEO::vec3 nearest_point;
            double sq_dist;
            get_nearest_edge_hint(p, nearest_edge, nearest_point, sq_dist);
            return nearest_edge_less_than_recursive(
                p, less_than_sq_dist,
                nearest_edge, nearest_point, sq_dist,
                1, 0, edges_nb_);
        }

    protected:
        /**
         * @brief Recursive helper to visit all leaf edges whose bboxes contain `p`.
         *
         * Internal recursion used by `containing_boxes`. Traverses the AABB tree
         * and invokes `action` for each leaf element whose bbox contains `p`.
         *
         * @param[in] action Callback invoked with leaf edge indices.
         * @param[in] p Query point.
         * @param[in] node Current AABB node index.
         * @param[in] b Begin index of the element interval represented by `node`.
         * @param[in] e End index (exclusive) of the element interval represented by `node`.
         */
        void containing_bboxes_recursive(
            const std::function<void(GEO::index_t)>& action,
            const GEO::vec3& p,
            const GEO::index_t node, const GEO::index_t b, const GEO::index_t e
            ) const {
            geo_debug_assert(e != b);

            // Prune subtree that does not have intersection
            if(!bboxes_[node].contains(p)) {
                return;
            }

            // Leaf case
            if(e == b+1) {
                action(element_in_leaf(b));
                return;
            }

            // Recursion
            const GEO::index_t m = b + (e - b) / 2;
            const GEO::index_t node_l = 2 * node;
            const GEO::index_t node_r = 2 * node + 1;

            containing_bboxes_recursive(action, p, node_l, b, m);
            containing_bboxes_recursive(action, p, node_r, m, e);
        }

        /**
         * @brief Produce an initial hint (leaf) for the nearest-edge search.
         *
         * Traverses the AABB tree top-down selecting the child whose box center
         * is closer to `p`. The function returns a candidate leaf index and an
         * approximate squared distance to that leaf's representative point. The
         * result is used as an initial bound to speed-up the exact search.
         *
         * @param[in] p Query point.
         * @param[out] nearest_e Initial candidate edge index (in/out for search).
         * @param[out] nearest_point Point on the candidate edge used to initialize distance.
         * @param[out] sq_dist Squared distance from `p` to `nearest_point`.
         */
        void get_nearest_edge_hint(
            const GEO::vec3& p,
            GEO::index_t& nearest_e, GEO::vec3& nearest_point, double& sq_dist
            ) const;

        /**
         * @brief Parallel-capable recursive routine for self-intersection tests.
         *
         * Recursively traverses two intervals of the tree (possibly the same)
         * and invokes `action(i,j)` for intersecting leaf pairs. This version
         * uses parallel tasks to accelerate traversal when beneficial.
         *
         * @param[in] action Callback invoked with edge index pairs.
         * @param[in] node1,node2 Current node indices for the two traversals.
         * @param[in] b1,e1 Element interval for node1 and b2,e2 for node2.
         */
        void self_intersect_recursive_in_parallel(
            const std::function<void(GEO::index_t,GEO::index_t)>& action,
            const GEO::index_t node1, const GEO::index_t b1, const GEO::index_t e1,
            const GEO::index_t node2, const GEO::index_t b2, const GEO::index_t e2
            ) const {
            geo_debug_assert(e1 != b1);
            geo_debug_assert(e2 != b2);

            // Since we are intersecting the AABBTree with *itself*,
            // we can prune half of the cases by skipping the test
            // whenever node2's facet index interval is greated than
            // node1's facet index interval.
            if(e2 <= b1) {
                return;
            }

            // The acceleration is here:
            if(!bboxes_overlap(bboxes_[node1], bboxes_[node2])) {
                return;
            }

            // Simple case: leaf - leaf intersection.
            if(b1 + 1 == e1 && b2 + 1 == e2) {
                action(element_in_leaf(b1), element_in_leaf(b2));
                return;
            }

            // If node2 has more elements than node1, then
            //   intersect node2's two children with node1
            // else
            //   intersect node1's two children with node2
            if(e2 - b2 > e1 - b1) {
                const GEO::index_t m2 = b2 + (e2 - b2) / 2;
                const GEO::index_t node2_l = 2 * node2;
                const GEO::index_t node2_r = 2 * node2 + 1;

                auto func1 = [&]() {
                    self_intersect_recursive(action, node1, b1, e1, node2_l, b2, m2);
                };
                auto func2 = [&]() {
                    self_intersect_recursive(action, node1, b1, e1, node2_r, m2, e2);
                };
                GEO::parallel(func1, func2);
            } else {
                const GEO::index_t m1 = b1 + (e1 - b1) / 2;
                const GEO::index_t node1_l = 2 * node1;
                const GEO::index_t node1_r = 2 * node1 + 1;
                auto func1 = [&]() {
                    self_intersect_recursive(action, node1_l, b1, m1, node2, b2, e2);
                };
                auto func2 = [&]() {
                    self_intersect_recursive(action, node1_r, m1, e1, node2, b2, e2);
                };
                GEO::parallel(func1, func2);
            }
        }

        /**
         * @brief Exact recursive nearest-edge search on the AABB tree.
         *
         * Recursively explores the subtree rooted at `n` covering element
         * interval `[b,e)` and updates `nearest_e`, `nearest_point` and
         * `sq_dist` when a closer edge is found.
         *
         * @param[in] p Query point.
         * @param[in,out] nearest_e Current best edge index (updated when closer found).
         * @param[in,out] nearest_point Current best nearest point on the best edge.
         * @param[in,out] sq_dist Current best squared distance (updated in-place).
         * @param[in] n Current AABB node index.
         * @param[in] b Begin index of the element interval.
         * @param[in] e End index (exclusive) of the element interval.
         */
        void nearest_edge_recursive(
            const GEO::vec3& p,
            GEO::index_t& nearest_e, GEO::vec3& nearest_point, double& sq_dist,
            GEO::index_t n, GEO::index_t b, GEO::index_t e) const;

        /**
         * @brief Recursive predicate that checks for a closer edge than threshold.
         *
         * Traverses the subtree and returns early when any edge closer than
         * `less_than_sq_dist` is found. Also updates the nearest candidate if
         * a closer element is discovered during traversal.
         *
         * @param[in] p Query point.
         * @param[in] less_than_sq_dist Squared distance threshold for early exit.
         * @param[in,out] nearest_e Current best edge index (may be updated).
         * @param[in,out] nearest_point Current nearest point (may be updated).
         * @param[in,out] sq_dist Current best squared distance (may be updated).
         * @param[in] n Current AABB node index.
         * @param[in] b Begin index of the element interval.
         * @param[in] e End index (exclusive) of the element interval.
         * @return true if an edge closer than `less_than_sq_dist` was found.
         */
        bool nearest_edge_less_than_recursive(
            const GEO::vec3& p, double less_than_sq_dist,
            GEO::index_t& nearest_e, GEO::vec3& nearest_point, double& sq_dist,
            GEO::index_t n, GEO::index_t b, GEO::index_t e) const;

    private:
        GEO::index_t edges_nb_ = 0;
    };
}
#endif //GEOLIO_MESH_EDGES_AABB3D_H
