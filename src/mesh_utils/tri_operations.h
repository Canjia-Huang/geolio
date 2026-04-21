//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/3/13.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOGRAM_MESH_UTILS_TRIANGLE_OPERATIONS_H
#define GEOGRAM_MESH_UTILS_TRIANGLE_OPERATIONS_H

#include <geogram/mesh/mesh.h>

namespace GEO::MeshUtils
{
    /**
     * @brief Collect triangles incident to a vertex in one-ring order.
     *
     * Starting from facet @p start_f and its local vertex slot @p start_lv, this function traverses
     * all incident triangles around that vertex and outputs ordered pairs (facet index, local vertex index).
     * For interior vertices, the sequence forms a closed ring. For border vertices, the sequence is ordered
     * from one border side to the other.
     *
     * @param[in] M Input triangle mesh.
     * @param[in] start_f Seed facet index incident to the target vertex.
     * @param[in] start_lv Local vertex index (0, 1, or 2) of the target vertex in @p start_f.
     * @param[out] ordered_f_and_lv Output ordered one-ring list. Each element is (f, lv), where
     *                              @p f is an incident facet and @p lv is the local index of the target
     *                              vertex inside that facet. Existing contents are cleared.
     * @return true if the target vertex is on the mesh border; false if it is an interior vertex.
     */
    bool get_vertex_incident_triangles(
        const GEO::Mesh& M,
        GEO::index_t start_f,
        GEO::index_t start_lv,
        std::vector<std::pair<GEO::index_t, GEO::index_t>>& ordered_f_and_lv);

    /**
     * @brief Split an edge of a triangle in a mesh and update the adjacency topology accordingly.
     *
     * Given triangle facet @p f and local vertex index @p lv, a new vertex @p new_v is inserted
     * on the edge (lv → lv+1) at interpolation ratio @p r, splitting @p f into two triangles
     * (@p f and @p new_f1). If the adjacent facet across that edge exists (af != NO_FACET),
     * it is simultaneously split into two triangles (@p af and @p new_f2) to keep the mesh
     * topology consistent.
     *
     * @param[in, out] M The target mesh. Vertex and facet storage must be pre-allocated.
     * @param[in] f Index of the triangle facet to split.
     * @param[in] lv Local vertex index (0, 1, or 2) that identifies the edge to split (lv → lv+1).
     * @param[in] r Interpolation ratio in [0, 1] controlling where the new vertex is placed along the edge.
     * @param[in] new_v Index of the pre-allocated new vertex. Its position will be set to (1-r)*p(lv) + r*p(lv+1).
     * @param[in] new_f0 Index of the pre-allocated new facet produced by splitting @p f.
     * @param[in] new_f1 Index of the pre-allocated new facet produced by splitting the adjacent facet @p af.
     * Ignored when @p af does not exist (NO_FACET).
     */
     void edge_split(
        GEO::Mesh& M,
        GEO::index_t f,
        GEO::index_t lv,
        double r,
        GEO::index_t new_v,
        GEO::index_t new_f0,
        GEO::index_t new_f1);

    /**
     * @brief Collapse an edge of a triangle and update local connectivity.
     *
     * Given facet @p f and local vertex index @p lv, this function collapses edge (lv -> lv+1)
     * by moving vertex v(lv) to (1-r)*p(lv) + r*p(lv+1), then merging v(lv+1) into v(lv).
     * The two incident triangles on that edge become unused: @p f and, if it exists, the adjacent
     * facet across local edge @p lv.
     *
     * @param[in, out] M The target mesh topology/geometry to update.
     * @param[in] f Index of the triangle facet that owns the collapsed edge.
     * @param[in] lv Local vertex index (0, 1, or 2) identifying the edge to collapse (lv -> lv+1).
     * @param[in] r Interpolation ratio in [0, 1] for the new position of v(lv).
     * @param[out] disuse_v Index of the merged-away vertex (original v(lv+1)).
     * @param[out] disuse_f0 Index of the first unused facet after collapse (always @p f).
     * @param[out] disuse_f1 Index of the second unused facet across the collapsed edge;
     *                       set to GEO::NO_FACET when the edge is on the border.
     */
    void edge_collapse(
        GEO::Mesh& M,
        GEO::index_t f,
        GEO::index_t lv,
        double r,
        GEO::index_t& disuse_v,
        GEO::index_t& disuse_f0,
        GEO::index_t& disuse_f1);

    /**
     * @brief Swap an interior edge shared by two triangles.
     *
     * For facet @p f and local edge @p lv (between local vertices @c lv1 and @c lv2), this
     * operation replaces the shared diagonal with the other diagonal of the local quadrilateral.
     * The two incident facets keep their indices, while their vertex connectivity and adjacency
     * links are updated in-place.
     *
     * @param[in, out] M Target triangle mesh whose facet connectivity/adjacency is modified.
     * @param[in] f Index of one incident facet of the edge to flip.
     * @param[in] lv Local edge index (0, 1, or 2) in facet @p f identifying the edge opposite
     *               local vertex @p lv. The edge must have a valid adjacent facet
     *               (@c M.facets.adjacent(f, lv) != GEO::NO_FACET).
     * @pre the edge to flip is a border edge
     */
    void edge_swap(
        GEO::Mesh& M,
        GEO::index_t f,
        GEO::index_t lv);
}

#endif //GEOGRAM_MESH_UTILS_TRIANGLE_OPERATIONS_H

