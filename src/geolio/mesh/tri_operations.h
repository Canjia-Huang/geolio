//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/3/13.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_TRIANGLE_OPERATIONS_H
#define GEOLIO_TRIANGLE_OPERATIONS_H

#include <geogram/mesh/mesh.h>

namespace geolio
{
    /**
     * @brief Split a triangle edge by inserting a vertex and update connectivity.
     * @details For triangle facet @p f and local vertex index @p lv, inserts a new vertex @p new_v
     *          on the oriented edge from local vertex lv to (lv+1)%3. The new vertex coordinates
     *          are set to the interpolated point on that edge (typically the midpoint). Facet @p f
     *          is replaced by two triangles that reference @p new_v. If the opposite facet exists,
     *          it is also split and both facets' adjacency is updated so manifold connectivity is
     *          preserved. Per-facet and per-corner attributes are copied/restored when
     *          @p update_attributes is true.
     *
     * Preconditions:
     *  - @p new_v, @p new_f0 and (if used) @p new_f1 must be unused (preallocated) indices.
     *  - The mesh must allow writing vertex coordinates and facet connectivity.
     *
     * Side effects:
     *  - Adds/sets coordinates for @p new_v and rewrites facet indices for @p f and the new facets.
     *  - Updates facet-to-facet adjacency for the affected triangles.
     *
     * @tparam DIM Coordinate dimension (2 or 3) used to read/write vertex coordinates.
     * @param[in,out] mesh Mesh to modify (topology and geometry are mutated).
     * @param[in] f Index of the triangle facet to split.
     * @param[in] lv Local vertex index in {0,1,2} specifying the edge (lv -> (lv+1)%3).
     * @param[in] new_v Index of a pre-allocated vertex; its coordinates will be written.
     * @param[in] new_f0 Index of a pre-allocated facet used to store one of the new triangles.
     * @param[in] new_f1 Index of a pre-allocated facet used to split the adjacent triangle; ignored if the edge is a boundary.
     * @param[in] update_attributes When true, per-facet/per-corner attributes are preserved on the new elements.
     */
    template <GEO::index_t DIM>
    void tri_edge_split(
        GEO::Mesh& mesh,
        GEO::index_t f,
        GEO::index_t lv,
        GEO::index_t new_v,
        GEO::index_t new_f0,
        GEO::index_t new_f1,
        bool update_attributes = true);

    extern template void tri_edge_split<2>(
        GEO::Mesh& mesh, GEO::index_t f, GEO::index_t lv, GEO::index_t new_v, GEO::index_t new_f0, GEO::index_t new_f1, bool update_attributes);
    extern template void tri_edge_split<3>(
        GEO::Mesh& mesh, GEO::index_t f, GEO::index_t lv, GEO::index_t new_v, GEO::index_t new_f0, GEO::index_t new_f1, bool update_attributes);

    /**
     * @brief Check whether collapsing a triangle edge keeps the mesh valid.
     * @details For facet @p f and local edge (lv -> lv+1), the function evaluates the collapse
     *          that moves vertex v(lv) to `(1-r)*p(lv) + r*p((lv+1)%3)` and merges v((lv+1)%3)
     *          into v(lv). It collects the one-rings of both endpoints via
     *          get_vertex_incident_facets(), rejects boundary configurations that would create a
     *          non-manifold vertex, checks for degenerate or duplicate triangles around the
     *          collapsed edge, and enforces the link condition: the two endpoints may only have in
     *          common the vertices opposite to the collapsed edge. Merging the two stars indeed
     *          turns the edge joining such a common neighbour to the surviving vertex into two
     *          copies of the same edge, which would leave the mesh non-manifold.
     * @param[in] mesh Target triangle mesh used only for geometric/topological queries.
     * @param[in] f Index of a triangle facet adjacent to the candidate edge.
     * @param[in] lv Local vertex index in {0,1,2} identifying the oriented edge (lv -> lv+1).
     * @return true if the local edge collapse preserves triangle orientations and manifoldness;
     *         false if any incident triangle would flip, become degenerate, or violate boundary constraints.
     */
    bool is_tri_edge_collapse_valid(
        const GEO::Mesh& mesh,
        GEO::index_t f,
        GEO::index_t lv);

    /**
     * @brief Collapse an edge of a triangle and update local connectivity.
     * @details Given facet @p f and local vertex index @p lv, this function collapses edge
     *          (lv -> lv+1) by moving vertex v(lv) to (1-r)*p(lv) + r*p((lv+1)%3), then merging
     *          v((lv+1)%3) into v(lv). It interpolates the vertex attributes, rewires every facet
     *          incident to the removed vertex to reference the surviving vertex, and relinks the
     *          facet-to-facet adjacency of the neighbouring facets across the collapsed cavity.
     *          Incident facets that used the collapsed edge become unused and are reported through
     *          output parameters; physical deletion is left to the caller.
     * @tparam DIM Coordinate dimension (2 or 3) used to read the interpolated vertex position.
     * @param[in,out] mesh The target mesh topology/geometry to update.
     * @param[in] f Index of a triangle facet incident to the edge to collapse.
     * @param[in] lv Local vertex index in {0,1,2} identifying the directed edge (lv -> lv+1).
     * @param[out] disuse_v Receives the index of the vertex that was merged away (the original v(lv+1)).
     * @param[out] disuse_f0 Receives the index of the first facet that becomes unused (typically @p f).
     * @param[out] disuse_f1 Receives the index of the opposite facet across the collapsed edge, or
     *                      GEO::NO_FACET if the edge was on the boundary.
     */
    template <GEO::index_t DIM>
    void tri_edge_collapse(
        GEO::Mesh& mesh,
        GEO::index_t f,
        GEO::index_t lv,
        GEO::index_t& disuse_v,
        GEO::index_t& disuse_f0,
        GEO::index_t& disuse_f1);

    extern template void tri_edge_collapse<2>(
        GEO::Mesh& mesh, GEO::index_t f, GEO::index_t lv, GEO::index_t& disuse_v, GEO::index_t& disuse_f0,
        GEO::index_t& disuse_f1);
    extern template void tri_edge_collapse<3>(
        GEO::Mesh& mesh, GEO::index_t f, GEO::index_t lv, GEO::index_t& disuse_v, GEO::index_t& disuse_f0,
        GEO::index_t& disuse_f1);

    /**
     * @brief Check whether swapping a triangle edge keeps the mesh consistent.
     * @details The function inspects the interior edge shared by facet @p f and its adjacent
     *          facet across local edge @p lv: together they form a quadrilateral whose diagonal is
     *          that edge, and the flip replaces it by the other diagonal (the edge joining the
     *          vertex opposite to @p lv in @p f to the vertex opposite to the shared edge in the
     *          adjacent facet). It first requires the edge to have an adjacent facet, then rejects
     *          the flip when that other diagonal is already an edge of the mesh, which would make
     *          the two flipped facets share it with the facets that already own it and break the
     *          2-manifoldness of the mesh. Every facet that may own that edge is incident to the
     *          vertex opposite to @p lv, so inspecting the one-ring of that vertex is complete.
     * @note This is a topological test. The caller is responsible for the geometric part of the
     *       validity, namely that the quadrilateral is convex, so that the flipped triangles stay
     *       inside the quad and keep their orientation: a non-convex quad must not be flipped (the
     *       Delaunay criterion of SwapOperation rejects exactly those flips).
     * @param[in] mesh Target triangle mesh used for topological queries.
     * @param[in] f Index of one incident facet of the interior edge to consider.
     * @param[in] lv Local edge index in {0,1,2} in facet @p f identifying the edge opposite local vertex @p lv.
     * @return true if the edge is interior and flipping it keeps the connectivity of the mesh
     *         manifold and consistent; false otherwise.
     */
    bool is_tri_edge_swap_valid(
        const GEO::Mesh& mesh,
        GEO::index_t f,
        GEO::index_t lv);

    /**
     * @brief Swap (flip) an interior edge shared by two triangles.
     * @details For triangle facet @p f and local vertex index @p lv, flips the interior edge
     *          opposite to local vertex lv: the two incident triangles that form a convex
     *          quadrilateral will have their shared diagonal replaced by the other diagonal.
     *          The operation keeps facet indices but rewrites their vertex connectivity and
     *          updates facet-to-facet adjacency around the quad. When @p update_attributes is
     *          true, per-facet and per-corner attributes are copied to preserve data consistency.
     *
     * Preconditions and validity checks:
     *  - The edge must be interior (have an adjacent facet). Boundary edges are not flipped.
     *  - The flip is rejected if it would create duplicate edges, non-manifold connectivity, or
     *    degenerate/inverted triangles. Use is_tri_edge_swap_valid() to test beforehand.
     *
     * @param[in,out] mesh Mesh to modify (topology and adjacency are mutated).
     * @param[in] f Index of one incident facet of the interior edge to flip.
     * @param[in] lv Local vertex index in {0,1,2} in facet @p f identifying the edge opposite local vertex @p lv.
     * @param[in] update_attributes If true, per-facet/per-corner attributes are preserved on the updated facets.
     * @return true if the flip was performed; false if the edge is a boundary or the operation was invalid.
     */
    bool tri_edge_swap(
        GEO::Mesh& mesh,
        GEO::index_t f,
        GEO::index_t lv,
        bool update_attributes = true);
}

#endif //GEOLIO_TRIANGLE_OPERATIONS_H
