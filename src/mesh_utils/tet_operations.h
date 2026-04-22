//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/3/23.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOGRAM_MESH_UTILS_TETRAHEDRON_OPERATIONS_H
#define GEOGRAM_MESH_UTILS_TETRAHEDRON_OPERATIONS_H

#include <geogram/mesh/mesh.h>

namespace GEO::MeshUtils::Tet
{
    /**
     * Collects tetrahedra incident to a mesh vertex in traversal order.
     *
     * Starting from a cell-local-vertex seed, this function traces the
     * incident tetrahedra around the queried vertex and outputs an ordered
     * list of (cell, local-vertex) pairs.
     *
     * @param[in]     M              The tetrahedral mesh to query.
     * @param[in]     _c        Index of the seed cell.
     * @param[in]     _lv       Local vertex index (0-3) in @p start_c.
     * @param[in,out] c_and_lv       Output ordered (cell, local-vertex) list; existing contents are cleared.
     * @return true if the queried vertex is on the mesh border; false if it is an interior vertex.
     */
    bool get_vertex_incident_tetrahedra(
        const GEO::Mesh& M,
        GEO::index_t _c,
        GEO::index_t _lv,
        std::vector<std::pair<GEO::index_t, GEO::index_t>>& c_and_lv);

    /**
     * Collects tetrahedra incident to a mesh edge in traversal order.
     *
     * Starting from a cell-local-edge seed, this function traces the edge
     * ring (or border chain) and outputs an ordered list of (cell, local-facet)
     * pairs, where each local facet contains the queried edge.
     *
     * @param[in]     M                 The tetrahedral mesh to query
     * @param[in]     _c           Index of the seed cell
     * @param[in]     _le          Local edge index (0-5) in @p start_c
     * @param[in,out] ordered_c_and_lf  Output ordered (cell, local-facet) list; existing contents are cleared
     * @return true if the queried edge is on the mesh border; false if it is an interior edge
     */
    bool get_edge_incident_tetrahedra(
        const GEO::Mesh& M,
        GEO::index_t _c,
        GEO::index_t _le,
        std::vector<std::pair<GEO::index_t, GEO::index_t>>& ordered_c_and_lf);

    /**
     * Collects tetrahedra incident to a mesh edge in traversal order.
     *
     * Starting from a cell-facet-local-vertex seed, this function traces the edge
     * ring (or border chain) and outputs an ordered list of (cell, local-facet)
     * pairs, where each local facet contains the queried edge.
     *
     * @param[in]     M                 The tetrahedral mesh to query
     * @param[in]     _c           Index of the seed cell
     * @param[in]     _lf          Local facet index (0-3) in @p start_c
     * @param[in]     _lv          Local vertex index (0-2) in facet @p start_lf; with
     *                                  `(start_lv + 1) % 3` it defines the queried edge
     * @param[in,out] ordered_c_and_lf  Output ordered (cell, local-facet) list; existing contents are cleared
     * @return true if the queried edge is on the mesh border; false if it is an interior edge
     */
    bool get_edge_incident_tetrahedra(
        const GEO::Mesh& M,
        GEO::index_t _c,
        GEO::index_t _lf,
        GEO::index_t _lv,
        std::vector<std::pair<GEO::index_t, GEO::index_t>>& ordered_c_and_lf);

    /**
     * Splits one tetrahedron into four tetrahedra by inserting an interior vertex.
     *
     * The vertex index @p new_v is expected to be pre-allocated. Its position will be set
     * to the barycenter of cell @p c. The original cell @p c is updated in-place and
     * three additional tetrahedra (@p new_c0, @p new_c1, @p new_c2) are filled.
     * Adjacency between the four resulting cells and neighboring cells is updated.
     *
     * @param[in,out] M      The tetrahedral mesh to modify
     * @param[in]     c      Index of the tetrahedron to split
     * @param[in]     new_v  Index of a pre-allocated vertex used as the split vertex
     * @param[in]     new_c0 Index of the first pre-allocated tetrahedron created by the split
     * @param[in]     new_c1 Index of the second pre-allocated tetrahedron created by the split
     * @param[in]     new_c2 Index of the third pre-allocated tetrahedron created by the split
     */
    void cell_split(
        GEO::Mesh& M,
        GEO::index_t c,
        GEO::index_t new_v,
        GEO::index_t new_c0,
        GEO::index_t new_c1,
        GEO::index_t new_c2);

    /**
     * Splits a tetrahedral facet by inserting a new vertex on the facet.
     *
     * The vertex index @p new_v is expected to be pre-allocated. Its position will be set
     * to the barycenter of facet @p lf in cell @p c. The incident tetrahedral connectivity
     * is updated, and the resulting tetrahedra are written into the pre-allocated cells.
     * For a boundary facet, only @p new_c0 and @p new_c1 are used. For an interior facet,
     * @p new_c2 and @p new_c3 are also used.
     *
     * @param[in,out] M      The tetrahedral mesh to modify.
     * @param[in]     c      Index of the tetrahedron that owns the target facet.
     * @param[in]     lf     Local facet index (0-3) in cell @p c.
     * @param[in]     new_v  Index of a pre-allocated vertex used as the split vertex.
     * @param[in]     new_c0 Index of the first pre-allocated tetrahedron created by the split.
     * @param[in]     new_c1 Index of the second pre-allocated tetrahedron created by the split.
     * @param[in]     new_c2 Optional; index of the third pre-allocated tetrahedron used for interior facets.
     * @param[in]     new_c3 Optional; index of the fourth pre-allocated tetrahedron used for interior facets.
     */
    void cell_facet_split(
        GEO::Mesh& M,
        GEO::index_t c,
        GEO::index_t lf,
        GEO::index_t new_v,
        GEO::index_t new_c0,
        GEO::index_t new_c1,
        GEO::index_t new_c2 = GEO::NO_CELL,
        GEO::index_t new_c3 = GEO::NO_CELL);

    /**
     * Splits a tetrahedral edge by inserting one vertex and splitting all incident cells.
     *
     * The edge is identified by local edge index @p le in cell @p _c. The function
     * traverses all tetrahedra incident to that edge, places @p new_v on the edge,
     * and splits each incident tetrahedron into two tetrahedra while updating
     * local adjacencies.
     *
     * Vector @p new_cs provides pre-allocated cell indices for the new tetrahedra,
     * one per incident cell. If it contains fewer indices than required, additional
     * tetrahedra are created internally, which is slower.
     *
     * @param[in,out] M      The tetrahedral mesh to modify.
     * @param[in]     _c     Index of a seed cell containing the target edge.
     * @param[in]     le     Local edge index (0-5) in cell @p _c.
     * @param[in]     new_v  Index of a pre-allocated vertex used as the split vertex.
     * @param[in,out] new_cs Pre-allocated cell indices for split results; consumed entries are set to @c GEO::NO_CELL.
     * @param[in]     r      Interpolation ratio for placing @p new_v on the edge (`0` at the first endpoint, `1` at the second).
     */
    void cell_edge_split(
        GEO::Mesh& M,
        GEO::index_t _c,
        GEO::index_t le,
        GEO::index_t new_v,
        std::vector<GEO::index_t>& new_cs,
        double r = 0.5);

    /**
     * Collapses a tetrahedral edge by moving one endpoint along the edge and
     * updating the local cavity connectivity.
     *
     * The edge is identified by local edge index @p le in cell @p c.
     * Parameter @p r controls the new endpoint position by interpolation on the
     * edge segment (`0` keeps the first endpoint, `1` keeps the second endpoint).
     * Collapsed cells/vertices are reported through optional output arguments.
     *
     * @param[in,out] M         The tetrahedral mesh to modify.
     * @param[in]     c         Index of a cell containing the target edge.
     * @param[in]     le        Local edge index (0-5) in cell @p c.
     * @param[in]     r         Interpolation ratio for the kept vertex position on the edge.
     * @param[out]    disuse_v  Optional; receives the removed vertex index when non-null.
     * @param[out]    disuse_cs Optional; receives indices of cells removed by the collapse when non-null.
     */
    void cell_edge_collapse(
        GEO::Mesh& M,
        GEO::index_t c,
        GEO::index_t le,
        double r = 0.5,
        GEO::index_t* disuse_v = nullptr,
        std::vector<GEO::index_t>* disuse_cs = nullptr);

    /**
     * Performs a 2-3 edge swap operation on a tetrahedral mesh.
     *
     * This operation replaces 2 tetrahedra sharing a common facet with 3 tetrahedra
     * by flipping the shared facet. Given a cell @p c with a local facet @p lf,
     * this function identifies the adjacent cell across that facet and performs
     * the topological transformation.
     *
     * @param[in,out] M          The tetrahedral mesh to modify
     * @param[in]     c          Index of the first cell (tetrahedron) to swap
     * @param[in]     lf         Local facet index (0-3) of cell @p c defining the swap edge
     * @param[in,out] new_c      Index of the newly created cell for the 2-3 swap
     */
    void cell_edge_swap_2_3(
        GEO::Mesh& M,
        GEO::index_t c,
        GEO::index_t lf,
        GEO::index_t new_c);
}

#endif //GEOGRAM_MESH_UTILS_TETRAHEDRON_OPERATIONS_H