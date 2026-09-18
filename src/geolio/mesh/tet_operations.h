//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/3/23.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_TETRAHEDRON_OPERATIONS_H
#define GEOLIO_TETRAHEDRON_OPERATIONS_H

#include <geogram/mesh/mesh.h>
#include <cassert>
#include <ranges>
#include <stack>
#include <unordered_set>
#include <vector>
#include "tet_descriptor.h"

namespace geolio
{
    /**
     * @brief Find the local edge index in a tetrahedron from two local endpoint vertices.
     * @details Encodes the two local vertex indices as a bit mask `(1<<lv0)|(1<<lv1)` and
     *          compares it against the precomputed TET_ENCODED_LE table. Edge direction is
     *          ignored, and a mask that matches no edge returns GEO::NO_INDEX.
     * @param[in] lv0 Local vertex index (0-3) of one endpoint
     * @param[in] lv1 Local vertex index (0-3) of the other endpoint
     * @return Local edge index (0-5) if @p lv0 and @p lv1 form a tetrahedron edge; otherwise GEO::NO_INDEX
     */
    inline GEO::index_t find_tet_edge_from_local_vertices(
        const GEO::index_t lv0,
        const GEO::index_t lv1
        ) {
        assert(lv0 < 4);
        assert(lv1 < 4);

        switch ((1<<lv0) | (1<<lv1)) {
            case TET_ENCODED_LE[0]: return 0;
            case TET_ENCODED_LE[1]: return 1;
            case TET_ENCODED_LE[2]: return 2;
            case TET_ENCODED_LE[3]: return 3;
            case TET_ENCODED_LE[4]: return 4;
            case TET_ENCODED_LE[5]: return 5;
            default:
                return GEO::NO_INDEX;
        }
    }

    /**
     * @brief Find the local edge index in a tetrahedron from two endpoint vertices.
     * @details The search is performed in cell @p c using global vertex indices @p v0 and @p v1:
     *          it scans the cell vertices for @p v0, checks each of its adjacent local vertices
     *          for @p v1, and delegates to find_tet_edge_from_local_vertices(). Edge direction is
     *          ignored.
     * @param[in] mesh    The mesh to query
     * @param[in] c    Index of the tetrahedral cell to search
     * @param[in] v0  Global vertex index of one endpoint
     * @param[in] v1  Global vertex index of the other endpoint
     * @return Local edge index (0-5) in @p c if found; otherwise GEO::NO_INDEX
     */
    inline GEO::index_t find_tet_edge(
        const GEO::Mesh& mesh,
        const GEO::index_t c,
        const GEO::index_t v0,
        const GEO::index_t v1
        ) {
        assert(c < mesh.cells.nb());
        assert(mesh.cells.type(c) == GEO::MeshCellType::MESH_TET);

        for (GEO::index_t lv = 0; lv < 4; ++lv) {
            if (mesh.cells.vertex(c, lv) == v0) {
                for (const auto& adj_lv : TET_LV_ADJACENT_LV[lv]) {
                    if (mesh.cells.vertex(c, adj_lv) == v1)
                        return find_tet_edge_from_local_vertices(lv, adj_lv);
                }
                break;
            }
        }
        return GEO::NO_INDEX;
    }

    /**
     * @brief Return the third vertex of a tetrahedron facet from two known facet vertices.
     * @details XORs the three global vertex indices of facet (@p c, @p lf) together with @p v0
     *          and @p v1. Since the facet has three distinct vertices, the XOR cancels the two
     *          known vertices and leaves the third one. Preconditions (@p c, @p lf, and the
     *          membership of @p v0 and @p v1 in the facet) are debug-checked with assertions.
     * @param[in] mesh  Input tetrahedral mesh.
     * @param[in] c  Cell index.
     * @param[in] lf Local facet index (0..3) in cell @p c.
     * @param[in] v0 First known vertex on facet (@p c, @p lf).
     * @param[in] v1 Second known vertex on facet (@p c, @p lf); must be different from @p v0.
     * @return The facet vertex in (@p c, @p lf) that is different from @p v0 and @p v1.
     */
    inline GEO::index_t get_tet_facet_another_vertex(
        const GEO::Mesh& mesh,
        const GEO::index_t c,
        const GEO::index_t lf,
        const GEO::index_t v0,
        const GEO::index_t v1
        ) {
        assert(c < mesh.cells.nb());
        assert(mesh.cells.type(c) == GEO::MeshCellType::MESH_TET);
        assert(lf < 4);
        assert(mesh.cells.facet_vertex(c, lf, 0) == v0 || mesh.cells.facet_vertex(c, lf, 1) == v0 || mesh.cells.facet_vertex(c, lf, 2) == v0);
        assert(mesh.cells.facet_vertex(c, lf, 0) == v1 || mesh.cells.facet_vertex(c, lf, 1) == v1 || mesh.cells.facet_vertex(c, lf, 2) == v1);

        return mesh.cells.facet_vertex(c, lf, 0)^
               mesh.cells.facet_vertex(c, lf, 1)^
               mesh.cells.facet_vertex(c, lf, 2)^
               v0^
               v1;

        GEO::index_t v2 = GEO::NO_VERTEX;
        for (GEO::index_t lv = 0; lv < 3; ++lv) {
            v2 = mesh.cells.facet_vertex(c, lf, lv);
            if (v2 != v0 && v2 != v1)
                break;
        }
        return v2;
    }

    /**
     * @brief Split one tetrahedron into four tetrahedra by inserting an interior vertex.
     * @details The vertex index @p new_v is expected to be pre-allocated; its position is set
     *          to the barycenter of cell @p c. The original cell @p c is updated in place and
     *          three additional tetrahedra (@p new_c0, @p new_c1, @p new_c2) are filled. Each of
     *          the four resulting cells keeps three original vertices plus @p new_v, and the
     *          cell-to-cell adjacency is relinked, including the neighboring cells of @p c.
     * @param[in,out] mesh           The tetrahedral mesh to modify.
     * @param[in]     c              Index of the tetrahedron to split.
     * @param[in]     new_v          Index of a pre-allocated vertex used as the split vertex.
     * @param[in]     new_c0         Index of the first pre-allocated tetrahedron created by the split.
     * @param[in]     new_c1         Index of the second pre-allocated tetrahedron created by the split.
     * @param[in]     new_c2         Index of the third pre-allocated tetrahedron created by the split.
     * @param[in]     update_attributes If true, per-cell attributes are copied/restored on the new tetrahedra.
     */
    void tet_split(
        GEO::Mesh& mesh,
        GEO::index_t c,
        GEO::index_t new_v,
        GEO::index_t new_c0,
        GEO::index_t new_c1,
        GEO::index_t new_c2,
        bool update_attributes = true);

    /**
     * @brief Split a tetrahedral facet by inserting a new vertex on the facet.
     * @details The vertex index @p new_v is expected to be pre-allocated; its position is set
     *          to the barycenter of facet @p lf in cell @p c. The owning cell @p c is replaced by
     *          two tetrahedra written into @p new_c0 and @p new_c1. If the facet is interior, the
     *          adjacent tetrahedron is split symmetrically into @p new_c2 and @p new_c3, and all
     *          cell-to-cell adjacencies (including neighbors of the original two cells) are
     *          relinked. For a boundary facet only @p new_c0 and @p new_c1 are used.
     * @param[in,out] mesh           The tetrahedral mesh to modify.
     * @param[in]     c              Index of the tetrahedron that owns the target facet.
     * @param[in]     lf             Local facet index (0-3) in cell @p c.
     * @param[in]     new_v          Index of a pre-allocated vertex used as the split vertex.
     * @param[in]     new_c0         Index of the first pre-allocated tetrahedron created by the split.
     * @param[in]     new_c1         Index of the second pre-allocated tetrahedron created by the split.
     * @param[in]     new_c2         Index of the third pre-allocated tetrahedron used for interior facets; unused (and typically GEO::NO_CELL) for boundary facets.
     * @param[in]     new_c3         Index of the fourth pre-allocated tetrahedron used for interior facets; unused (and typically GEO::NO_CELL) for boundary facets.
     * @param[in]     update_attributes If true, per-cell attributes are preserved on all new tetrahedra.
     */
    void tet_facet_split(
        GEO::Mesh& mesh,
        GEO::index_t c,
        GEO::index_t lf,
        GEO::index_t new_v,
        GEO::index_t new_c0,
        GEO::index_t new_c1,
        GEO::index_t new_c2,
        GEO::index_t new_c3,
        bool update_attributes = true);

    /**
     * @brief Split a tetrahedral edge by inserting one vertex and splitting all incident cells.
     * @details The incident cells are provided in order through @p ordered_c_le_lf. Each element
     *          is a tuple (cell, local_edge, local_facet) describing a tetrahedron incident to
     *          the target edge; the vector must be in ring/chain order around the edge
     *          (as produced by get_edge_incident_cells()). The function places @p new_v on the
     *          edge at interpolation factor @p r (0 at the first endpoint, 1 at the second), then
     *          splits every incident tetrahedron into two and relinks the surrounding cell-to-cell
     *          adjacency. The @p new_cs array must contain one pre-allocated cell index per
     *          incident tetrahedron; entries are consumed in the same order and reset to
     *          GEO::NO_CELL on return.
     * @param[in,out] mesh                The tetrahedral mesh to modify.
     * @param[in]     ordered_c_le_lf     Ordered list of (cell, local_edge, local_facet) tuples for all tetrahedra incident to the edge, in ring/chain order.
     * @param[in]     new_v               Index of the pre-allocated vertex to place on the edge.
     * @param[in]     new_cs              Array of pre-allocated tetrahedron indices (one per incident cell).
     * @param[in]     update_attributes   If true, per-cell attributes are copied/restored on the new tetrahedra.
     */
    void tet_edge_split(
        GEO::Mesh& mesh,
        const std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>>& ordered_c_le_lf,
        GEO::index_t new_v,
        const std::vector<GEO::index_t>& new_cs,
        bool update_attributes = true);

    /**
     * @brief Check whether a tetrahedral edge collapse is topologically valid.
     * @details The operation removes one endpoint of the edge by merging it into the other,
     *          and then rewrites all incident cells. This validation rejects cases that would
     *          create a non-manifold vertex, degenerate facets, or duplicate tetrahedra in the
     *          surrounding cavity. It also enforces the link condition: the two endpoints may only
     *          have in common the vertices opposite the collapsed edge in the cells that contain
     *          it. Merging the two stars indeed turns the edge joining such a common neighbour to
     *          the surviving vertex into two copies of the same edge, which would leave the mesh
     *          non-manifold.
     * @param[in] mesh Input tetrahedral mesh.
     * @param[in] c    Index of a tetrahedron containing the target edge.
     * @param[in] le   Local edge index (0-5) in cell @p c.
     * @return true if collapsing edge (@p c, @p le) is valid; false otherwise.
     */
    bool is_tet_edge_collapse_valid(
        const GEO::Mesh& mesh,
        GEO::index_t c,
        GEO::index_t le);

    /**
     * @brief Collapse a tetrahedral edge by moving one endpoint along the edge and updating the local cavity connectivity.
     * @details The edge is identified by local edge index @p _le in cell @p _c. The function
     *          interpolates the surviving endpoint along the edge segment, rewrites the removed
     *          endpoint to the surviving vertex in all remaining cells, relinks the adjacency of
     *          the surviving cavity cells, and reports the removed vertex and cells through output
     *          arguments.
     * @note The collapse is assumed valid: test the edge with is_tet_edge_collapse_valid()
     *       beforehand, otherwise the resulting mesh may not be a valid tetrahedralization.
     * @param[in,out] mesh         The tetrahedral mesh to modify.
     * @param[in]     _c           Index of a cell containing the target edge.
     * @param[in]     _le          Local edge index (0-5) in cell @p _c.
     * @param[out]    disuse_v     Receives the removed vertex index.
     * @param[out]    disuse_cs    Receives indices of cells removed by the collapse.
     */
    void tet_edge_collapse(
        GEO::Mesh& mesh,
        GEO::index_t _c,
        GEO::index_t _le,
        GEO::index_t& disuse_v,
        std::vector<GEO::index_t>& disuse_cs);

    /**
     * @brief Check whether a 2-3 facet swap is geometrically and topologically valid.
     * @details This validates the local operation that replaces two tetrahedra sharing a
     *          facet by three tetrahedra. The check ensures the shared facet is interior,
     *          and that the resulting cavity does not introduce duplicate vertices on the
     *          incident neighborhoods of the two swapped tetrahedra.
     * @param[in] mesh Input tetrahedral mesh.
     * @param[in] c    Index of the seed tetrahedron containing the target facet.
     * @param[in] lf   Local facet index (0-3) in cell @p c.
     * @return true if the 2-3 swap on facet (@p c, @p lf) is valid; false otherwise.
     */
    bool is_tet_edge_swap_2_3_valid(
        const GEO::Mesh& mesh,
        GEO::index_t c,
        GEO::index_t lf);

    /**
     * @brief Perform a 2-3 facet swap operation on a tetrahedral mesh.
     * @details This operation replaces two tetrahedra that share a common facet with three
     *          tetrahedra by flipping that facet. Given a seed cell @p c and its local facet @p lf,
     *          the function identifies the adjacent tetrahedron across the facet, rewrites the
     *          vertices of all three resulting cells (the original two plus the pre-allocated
     *          @p new_c), and relinks every cell-to-cell adjacency around the transformed cavity.
     * @param[in,out] mesh            The tetrahedral mesh to modify.
     * @param[in]     c               Index of the seed cell containing the target facet.
     * @param[in]     lf              Local facet index (0-3) of cell @p c.
     * @param[in]     new_c           Index of the pre-allocated cell used to store the newly created tetrahedron.
     * @param[in]     update_attributes If true, per-cell attributes are preserved on the modified cells.
     */
    void tet_edge_swap_2_3(
        GEO::Mesh& mesh,
        GEO::index_t c,
        GEO::index_t lf,
        GEO::index_t new_c,
        bool update_attributes = true);

    /**
     * @brief Perform a 3-2 edge swap operation on a tetrahedral mesh.
     * @details This operation replaces the three tetrahedra around a common interior edge with
     *          two tetrahedra by reconnecting the ring of cells that share the edge. The input
     *          @p ordered_c_le_lf stores the three incident cells in circular order as
     *          `(cell index, local edge index, local facet index)`, which identifies the edge
     *          and the facet on each cell used to walk the cavity. The function rewrites the
     *          vertices of the two surviving cells, relinks adjacent cells around the modified
     *          cavity, and reports the removed cell index through @p disuse_c.
     * @param[in,out] mesh               The tetrahedral mesh to modify.
     *          The ring is validated before anything is modified: it must hold three distinct cells
     *          that share the same interior edge, and the two resulting cells must not already exist
     *          in the mesh (that is, no cell may already hold the three vertices of their common
     *          facet).
     * @param[in]     ordered_c_le_lf    Three ordered tuples describing the edge-connected cell ring.
     * @param[out]    disuse_c           Reference to receive the index of the cell removed by the 3-2 swap.
     * @param[in]     update_attributes  If true, per-cell attributes are copied/restored around the modified cavity.
     * @return true if the swap is performed successfully; false if the input ring is invalid or the operation cannot be applied.
     */
    bool tet_edge_swap_3_2(
        GEO::Mesh& mesh,
        const std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>>& ordered_c_le_lf,
        GEO::index_t& disuse_c,
        bool update_attributes = true);
}

#endif //GEOLIO_TETRAHEDRON_OPERATIONS_H
