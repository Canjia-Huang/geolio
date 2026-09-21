//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/3/31.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_HEXAHEDRON_OPERATIONS_H
#define GEOLIO_HEXAHEDRON_OPERATIONS_H

#include <geogram/mesh/mesh.h>
#include <cassert>
#include "hex_descriptor.h"

namespace geolio
{
    /**
     * @brief Find the local vertex index in a hexahedron from a global vertex index.
     * @details Linearly scans the 8 vertices of cell @p c and returns the first local slot
     *          whose global vertex matches @p v; returns GEO::NO_INDEX if the vertex is not
     *          present.
     * @param[in] mesh  The hexahedral mesh to query
     * @param[in] c  Index of the hexahedral cell to search
     * @param[in] v  Global vertex index to locate in cell @p c
     * @return Local vertex index (0-7) in @p c if found; otherwise GEO::NO_INDEX
     */
    inline GEO::index_t find_hex_vertex(
        const GEO::Mesh& mesh,
        const GEO::index_t c,
        const GEO::index_t v
        ) {
        assert(c < mesh.cells.nb());
        assert(mesh.cells.type(c) == GEO::MeshCellType::MESH_HEX);

        for (GEO::index_t lv = 0; lv < 8; ++lv) {
            if (mesh.cells.vertex(c, lv) == v)
                return lv;
        }
        return GEO::NO_INDEX;
    }

    /**
     * @brief Find the local edge index in a hexahedron from two local endpoint vertices.
     * @details Encodes the two local vertex indices as a bit mask `(1<<lv0)|(1<<lv1)` and
     *          compares it against the precomputed HEX_ENCODED_LE table. Edge direction is
     *          ignored, and a mask that matches no edge returns GEO::NO_INDEX.
     * @param[in] lv0 Local vertex index (0-7) of one endpoint
     * @param[in] lv1 Local vertex index (0-7) of the other endpoint
     * @return Local edge index (0-11) if @p lv0 and @p lv1 form a hexahedron edge; otherwise GEO::NO_INDEX
     */
    inline GEO::index_t find_hex_edge_from_local_vertices(
        const GEO::index_t lv0,
        const GEO::index_t lv1
        ) {
        assert(lv0 < 8);
        assert(lv1 < 8);

        switch ((1<<lv0) | (1<<lv1)) {
            case HEX_ENCODED_LE[0]: return 0;
            case HEX_ENCODED_LE[1]: return 1;
            case HEX_ENCODED_LE[2]: return 2;
            case HEX_ENCODED_LE[3]: return 3;
            case HEX_ENCODED_LE[4]: return 4;
            case HEX_ENCODED_LE[5]: return 5;
            case HEX_ENCODED_LE[6]: return 6;
            case HEX_ENCODED_LE[7]: return 7;
            case HEX_ENCODED_LE[8]: return 8;
            case HEX_ENCODED_LE[9]: return 9;
            case HEX_ENCODED_LE[10]: return 10;
            case HEX_ENCODED_LE[11]: return 11;
            default:
                return GEO::NO_INDEX;
        }
    }

    /**
     * @brief Find the local edge index in a hexahedron from two endpoint vertices.
     * @details The search is performed in cell @p c using global vertex indices @p v0 and @p v1:
     *          it scans the cell vertices for @p v0, checks each of its adjacent local vertices
     *          for @p v1, and delegates to find_hex_edge_from_local_vertices(). Edge direction is
     *          ignored.
     * @param[in] mesh    The hexahedral mesh to query
     * @param[in] c    Index of the hexahedral cell to search
     * @param[in] v0  Global vertex index of one endpoint
     * @param[in] v1  Global vertex index of the other endpoint
     * @return Local edge index (0-11) in @p c if found; otherwise GEO::NO_INDEX
     */
    inline GEO::index_t find_hex_edge(
        const GEO::Mesh& mesh,
        const GEO::index_t c,
        const GEO::index_t v0,
        const GEO::index_t v1
        ) {
        assert(c < mesh.cells.nb());
        assert(mesh.cells.type(c) == GEO::MeshCellType::MESH_HEX);

        for (GEO::index_t lv = 0; lv < 8; ++lv) {
            if (mesh.cells.vertex(c, lv) == v0) {
                for (const auto& adj_lv : HEX_LV_ADJACENT_LV[lv]) {
                    if (mesh.cells.vertex(c, adj_lv) == v1)
                        return find_hex_edge_from_local_vertices(lv, adj_lv);
                }
                break;
            }
        }
        return GEO::NO_INDEX;
    }

    /**
     * @brief Find a local facet index from three local vertices of a hexahedron.
     * @details The three local vertices can be provided in any order. The function returns
     *          GEO::NO_INDEX if any two vertices coincide, then encodes the vertices as a bit
     *          mask and checks whether it is a subset of one of the precomputed HEX_ENCODED_LF
     *          facet masks.
     * @param[in] lv0 Local vertex index (0-7)
     * @param[in] lv1 Local vertex index (0-7)
     * @param[in] lv2 Local vertex index (0-7)
     * @return Local facet index (0-5) if the three vertices are on the same facet; otherwise GEO::NO_INDEX
     */
    inline GEO::index_t find_hex_facet_from_local_vertices(
        const GEO::index_t lv0,
        const GEO::index_t lv1,
        const GEO::index_t lv2
        ) {
        assert(lv0 < 8);
        assert(lv1 < 8);
        assert(lv2 < 8);

        if (lv0 == lv1 || lv1 == lv2 || lv2 == lv0)
            return GEO::NO_INDEX;
        if (const auto& encoded_lf = (1<<lv0) | (1<<lv1) | (1<<lv2);
            (encoded_lf & HEX_ENCODED_LF[0]) == encoded_lf)
            return 0;
        else if ((encoded_lf & HEX_ENCODED_LF[1]) == encoded_lf)
            return 1;
        else if ((encoded_lf & HEX_ENCODED_LF[2]) == encoded_lf)
            return 2;
        else if ((encoded_lf & HEX_ENCODED_LF[3]) == encoded_lf)
            return 3;
        else if ((encoded_lf & HEX_ENCODED_LF[4]) == encoded_lf)
            return 4;
        else if ((encoded_lf & HEX_ENCODED_LF[5]) == encoded_lf)
            return 5;
        return GEO::NO_INDEX;
    }

    /**
     * @brief Find a local facet index from four local vertices of a hexahedron.
     * @details The four local vertices can be provided in any order. The function encodes them
     *          as a bit mask `(1<<lv0)|(1<<lv1)|(1<<lv2)|(1<<lv3)` and looks it up directly in
     *          the precomputed HEX_ENCODED_LF table; a mask that matches no facet (for example
     *          a set of vertices not coplanar on a single facet) returns GEO::NO_INDEX.
     * @param[in] lv0 Local vertex index (0-7)
     * @param[in] lv1 Local vertex index (0-7)
     * @param[in] lv2 Local vertex index (0-7)
     * @param[in] lv3 Local vertex index (0-7)
     * @return Local facet index (0-5) if the four vertices exactly define one facet; otherwise GEO::NO_INDEX
     */
    inline GEO::index_t find_hex_facet_from_local_vertices(
        const GEO::index_t lv0,
        const GEO::index_t lv1,
        const GEO::index_t lv2,
        const GEO::index_t lv3
        ) {
        assert(lv0 < 8);
        assert(lv1 < 8);
        assert(lv2 < 8);
        assert(lv3 < 8);

        switch ((1<<lv0) | (1<<lv1) | (1<<lv2) | (1<<lv3)) {
            case HEX_ENCODED_LF[0]: return 0;
            case HEX_ENCODED_LF[1]: return 1;
            case HEX_ENCODED_LF[2]: return 2;
            case HEX_ENCODED_LF[3]: return 3;
            case HEX_ENCODED_LF[4]: return 4;
            case HEX_ENCODED_LF[5]: return 5;
            default:
                return GEO::NO_INDEX;
        }
    }

    /**
     * @brief Find the local facet index in a hexahedron from three global vertices.
     * @details The three vertices can be provided in any order. The function maps each global
     *          vertex to its local slot in cell @p c via find_hex_vertex() and then delegates to
     *          find_hex_facet_from_local_vertices(). If any vertex is absent from @p c, it returns
     *          GEO::NO_INDEX.
     * @param[in] mesh   The hexahedral mesh to query
     * @param[in] c   Index of the hexahedral cell to search
     * @param[in] v0  Global vertex index on the target facet
     * @param[in] v1  Global vertex index on the target facet
     * @param[in] v2  Global vertex index on the target facet
     * @return Local facet index (0-5) in @p c if found; otherwise GEO::NO_INDEX
     */
    inline GEO::index_t find_hex_facet(
        const GEO::Mesh& mesh,
        const GEO::index_t c,
        const GEO::index_t v0,
        const GEO::index_t v1,
        const GEO::index_t v2
        ) {
        assert(c < mesh.cells.nb());
        assert(mesh.cells.type(c) == GEO::MeshCellType::MESH_HEX);
        assert(v0 < mesh.vertices.nb());
        assert(v1 < mesh.vertices.nb());
        assert(v2 < mesh.vertices.nb());

        const auto lv0 = find_hex_vertex(mesh, c, v0);
        const auto lv1 = find_hex_vertex(mesh, c, v1);
        const auto lv2 = find_hex_vertex(mesh, c, v2);
        if (lv0 == GEO::NO_INDEX || lv1 == GEO::NO_INDEX || lv2 == GEO::NO_INDEX)
            return GEO::NO_INDEX;
        return find_hex_facet_from_local_vertices(lv0, lv1, lv2);
    }

            /**
         * @brief Trace a hexahedral-stacked sheet (loop) starting from a given cell and edge.
         *
         * This function traces through the stacked hex-cell structure to identify all cells
         * that form a continuous topological loop (or sheet) starting from the specified
         * starting cell and local edge. The traced loop is recorded as a sequence of
         * (cell_index, cut_type) pairs that encode both the cell identity and the specific
         * pair of cutting edges within that cell.
         *
         * The tracing follows the adjacency structure of the hexahedral mesh, moving from
         * one cell to its neighbors while maintaining the continuity of the topological loop.
         *
         * @param[in] mesh The hexahedral mesh to query
         * @param[in] start_c Index of the starting hexahedral cell (0..hex_mesh_.cells.nb()-1).
         * @param[in] start_le Local edge index within the starting cell defining the initial
         *            direction of the loop. Valid range: 0-11.
         * @param[out] sheet_hexes Output vector containing pairs (cell_index, cell_cut_type)
         *             for all cells along the traced loop, in order of traversal.
         *             The vector is cleared before population.
         *             Each cell_cut_type is a 6-bit unsigned integer encoding the pair of
         *             cutting edges within the cell:
         *             - Bit 0: Cut from cell vertex v0 to v1.
         *             - Bit 1: Cut from cell vertex v1 to v0.
         *             - Bit 2: Cut from cell vertex v0 to v2.
         *             - Bit 3: Cut from cell vertex v2 to v0.
         *             - Bit 4: Cut from cell vertex v0 to v4.
         *             - Bit 5: Cut from cell vertex v4 to v0.
         *
         * @note The tracing continues until the loop closes (returns to the starting cell
         *       and edge configuration), or until the boundary is reached. The function
         *       does not validate whether the traced loop is topologically valid.
         */
        void find_sheet_hexes(
            const GEO::Mesh& mesh,
            GEO::index_t start_c,
            GEO::index_t start_le,
            std::vector<std::pair<GEO::index_t, GEO::Numeric::uint8>>& sheet_hexes);
}

#endif //GEOLIO_HEXAHEDRON_OPERATIONS_H