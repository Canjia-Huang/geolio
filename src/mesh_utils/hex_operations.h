//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/3/31.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOGRAM_MESH_UTILS_HEXAHEDRON_OPERATIONS_H
#define GEOGRAM_MESH_UTILS_HEXAHEDRON_OPERATIONS_H

#include <geogram/mesh/mesh.h>
#include <cassert>
#include "hex_descriptor.h"

namespace GEO::MeshUtils::Hex
{
    /**
     * Finds the local vertex index in a hexahedron from a global vertex index.
     *
     * @param[in] M  The hexahedral mesh to query
     * @param[in] c  Index of the hexahedral cell to search
     * @param[in] v  Global vertex index to locate in cell @p c
     * @return Local vertex index (0-7) in @p c if found; otherwise GEO::NO_INDEX
     */
    inline GEO::index_t find_hex_vertex(
        const GEO::Mesh& M,
        const GEO::index_t c,
        const GEO::index_t v
        ) {
        assert(c < M.cells.nb());
        for (GEO::index_t lv = 0; lv < 8; ++lv) {
            if (M.cells.vertex(c, lv) == v)
                return lv;
        }
        return GEO::NO_INDEX;
    }

    /**
     * Finds the local edge index in a hexahedron from two local endpoint vertices.
     *
     * Edge direction is ignored.
     *
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
     * Finds the local edge index in a hexahedron from two endpoint vertex.
     *
     * The search is performed in cell @p c using global vertex indices @p ev0 and @p ev1.
     * Edge direction is ignored.
     *
     * @param[in] M    The hexahedral mesh to query
     * @param[in] c    Index of the hexahedral cell to search
     * @param[in] v0  Global vertex index of one endpoint
     * @param[in] v1  Global vertex index of the other endpoint
     * @return Local edge index (0-11) in @p c if found; otherwise GEO::NO_INDEX
     */
    inline GEO::index_t find_hex_edge(
        const GEO::Mesh& M,
        const GEO::index_t c,
        const GEO::index_t v0,
        const GEO::index_t v1
        ) {
        assert(c < M.cells.nb());
        for (GEO::index_t lv = 0; lv < 8; ++lv) {
            if (M.cells.vertex(c, lv) == v0) {
                for (const auto& adj_lv : HEX_LV_ADJACENT_LV[lv]) {
                    if (M.cells.vertex(c, adj_lv) == v1)
                        return find_hex_edge_from_local_vertices(lv, adj_lv);
                }
                break;
            }
        }
        return GEO::NO_INDEX;
    }

    /**
     * Finds a local facet index from three local vertices of a hexahedron.
     *
     * The three local vertices can be provided in any order.
     *
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
     * Finds a local facet index from four local vertices of a hexahedron.
     *
     * The four local vertices can be provided in any order.
     *
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
     * Finds the local facet index in a hexahedron from three global vertices.
     *
     * The three vertices can be provided in any order.
     *
     * @param[in] M   The hexahedral mesh to query
     * @param[in] c   Index of the hexahedral cell to search
     * @param[in] v0  Global vertex index on the target facet
     * @param[in] v1  Global vertex index on the target facet
     * @param[in] v2  Global vertex index on the target facet
     * @return Local facet index (0-5) in @p c if found; otherwise GEO::NO_INDEX
     */
    inline GEO::index_t find_hex_facet(
        const GEO::Mesh& M,
        const GEO::index_t c,
        const GEO::index_t v0,
        const GEO::index_t v1,
        const GEO::index_t v2
        ) {
        assert(c < M.cells.nb());
        assert(v0 < M.vertices.nb());
        assert(v1 < M.vertices.nb());
        assert(v2 < M.vertices.nb());
        const auto lv0 = find_hex_vertex(M, c, v0);
        const auto lv1 = find_hex_vertex(M, c, v1);
        const auto lv2 = find_hex_vertex(M, c, v2);
        if (lv0 == GEO::NO_INDEX || lv1 == GEO::NO_INDEX || lv2 == GEO::NO_INDEX)
            return GEO::NO_INDEX;
        return find_hex_facet_from_local_vertices(lv0, lv1, lv2);
    }

    /**
     * Collects hexahedra incident to a mesh edge in traversal order.
     *
     * Starting from a cell-local-edge seed, this function traces the edge
     * ring (or border chain) and outputs an ordered list of
     * (cell, local-edge, local-facet) tuples, where each local facet contains
     * the queried edge.
     *
     * @param[in]     M                The hexahedral mesh to query
     * @param[in]     start_c          Index of the seed cell
     * @param[in]     start_le         Local edge index (0-11) in @p start_c
     * @param[in,out] ordered_c_le_lf  Output ordered (cell, local-edge, local-facet) list; existing contents are cleared
     * @return true if the queried edge is on the mesh border; false if it is an interior edge
     */
    inline bool get_edge_incident_hexahedra(
        const GEO::Mesh& M,
        const GEO::index_t start_c,
        const GEO::index_t start_le,
        std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>>& ordered_c_le_lf
        ) {
        assert(start_c < M.cells.nb());
        assert(start_le < 12);

        const auto ev0 = M.cells.edge_vertex(start_c, start_le, 0);
        const auto ev1 = M.cells.edge_vertex(start_c, start_le, 1);
        bool is_on_border = false;

        std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>> next_ordered_c_le_lf;
        std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>> prev_ordered_c_le_lf;
        {
            GEO::index_t c = start_c;
            GEO::index_t le = start_le;
            GEO::index_t lf = M.cells.edge_adjacent_facet(start_c, start_le, 0);
            for (;;) {
                next_ordered_c_le_lf.emplace_back(c, le, lf);

                const GEO::index_t nc = M.cells.adjacent(c, lf);
                if (nc == GEO::NO_CELL) {
                    is_on_border = true;
                    break;
                }
                if (nc == start_c) // a loop
                    break;

                /* Get next lf */
                le = find_hex_edge(M, nc, ev0, ev1);
                assert(le != GEO::NO_INDEX);
                lf = M.cells.edge_adjacent_facet(nc, le, 0);
                if (M.cells.adjacent(nc, lf) == c)
                    lf = M.cells.edge_adjacent_facet(nc, le, 1);
                assert(M.cells.adjacent(nc, lf) != c);
                c = nc;
            }
        }

        if (is_on_border) {
            GEO::index_t c = start_c;
            GEO::index_t lf = M.cells.edge_adjacent_facet(start_c, start_le, 1);
            for (;;) {
                const GEO::index_t nc = M.cells.adjacent(c, lf);
                if (nc == GEO::NO_CELL)
                    break;

                /* Get next lf */
                const auto le = find_hex_edge(M, nc, ev0, ev1);
                assert(le != GEO::NO_INDEX);
                lf = M.cells.edge_adjacent_facet(nc, le, 0);
                GEO::index_t lf1 = M.cells.edge_adjacent_facet(nc, le, 1);
                if (M.cells.adjacent(nc, lf) == c)
                    std::swap(lf, lf1);
                assert(M.cells.adjacent(nc, lf) != c && M.cells.adjacent(nc, lf1) == c);
                c = nc;

                prev_ordered_c_le_lf.emplace_back(c, le, lf1);
            }
        }

        /* Output */
        std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>>().swap(ordered_c_le_lf);
        ordered_c_le_lf.reserve(next_ordered_c_le_lf.size() + prev_ordered_c_le_lf.size());
        for (GEO::index_t i = 0, i_end = prev_ordered_c_le_lf.size(); i < i_end; ++i)
            ordered_c_le_lf.push_back(prev_ordered_c_le_lf[i_end-i-1]);
        for (const auto& c_lf : next_ordered_c_le_lf)
            ordered_c_le_lf.push_back(c_lf);

        return is_on_border;
    }
}

#endif //GEOGRAM_MESH_UTILS_HEXAHEDRON_OPERATIONS_H