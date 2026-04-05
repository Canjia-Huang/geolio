//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/3/31.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "hexahedron_operators.h"
#include <cassert>

namespace
{
    constexpr GEO::index_t ENCODED_EDGE_TO_LE(
        const uint8_t encoded_edges
        ) {
        switch (encoded_edges) {
            case (1<<0) | (1<<1): return 0;
            case (1<<1) | (1<<3): return 1;
            case (1<<3) | (1<<2): return 2;
            case (1<<2) | (1<<0): return 3;
            case (1<<4) | (1<<5): return 4;
            case (1<<5) | (1<<7): return 5;
            case (1<<7) | (1<<6): return 6;
            case (1<<6) | (1<<4): return 7;
            case (1<<0) | (1<<4): return 8;
            case (1<<1) | (1<<5): return 9;
            case (1<<3) | (1<<7): return 10;
            case (1<<2) | (1<<6): return 11;
            default:
                assert(0);
                return GEO::NO_INDEX;
        }
    }

    constexpr GEO::index_t ENCODED_EDGE_TO_LE(
        const GEO::index_t lv0,
        const GEO::index_t lv1
        ) {
        return ENCODED_EDGE_TO_LE((1<<lv0) | (1<<lv1));
    }

    constexpr std::array<std::array<GEO::index_t, 3>, 8> CELL_LV_ADJACENT_LV = {
        {
            {1, 2, 4},
            {0, 3, 5},
            {0, 3, 6},
            {1, 2, 7},
            {0, 5, 6},
            {1, 4, 7},
            {2, 4, 7},
            {3, 5, 6}
        }
    };
}

namespace GEO::MeshUtils
{
    GEO::index_t find_hex_edge(
        const GEO::Mesh& M,
        const GEO::index_t c,
        const GEO::index_t ev0,
        const GEO::index_t ev1
        ) {
        assert(c < M.cells.nb());
        for (GEO::index_t lv = 0; lv < 8; ++lv) {
            if (M.cells.vertex(c, lv) == ev0) {
                for (const auto& adj_lv : CELL_LV_ADJACENT_LV[lv]) {
                    if (M.cells.vertex(c, adj_lv) == ev1)
                        return ENCODED_EDGE_TO_LE(lv, adj_lv);
                }
                break;
            }
        }
        return GEO::NO_INDEX;
    }

    bool get_edge_incident_hexahedra(
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
