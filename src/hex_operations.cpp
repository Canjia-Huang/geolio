//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/6/25.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//

#include "geolio/hex_operations.h"
#include <cassert>

namespace geolio
{
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