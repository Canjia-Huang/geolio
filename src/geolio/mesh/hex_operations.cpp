//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/6/25.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//

#include "hex_operations.h"
#include <cassert>

namespace geolio
{
    void find_sheet_hexes(
        const GEO::Mesh& mesh,
        const GEO::index_t start_c,
        const GEO::index_t start_le,
        std::vector<std::pair<GEO::index_t, GEO::Numeric::uint8>>& sheet_hexes
        ) {
        // LOG::TRACE("{}({}, {})", __FUNCTION__, start_c, start_le);
        assert(start_c < mesh.cells.nb());
        assert(start_le < 12);

        std::unordered_map<GEO::index_t, GEO::Numeric::uint8> sheet_hex_type_map; // [c] -> type
        std::vector<GEO::Numeric::uint8> mesh_ce_cut(12*mesh.cells.nb(), 0); /*
            [12*c+le] -> 0: not cut, 1<<0: cut ev0->ev1, 1<<1: cut ev1->ev0 */

        std::vector<std::tuple<GEO::index_t, GEO::index_t, bool>> stack; // (c, le, inverse)
        stack.emplace_back(start_c, start_le, false);
        while (!stack.empty()) {
            const auto [c, le, inverse] = stack.back();
            stack.pop_back();

            sheet_hex_type_map.try_emplace(c, 0); // type will be set later (set 0 here)

            if (mesh_ce_cut[12*c+le] & (inverse ? (1<<1) : (1<<0))) // this edge has been cut
                continue;

            for (GEO::index_t i = 0; i < 4; ++i) { // loop edges/facets nb == 4
                const auto& lle = HEX_LE_LOOP_LE[le][i];
                assert(lle < 12);

                const auto lle_inverse = inverse ? HEX_LE_LOOP_LE_ORIENT[le][i] : (!HEX_LE_LOOP_LE_ORIENT[le][i]); /*
                    HEX_LE_LOOP_LE_ORIENT[le][i] -> whether it is consistent with the first le?
                    inverse                      -> whether it need to be inverse? */
                if (lle_inverse)
                    mesh_ce_cut[12*c+lle] |= 1<<1;
                else
                    mesh_ce_cut[12*c+lle] |= 1<<0;

                /* Find next facets */
                if (const auto& nc = mesh.cells.adjacent(c, HEX_LE_LOOP_LF[le][i]);
                    nc != GEO::NO_CELL
                    ) {
                    const auto& ev0 = mesh.cells.edge_vertex(c, lle, 0);
                    const auto& ev1 = mesh.cells.edge_vertex(c, lle, 1);
                    const auto nle = find_hex_edge(mesh, nc, ev0, ev1);
                    assert(nle < 12);

                    if (mesh.cells.edge_vertex(c, lle, 0) == mesh.cells.edge_vertex(nc, nle, 0)) {
                        assert(mesh.cells.edge_vertex(c, lle, 1) == mesh.cells.edge_vertex(nc, nle, 1));
                        if (!(mesh_ce_cut[12*nc+nle] & (lle_inverse ? (1<<1) : (1<<0)))) // this edge is not being cut by this orientation
                            stack.emplace_back(nc, nle, lle_inverse);
                    }
                    else {
                        assert(mesh.cells.edge_vertex(c, lle, 0) == mesh.cells.edge_vertex(nc, nle, 1));
                        assert(mesh.cells.edge_vertex(c, lle, 1) == mesh.cells.edge_vertex(nc, nle, 0));
                        if (!(mesh_ce_cut[12*nc+nle] & (!lle_inverse ? (1<<1) : (1<<0)))) // this edge is not being cut by this orientation
                            stack.emplace_back(nc, nle, !lle_inverse);
                    }
                }
            }
        }

        /* Output */
        for (auto& [c, cut_type] : sheet_hex_type_map) { // 0bit (v0->v1), 1bit (v1->v0), 2bit (v0->v2), 3bit (v2->v0), 4bit (v0->v4), 5bit(v4->v0)
            for (GEO::index_t le = 0; le < 12; ++le) {
                if (mesh_ce_cut[12*c+le] == 0)
                    continue;

                if (mesh_ce_cut[12*c+le] & (1<<0)) { // cut le's ev0->ev1
                    switch (le) {
                        case 0:     cut_type |= 1<<0; break;
                        case 1:     cut_type |= 1<<2; break;
                        case 2:     cut_type |= 1<<1; break;
                        case 3:     cut_type |= 1<<3; break;
                        case 4:     cut_type |= 1<<0; break;
                        case 5:     cut_type |= 1<<2; break;
                        case 6:     cut_type |= 1<<1; break;
                        case 7:     cut_type |= 1<<3; break;
                        case 8:     [[fallthrough]];
                        case 9:     [[fallthrough]];
                        case 10:    [[fallthrough]];
                        case 11:    cut_type |= 1<<4; break;
                        default:    assert(0);
                    }
                }
                if (mesh_ce_cut[12*c+le] & (1<<1)) { // cut le's ev1->ev0
                    switch (le) {
                        case 0:     cut_type |= 1<<1; break;
                        case 1:     cut_type |= 1<<3; break;
                        case 2:     cut_type |= 1<<0; break;
                        case 3:     cut_type |= 1<<2; break;
                        case 4:     cut_type |= 1<<1; break;
                        case 5:     cut_type |= 1<<3; break;
                        case 6:     cut_type |= 1<<0; break;
                        case 7:     cut_type |= 1<<2; break;
                        case 8:     [[fallthrough]];
                        case 9:     [[fallthrough]];
                        case 10:    [[fallthrough]];
                        case 11:    cut_type |= 1<<5; break;
                        default:    assert(0);
                    }
                }
            }
        }

        sheet_hexes.clear();
        sheet_hexes.reserve(sheet_hex_type_map.size());
        for (const auto& [c, cut_type] : sheet_hex_type_map)
            sheet_hexes.emplace_back(c, cut_type);
    }
}