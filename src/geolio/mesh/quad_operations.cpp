//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/21.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "quad_operations.h"
#include <cassert>

namespace geolio
{
    void find_sheet_quads(
        const GEO::Mesh& mesh,
        const GEO::index_t start_f,
        const GEO::index_t start_lv,
        std::vector<std::pair<GEO::index_t, GEO::Numeric::uint8>>& sheet_quads
        ) {
        assert(start_f < mesh.facets.nb());
        assert(start_lv < 4);

        std::unordered_map<GEO::index_t, GEO::Numeric::uint8> sheet_quad_type_map; // [f] -> type
        std::vector<GEO::Numeric::uint8> mesh_fc_cut(mesh.facet_corners.nb(), 0); /*
            [facet.corner] -> 0: not cut, 1<<0: cut ev0->ev1, 1<<1: cut ev1->ev0 */

        std::vector<std::tuple<GEO::index_t, GEO::index_t, bool>> stack; // (f, lv, inverse)
        stack.emplace_back(start_f, start_lv, false);
        while (!stack.empty()) {
            const auto [f, lv, inverse] = stack.back();
            const auto fc = mesh.facets.corner(f, lv);
            stack.pop_back();

            sheet_quad_type_map.try_emplace(f, 0); // type will be set later (set 0 here)

            if (mesh_fc_cut[fc] & (inverse ? (1<<1) : (1<<0))) // this edge has been cut
                continue;

            const auto oppo_fc = mesh.facets.corner(f, (lv+2)%4);
            if (inverse) {
                mesh_fc_cut[fc] |= 1<<1;
                mesh_fc_cut[oppo_fc] |= 1<<0;
            }
            else {
                mesh_fc_cut[fc] |= 1<<0;
                mesh_fc_cut[oppo_fc] |= 1<<1;
            }

            /* Find next facet */
            if (const auto& nf = mesh.facet_corners.adjacent_facet(fc);
                nf != GEO::NO_FACET
                ) {
                auto nlv = mesh.facets.find_vertex(nf, mesh.facet_corners.vertex(fc));
                assert(nlv != GEO::NO_INDEX);
                nlv = (nlv+3)%4;

                if (const auto nfc = mesh.facets.corner(nf, nlv);
                    !(mesh_fc_cut[nfc] & (inverse ? (1<<0) : (1<<1))))
                    stack.emplace_back(nf, nlv, !inverse);
            }
            if (const auto& nf = mesh.facet_corners.adjacent_facet(oppo_fc);
                nf != GEO::NO_FACET
                ) {
                auto nlv = mesh.facets.find_vertex(nf, mesh.facet_corners.vertex(oppo_fc));
                assert(nlv != GEO::NO_INDEX);
                nlv = (nlv+3)%4;

                if (const auto nfc = mesh.facets.corner(nf, nlv);
                    !(mesh_fc_cut[nfc] & (inverse ? (1<<1) : (1<<0))))
                    stack.emplace_back(nf, nlv, inverse);
            }
        }

        /* Output */
        for (auto& [f, cut_type] : sheet_quad_type_map) { // 0bit (v0->v1), 1bit (v1->v0), 2bit (v0->v3), 3bit (v3->v0)
            for (GEO::index_t lv = 0; lv < 4; ++lv) {
                const auto fc = mesh.facets.corner(f, lv);

                if (mesh_fc_cut[fc] & (1<<0)) { // cut edge's ev0->ev1
                    switch (lv) {
                        case 0: cut_type |= 1<<0; break;
                        case 1: cut_type |= 1<<2; break;
                        case 2: cut_type |= 1<<1; break;
                        case 3: cut_type |= 1<<3; break;
                        default: assert(0);
                    }
                }
                if (mesh_fc_cut[fc] & (1<<1)) { // cut edge's ev1->ev0
                    switch (lv) {
                        case 0: cut_type |= 1<<1; break;
                        case 1: cut_type |= 1<<3; break;
                        case 2: cut_type |= 1<<0; break;
                        case 3: cut_type |= 1<<2; break;
                        default: assert(0);
                    }
                }
            }
        }

        sheet_quads.clear();
        sheet_quads.reserve(sheet_quad_type_map.size());
        for (const auto& [f, cut_type] : sheet_quad_type_map)
            sheet_quads.emplace_back(f, cut_type);
    }
}