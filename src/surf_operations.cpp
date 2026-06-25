//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/6/25.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//

#include "geolio/surf_operations.h"
#include <cassert>

namespace geolio
{
    bool get_vertex_incident_facets(
        const GEO::Mesh& M,
        const GEO::index_t _f,
        const GEO::index_t _lv,
        std::vector<std::pair<GEO::index_t, GEO::index_t>>& ordered_f_and_lv
        ) {
        assert(_f < M.facets.nb());
        assert(_lv < M.facets.nb_vertices(_f));

        const GEO::index_t v = M.facets.vertex(_f, _lv);
        bool is_on_border = false;

        std::vector<std::pair<GEO::index_t, GEO::index_t>> next_ordered_f_and_lv;
        std::vector<std::pair<GEO::index_t, GEO::index_t>> prev_ordered_f_and_lv;
        {
            GEO::index_t f = _f;
            GEO::index_t lv = _lv;
            do {
                next_ordered_f_and_lv.emplace_back(f, lv);

                const GEO::index_t next_f = M.facets.adjacent(f, lv);
                if (next_f == GEO::NO_FACET) { // is not 2-manifold vertex
                    is_on_border = true;
                    break;
                }
                f = next_f;
                lv = M.facets.find_vertex(f, v);
                assert(lv != GEO::NO_INDEX);
            } while (f != _f);
        }

        if (is_on_border) { // inverse travel
            GEO::index_t f = _f;
            GEO::index_t lv = (_lv+M.facets.nb_vertices(f)-1)%M.facets.nb_vertices(f);

            for (;;) {
                const GEO::index_t next_f = M.facets.adjacent(f, lv);
                if (next_f == GEO::NO_FACET)
                    break;
                f = next_f;
                lv = M.facets.find_vertex(f, v);
                prev_ordered_f_and_lv.emplace_back(f, lv);
                lv = (lv+M.facets.nb_vertices(f)-1)%M.facets.nb_vertices(f);
            }
        }

        /* Output */
        ordered_f_and_lv.clear();
        ordered_f_and_lv.reserve(next_ordered_f_and_lv.size() + prev_ordered_f_and_lv.size());
        for (GEO::index_t i = 0, i_end = prev_ordered_f_and_lv.size(); i < i_end; ++i)
            ordered_f_and_lv.push_back(prev_ordered_f_and_lv[i_end-i-1]);
        for (const auto& f_lv : next_ordered_f_and_lv)
            ordered_f_and_lv.push_back(f_lv);

        return is_on_border;
    }
}