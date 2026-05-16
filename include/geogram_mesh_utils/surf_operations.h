//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/5/16.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOGRAMMESHUTILS_SURF_OPERATIONS_H
#define GEOGRAMMESHUTILS_SURF_OPERATIONS_H

#include <geogram/mesh/mesh.h>
#include <cassert>

namespace GEO::MeshUtils
{
    /**
     * @brief Collect facets incident to a vertex in one-ring order.
     *
     * Starting from facet @p start_f and its local vertex slot @p start_lv, this function traverses
     * all incident triangles around that vertex and outputs ordered pairs (facet index, local vertex index).
     * For interior vertices, the sequence forms a closed ring. For border vertices, the sequence is ordered
     * from one border side to the other.
     *
     * @param[in] M Input triangle mesh.
     * @param[in] _f Seed facet index incident to the target vertex.
     * @param[in] _lv Local vertex index (0, 1, or 2) of the target vertex in @p start_f.
     * @param[out] ordered_f_and_lv Output ordered one-ring list. Each element is (f, lv), where
     *                              @p f is an incident facet and @p lv is the local index of the target
     *                              vertex inside that facet. Existing contents are cleared.
     * @return true if the target vertex is on the mesh border; false if it is an interior vertex.
     */
    inline bool get_vertex_incident_facets(
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

#endif //GEOGRAMMESHUTILS_SURF_OPERATIONS_H
