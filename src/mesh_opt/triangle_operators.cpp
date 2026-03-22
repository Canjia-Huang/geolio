//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/3/13.
// Copyright (c) 2026 Graphics@XMU. All rights reserved.
//

#include "triangle_operators.h"
#include "common/log.h"

namespace ProgressiveMeshOpt
{
    void get_vertex_one_ring_triangles(
        const GEO::Mesh& M,
        const GEO::index_t start_f,
        const GEO::index_t start_lv,
        std::vector<std::pair<GEO::index_t, GEO::index_t>>& ordered_f_and_lv
        ) {
        // LOG::TRACE("{}({}, {})", __FUNCTION__, f, v);
        assert(start_f < M.facets.nb());
        assert(start_lv < 3);

        const GEO::index_t v = M.facets.vertex(start_f, start_lv);
        bool is_on_border = false;

        std::vector<std::pair<GEO::index_t, GEO::index_t>> next_ordered_f_and_lv;
        std::vector<std::pair<GEO::index_t, GEO::index_t>> prev_ordered_f_and_lv;
        {
            GEO::index_t f = start_f;
            GEO::index_t lv = start_lv;
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
            } while (f != start_f);
        }

        if (is_on_border) { // inverse travel
            GEO::index_t f = start_f;
            GEO::index_t lv = (start_lv+2)%3;


            for (;;) {
                const GEO::index_t next_f = M.facets.adjacent(f, lv);
                if (next_f == GEO::NO_FACET)
                    break;
                f = next_f;
                lv = M.facets.find_vertex(f, v);
                prev_ordered_f_and_lv.emplace_back(f, lv);
                lv = (lv+2)%3;
            }
        }

        /* Output */
        std::vector<std::pair<GEO::index_t, GEO::index_t>>().swap(ordered_f_and_lv);
        ordered_f_and_lv.reserve(next_ordered_f_and_lv.size() + prev_ordered_f_and_lv.size());
        for (GEO::index_t i = 0, i_end = prev_ordered_f_and_lv.size(); i < i_end; ++i)
            ordered_f_and_lv.push_back(prev_ordered_f_and_lv[i_end-i-1]);
        for (const auto& f_lv : next_ordered_f_and_lv)
            ordered_f_and_lv.push_back(f_lv);
    }

    void split_triangle_edge(
       GEO::Mesh& M,
       const GEO::index_t f,
       const GEO::index_t lv,
       const double r,
       const GEO::index_t new_v,
       const GEO::index_t new_f0,
       const GEO::index_t new_f1
       ) {
        // LOG::TRACE("{}({}, {}, {}, {}, {}, {})", __FUNCTION__, f, lv, r, new_v, new_f0, new_f1);
        assert(f < M.facets.nb());
        assert(lv < 3);
        assert(r >= 0 && r <= 1);
        assert(new_v < M.vertices.nb());
        assert(new_f0 < M.facets.nb());

        /*
         *  +-------- v0        +-------- v0
         *  |       / |         | \     / |
         *  |     /   |    ->   |   \ /   |
         *  |   /  f  |         |   / \ f |
         *  | /       |         | / nf1 \ |
         *  v1 ------ v2        v1 ------ v2
         */
        const GEO::index_t lv0 = lv;
        const GEO::index_t lv1 = (lv+1)%3;
        const GEO::index_t lv2 = (lv+2)%3;
        // const GEO::index_t v0 = M.facets.vertex(f, lv0);
        const GEO::index_t v1 = M.facets.vertex(f, lv1);
        const GEO::index_t v2 = M.facets.vertex(f, lv2);
        const GEO::index_t af0 = M.facets.adjacent(f, lv0);
        const GEO::index_t af1 = M.facets.adjacent(f, lv1);

        /* Set new point */
        const auto& p0 = M.facets.point(f, lv0);
        const auto& p1 = M.facets.point(f, lv1);
        M.vertices.point(new_v) = (1-r)*p0 + r*p1;

        /* == Set facet vertices  */
        M.facets.set_vertex(f, lv1, new_v);
        M.facets.set_vertex(new_f0, lv0, new_v);
        M.facets.set_vertex(new_f0, lv1, v1);
        M.facets.set_vertex(new_f0, lv2, v2);

        /* Set facet adjacency */
        M.facets.set_adjacent(f, lv1, new_f0);
        assert(M.facets.adjacent(new_f0, lv0) == GEO::NO_FACET); // will be set later
        M.facets.set_adjacent(new_f0, lv1, af1);
        M.facets.set_adjacent(new_f0, lv2, f);
        if (af1 != GEO::NO_FACET) {
            assert(M.facets.find_vertex(af1, v2) != GEO::NO_INDEX);
            M.facets.set_adjacent(af1, M.facets.find_vertex(af1, v2), new_f0);
        }

        /* == Split adjacent facet ================================================================================= */
        if (af0 != GEO::NO_FACET) {
            assert(new_f1 < M.facets.nb());

            /*
             *  v2 ------ v1        v2 ------ v1
             *  |       / |         | \  af / |
             *  | af  /   |    ->   |nf2\ /   |
             *  |   /     |         |   / \   |
             *  | /       |         | /     \ |
             *  v0 -------+        v0 --------+
             */
            const GEO::index_t nlv0 = M.facets.find_vertex(af0, v1);
            assert(nlv0 != GEO::NO_INDEX);
            const GEO::index_t nlv1 = (nlv0+1)%3;
            const GEO::index_t nlv2 = (nlv0+2)%3;
            const GEO::index_t nv0 = M.facets.vertex(af0, nlv0);
            // const GEO::index_t nv1 = M.facets.vertex(af0, nlv1);
            const GEO::index_t nv2 = M.facets.vertex(af0, nlv2);
            const GEO::index_t naf2 = M.facets.adjacent(af0, nlv2);

            /* == Set facet vertices */
            M.facets.set_vertex(af0, nlv0, new_v);
            M.facets.set_vertex(new_f1, nlv0, nv0);
            M.facets.set_vertex(new_f1, nlv1, new_v);
            M.facets.set_vertex(new_f1, nlv2, nv2);

            /* Set facet adjacency */
            M.facets.set_adjacent(new_f0, lv0, new_f1);
            M.facets.set_adjacent(af0, nlv2, new_f1);
            M.facets.set_adjacent(new_f1, nlv0, new_f0);
            M.facets.set_adjacent(new_f1, nlv1, af0);
            M.facets.set_adjacent(new_f1, nlv2, naf2);
            if (naf2 != GEO::NO_FACET) {
                assert(M.facets.find_vertex(new_f1, nv0) != GEO::NO_INDEX);
                M.facets.set_adjacent(naf2, M.facets.find_vertex(new_f1, nv0), new_f1);
            }
        }
    }

    void collapse_triangle_edge(
        GEO::Mesh& M,
        const GEO::index_t f,
        const GEO::index_t lv,
        const double r,
        GEO::index_t& disuse_v,
        GEO::index_t& disuse_f0,
        GEO::index_t& disuse_f1
        ) {
        // LOG::TRACE("{}({}, {}, {})", __FUNCTION__, f, lv, r);
        assert(f < M.facets.nb());
        assert(lv < 3);
        assert(r >= 0 && r <= 1);

        /*
         *  v0 --------+            ++
         *  | \  af2 /              |  \
         *  |   \  /                |af2 \
         *  | f  v2         ->      v0 --- v2
         *  |   /  \                |af1 /
         *  | /  af1 \              |  /
         *  v1 --------+            ++
         */
        const GEO::index_t lv0 = lv;
        const GEO::index_t lv1 = (lv+1)%3;
        const GEO::index_t lv2 = (lv+2)%3;
        const GEO::index_t v0 = M.facets.vertex(f, lv0);
        const GEO::index_t v1 = M.facets.vertex(f, lv1);
        const GEO::index_t v2 = M.facets.vertex(f, lv2);
        const GEO::index_t af0 = M.facets.adjacent(f, lv0);
        const GEO::index_t af1 = M.facets.adjacent(f, lv1);
        const GEO::index_t af2 = M.facets.adjacent(f, lv2);

        /* Set collapsed point (v0) */
        const auto& p0 = M.facets.point(f, lv0);
        const auto& p1 = M.facets.point(f, lv1);
        M.vertices.point(v0) = (1-r)*p0 + r*p1;
        disuse_v = v1;

        disuse_f0 = f;

        /* Find all (f, lv) that incident to v1 */
        std::vector<std::pair<GEO::index_t, GEO::index_t>> ordered_f_and_lv;
        get_vertex_one_ring_triangles(M, f, lv1, ordered_f_and_lv);

        /* Set facet adjacency */
        if (af1 != GEO::NO_FACET) {
            assert(M.facets.find_vertex(af1, v2) != GEO::NO_INDEX);
            M.facets.set_adjacent(af1, M.facets.find_vertex(af1, v2), af2);
        }
        if (af2 != GEO::NO_FACET) {
            assert(M.facets.find_vertex(af2, v0) != GEO::NO_INDEX);
            M.facets.set_adjacent(af2, M.facets.find_vertex(af2, v0), af1);
        }

        /* == Collapse adjacent facet ============================================================================== */
        if (af0 != GEO::NO_FACET) {
            /*
             *  +-------- nv1                 ++
             *    \ naf1 / |                 / |
             *      \  /   |              /naf1|
             *      nv2 af0|     ->    nv2 --- nv1
             *      /  \   |              \naf2|
             *    / naf2 \ |                \  |
             *  +-------- nv0                 ++
             */
            const GEO::index_t nlv0 = M.facets.find_vertex(af0, v1);
            assert(nlv0 != GEO::NO_INDEX);
            const GEO::index_t nlv1 = (nlv0+1)%3;
            const GEO::index_t nlv2 = (nlv0+2)%3;
            const GEO::index_t nv0 = M.facets.vertex(af0, nlv0);
            // const GEO::index_t nv1 = M.facets.vertex(af0, nlv1);
            const GEO::index_t nv2 = M.facets.vertex(af0, nlv2);
            const GEO::index_t naf1 = M.facets.adjacent(af0, nlv1);
            const GEO::index_t naf2 = M.facets.adjacent(af0, nlv2);

            disuse_f1 = af0;

            /* Set facet adjacency */
            if (naf1 != GEO::NO_FACET) {
                assert(M.facets.find_vertex(naf1, nv2) != GEO::NO_INDEX);
                M.facets.set_adjacent(naf1, M.facets.find_vertex(naf1, nv2), naf2);
            }
            if (naf2 != GEO::NO_FACET) {
                assert(M.facets.find_vertex(naf2, nv0) != GEO::NO_INDEX);
                M.facets.set_adjacent(naf2, M.facets.find_vertex(naf2, nv0), naf1);
            }
        }
        else
            disuse_f1 = GEO::NO_FACET;

        /* Set facet vertices */
        for (const auto& [adj_f, adj_lv] : ordered_f_and_lv)
            M.facets.set_vertex(adj_f, adj_lv, v0);
    }

    void flip_triangle_edge(
        GEO::Mesh& M,
        const GEO::index_t f,
        const GEO::index_t lv
        ) {
        LOG::TRACE("{}({}, {})", __FUNCTION__, f, lv);
        assert(f < M.facets.nb());
        assert(lv < 3);

        /*
         *          af0                            af0
         *      v3 ------ v0                   v3 ------ v0
         *      |       / |                    | \       |
         *      | af  /   |        ->          |   \  f  |
         * af3  |   /  f  |  af2          af3  | af  \   |  af2
         *      | /       |                    |       \ |
         *      v1 ------ v2                   v1 ------ v2
         *          af1                            af1
         */
        const GEO::index_t lv1 = (lv+1)%3;
        const GEO::index_t lv2 = (lv+2)%3;
        // const GEO::index_t v0 = M.facets.vertex(f, lv);
        const GEO::index_t v1 = M.facets.vertex(f, lv1);
        const GEO::index_t v2 = M.facets.vertex(f, lv2);

        const GEO::index_t af = M.facets.adjacent(f, lv);
        if (af == GEO::NO_FACET)
            return;
        const GEO::index_t af1 = M.facets.adjacent(f, lv1);
        // const GEO::index_t af2 = M.facets.adjacent(f, lv2);

        const GEO::index_t nlv0 = M.facets.find_vertex(af, v1);
        assert(nlv0 != GEO::NO_INDEX);
        const GEO::index_t nlv1 = (nlv0+1)%3;
        const GEO::index_t nlv2 = (nlv0+2)%3;
        const GEO::index_t v3 = M.facets.vertex(af, nlv2);

        const GEO::index_t af0 = M.facets.adjacent(af, nlv1);
        // const GEO::index_t af3 = M.facets.adjacent(af, nlv2);

        /* Set vertices */
        M.facets.set_vertex(f, lv1, v3);
        M.facets.set_vertex(af, nlv1, v2);

        /* Set adjacency */
        M.facets.set_adjacent(f, lv, af0);
        M.facets.set_adjacent(f, lv1, af);
        M.facets.set_adjacent(af, nlv0, af1);
        M.facets.set_adjacent(af, nlv1, f);
        if (af0 != GEO::NO_FACET) {
            assert(M.facets.find_vertex(af0, v3) != GEO::NO_INDEX);
            M.facets.set_adjacent(af0, M.facets.find_vertex(af0, v3), f);
        }
        if (af1 != GEO::NO_FACET) {
            assert(M.facets.find_vertex(af1, v2) != GEO::NO_INDEX);
            M.facets.set_adjacent(af1, M.facets.find_vertex(af1, v2), af);
        }
    }
}