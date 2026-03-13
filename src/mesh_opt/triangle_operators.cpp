//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/3/13.
// Copyright (c) 2026 Graphics@XMU. All rights reserved.
//

#include "triangle_operators.h"
#include "common/log.h"

namespace MeshOpt
{
    void triangle_split_edge(
       GEO::Mesh& M,
       const GEO::index_t f,
       const GEO::index_t lv,
       const GEO::index_t new_v,
       const double r,
       const GEO::index_t new_f1,
       const GEO::index_t new_f2
       ) {
        LOG::TRACE("{}({}, {}, {}, {}, {}, {})", __FUNCTION__, f, lv, new_v, r, new_f1, new_f2);
        assert(new_v < M.vertices.nb());
        assert(new_f1 < M.facets.nb());

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
        const GEO::index_t af = M.facets.adjacent(f, lv0);
        const GEO::index_t af1 = M.facets.adjacent(f, lv1);

        /* Set new point */
        const auto& p0 = M.facets.point(f, lv0);
        const auto& p1 = M.facets.point(f, lv1);
        M.vertices.point(new_v) = (1-r)*p0 + r*p1;

        /* == Set facet vertices =================================================================================== */
        M.facets.set_vertex(f, lv1, new_v);
        M.facets.set_vertex(new_f1, lv0, new_v);
        M.facets.set_vertex(new_f1, lv1, v1);
        M.facets.set_vertex(new_f1, lv2, v2);

        /* Set facet adjacency */
        M.facets.set_adjacent(f, lv1, new_f1);
        assert(M.facets.adjacent(new_f1, lv0) == GEO::NO_FACET); // will be set later
        M.facets.set_adjacent(new_f1, lv1, af1);
        M.facets.set_adjacent(new_f1, lv2, f);
        if (af1 != GEO::NO_FACET) {
            assert(M.facets.find_vertex(af1, v2) != GEO::NO_INDEX);
            M.facets.set_adjacent(af1, M.facets.find_vertex(af1, v2), new_f1);
        }

        /* Split adjacent facet */
        if (af != GEO::NO_FACET) {
            assert(new_f2 < M.facets.nb());

            /*
             *  v2 ------ v1        v2 ------ v1
             *  |       / |         | \  af / |
             *  | af  /   |    ->   |nf2\ /   |
             *  |   /     |         |   / \   |
             *  | /       |         | /     \ |
             *  v0 -------+        v0 --------+
             */
            const GEO::index_t nlv0 = M.facets.find_vertex(af, v1);
            const GEO::index_t nlv1 = (nlv0+1)%3;
            const GEO::index_t nlv2 = (nlv0+2)%3;
            assert(nlv0 != GEO::NO_INDEX);
            const GEO::index_t nv0 = M.facets.vertex(af, nlv0);
            // const GEO::index_t nv1 = M.facets.vertex(af, nlv1);
            const GEO::index_t nv2 = M.facets.vertex(af, nlv2);
            const GEO::index_t naf2 = M.facets.adjacent(af, nlv2);

            /* == Set facet vertices =============================================================================== */
            M.facets.set_vertex(af, nlv0, new_v);
            M.facets.set_vertex(new_f2, nlv0, nv0);
            M.facets.set_vertex(new_f2, nlv1, new_v);
            M.facets.set_vertex(new_f2, nlv2, nv2);

            /* Set facet adjacency */
            M.facets.set_adjacent(new_f1, lv0, new_f2);
            M.facets.set_adjacent(af, nlv2, new_f2);
            M.facets.set_adjacent(new_f2, nlv0, new_f1);
            M.facets.set_adjacent(new_f2, nlv1, af);
            M.facets.set_adjacent(new_f2, nlv2, naf2);
            if (naf2 != GEO::NO_FACET) {
                assert(M.facets.find_vertex(new_f2, nv0) != GEO::NO_INDEX);
                M.facets.set_adjacent(naf2, M.facets.find_vertex(new_f2, nv0), new_f2);
            }
        }
    }
}