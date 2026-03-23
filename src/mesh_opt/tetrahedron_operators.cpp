//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/3/23.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//

#include "tetrahedron_operators.h"
#include <cassert>
#include "common/log.h"

namespace
{
    /*
     * Lookup table for mapping cell local facet vertices to cell local vertices in a tetrahedron
     * cf_lf_lv_to_c_lv[local_facet_index][local_vertex_index] = cell_local_vertex_index
     */
    std::array<std::array<GEO::index_t, 3>, 4> cf_lf_lv_to_c_lv = {
        {{1, 3, 2}, {0, 2, 3}, {3, 1, 0}, {0, 1, 2}}
    };
}

namespace ProgressiveMeshOpt::Tet
{
    void edge_swap_2_3(
        GEO::Mesh& M,
        const GEO::index_t c,
        const GEO::index_t lf,
        const GEO::index_t new_c
        ) {
        assert(c < M.cells.nb());
        assert(lf < 4);
        assert(new_c < M.cells.nb());

        const GEO::index_t nc = M.cells.adjacent(c, lf);
        if (nc == GEO::NO_CELL)
            return;

        const GEO::index_t v = M.cells.vertex(c, lf);
        const GEO::index_t v0 = M.cells.facet_vertex(c, lf, 0);
        const GEO::index_t v1 = M.cells.facet_vertex(c, lf, 1);
        const GEO::index_t v2 = M.cells.facet_vertex(c, lf, 2);
        const GEO::index_t lv0 = cf_lf_lv_to_c_lv[lf][0];
        const GEO::index_t lv1 = cf_lf_lv_to_c_lv[lf][1];
        const GEO::index_t lv2 = cf_lf_lv_to_c_lv[lf][2];
        const GEO::index_t nc0 = M.cells.adjacent(c, lv0);
        const GEO::index_t nc1 = M.cells.adjacent(c, lv1);
        const GEO::index_t nc2 = M.cells.adjacent(c, lv2);

        const GEO::index_t nlf = M.cells.find_tet_facet(nc, v2, v1, v0);
        assert(nlf != GEO::NO_INDEX);
        GEO::index_t nlv0{GEO::NO_INDEX}, nlv1{GEO::NO_INDEX}, nlv2{GEO::NO_INDEX};
        for (GEO::index_t i = 0; i < 3; ++i) {
            if (M.cells.facet_vertex(nc, nlf, i) == v0) { // cell_vertex(nc, nlv0) == cell_vertex(c, lv0)
                nlv0 = cf_lf_lv_to_c_lv[nlf][i];
                nlv1 = cf_lf_lv_to_c_lv[nlf][(i+1)%3];
                nlv2 = cf_lf_lv_to_c_lv[nlf][(i+2)%3];
                break;
            }
        }
        assert(nlv0 != GEO::NO_INDEX && nlv1 != GEO::NO_INDEX && nlv2 != GEO::NO_INDEX);
        const GEO::index_t nv = M.cells.vertex(nc, nlf);
        const GEO::index_t nv0 = M.cells.vertex(nc, nlv0);
        const GEO::index_t nv1 = M.cells.vertex(nc, nlv1);
        const GEO::index_t nv2 = M.cells.vertex(nc, nlv2);
        const GEO::index_t nc_nc0 = M.cells.adjacent(nc, nlv0);
        const GEO::index_t nc_nc1 = M.cells.adjacent(nc, nlv1);
        const GEO::index_t nc_nc2 = M.cells.adjacent(nc, nlv2);

        /* Set cell vertices */
        M.cells.set_vertex(c, 0, v);
        M.cells.set_vertex(c, 1, nv);
        M.cells.set_vertex(c, 2, v1);
        M.cells.set_vertex(c, 3, v0);
        M.cells.set_vertex(nc, 0, v);
        M.cells.set_vertex(nc, 1, nv);
        M.cells.set_vertex(nc, 2, v2);
        M.cells.set_vertex(nc, 3, v1);
        M.cells.set_vertex(new_c, 0, v);
        M.cells.set_vertex(new_c, 1, nv);
        M.cells.set_vertex(new_c, 2, v0);
        M.cells.set_vertex(new_c, 3, v2);

        /* Set adjacency */
        M.cells.set_adjacent(c, 0, nc_nc1);
        M.cells.set_adjacent(c, 1, nc2);
        M.cells.set_adjacent(c, 2, new_c);
        M.cells.set_adjacent(c, 3, nc);
        M.cells.set_adjacent(nc, 0, nc_nc0);
        M.cells.set_adjacent(nc, 1, nc0);
        M.cells.set_adjacent(nc, 2, c);
        M.cells.set_adjacent(nc, 3, new_c);
        M.cells.set_adjacent(new_c, 0, nc_nc2);
        M.cells.set_adjacent(new_c, 1, nc1);
        M.cells.set_adjacent(new_c, 2, nc);
        M.cells.set_adjacent(new_c, 3, c);
        if (nc0 != GEO::NO_CELL) {
            assert(M.cells.find_tet_facet(nc0, v, v1, v2) != GEO::NO_INDEX);
            M.cells.set_adjacent(nc0, M.cells.find_tet_facet(nc0, v, v1, v2), nc);
        }
        if (nc1 != GEO::NO_CELL) {
            assert(M.cells.find_tet_facet(nc1, v, v2, v0) != GEO::NO_INDEX);
            M.cells.set_adjacent(nc1, M.cells.find_tet_facet(nc1, v, v2, v0), new_c);
        }
        if (nc2 != GEO::NO_CELL) {
            assert(M.cells.find_tet_facet(nc2, v, v0, v1) != GEO::NO_INDEX);
            M.cells.set_adjacent(nc2, M.cells.find_tet_facet(nc2, v, v0, v1), c);
        }
        if (nc_nc0 != GEO::NO_CELL) {
            assert(M.cells.find_tet_facet(nc_nc0, nv, nv1, nv2) != GEO::NO_INDEX);
            M.cells.set_adjacent(nc_nc0, M.cells.find_tet_facet(nc_nc0, nv, nv1, nv2), nc);
        }
        if (nc_nc1 != GEO::NO_CELL) {
            assert(M.cells.find_tet_facet(nc_nc1, nv, nv2, nv0) != GEO::NO_INDEX);
            M.cells.set_adjacent(nc_nc1, M.cells.find_tet_facet(nc_nc1, nv, nv2, nv0), c);
        }
        if (nc_nc2 != GEO::NO_CELL) {
            assert(M.cells.find_tet_facet(nc_nc2, nv, nv0, nv1) != GEO::NO_INDEX);
            M.cells.set_adjacent(nc_nc2, M.cells.find_tet_facet(nc_nc2, nv, nv0, nv1), new_c);
        }
    }
}
