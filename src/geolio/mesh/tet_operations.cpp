//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/6/25.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "tet_operations.h"
#include <array>
#include <cassert>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
#include "mesh_operations.h"
#include "geolio/common/array_hash.h"

namespace geolio
{
    void tet_split(
        GEO::Mesh& mesh,
        const GEO::index_t c,
        const GEO::index_t new_v,
        const GEO::index_t new_c0,
        const GEO::index_t new_c1,
        const GEO::index_t new_c2,
        const bool update_attributes
        ) {
        assert(c < mesh.cells.nb());
        assert(new_v < mesh.vertices.nb());
        assert(new_c0 < mesh.cells.nb());
        assert(new_c1 < mesh.cells.nb());
        assert(new_c2 < mesh.cells.nb());

        const GEO::index_t v0 = mesh.cells.vertex(c, 0);
        const GEO::index_t v1 = mesh.cells.vertex(c, 1);
        const GEO::index_t v2 = mesh.cells.vertex(c, 2);
        const GEO::index_t v3 = mesh.cells.vertex(c, 3);
        const GEO::index_t nc0 = mesh.cells.adjacent(c, 0);
        const GEO::index_t nc1 = mesh.cells.adjacent(c, 1);
        const GEO::index_t nc2 = mesh.cells.adjacent(c, 2);
        // const GEO::index_t nc3 = M.cells.adjacent(c, 3);

        /* Create new vertex */
        mesh.vertices.point(new_v) = 0.25 * (
            mesh.cells.point(c, 0) + mesh.cells.point(c, 1) + mesh.cells.point(c, 2) + mesh.cells.point(c, 3));

        /* Set vertices */
        mesh.cells.set_vertex(new_c0, 0, new_v);
        mesh.cells.set_vertex(new_c0, 1, v1);
        mesh.cells.set_vertex(new_c0, 2, v2);
        mesh.cells.set_vertex(new_c0, 3, v3);
        mesh.cells.set_vertex(new_c1, 0, v0);
        mesh.cells.set_vertex(new_c1, 1, new_v);
        mesh.cells.set_vertex(new_c1, 2, v2);
        mesh.cells.set_vertex(new_c1, 3, v3);
        mesh.cells.set_vertex(new_c2, 0, v0);
        mesh.cells.set_vertex(new_c2, 1, v1);
        mesh.cells.set_vertex(new_c2, 2, new_v);
        mesh.cells.set_vertex(new_c2, 3, v3);
        mesh.cells.set_vertex(c, 3, new_v);

        /* Set adjacency */
        mesh.cells.set_adjacent(new_c0, 0, nc0);
        mesh.cells.set_adjacent(new_c0, 1, new_c1);
        mesh.cells.set_adjacent(new_c0, 2, new_c2);
        mesh.cells.set_adjacent(new_c0, 3, c);
        mesh.cells.set_adjacent(new_c1, 0, new_c0);
        mesh.cells.set_adjacent(new_c1, 1, nc1);
        mesh.cells.set_adjacent(new_c1, 2, new_c2);
        mesh.cells.set_adjacent(new_c1, 3, c);
        mesh.cells.set_adjacent(new_c2, 0, new_c0);
        mesh.cells.set_adjacent(new_c2, 1, new_c1);
        mesh.cells.set_adjacent(new_c2, 2, nc2);
        mesh.cells.set_adjacent(new_c2, 3, c);
        mesh.cells.set_adjacent(c, 0, new_c0);
        mesh.cells.set_adjacent(c, 1, new_c1);
        mesh.cells.set_adjacent(c, 2, new_c2);
        if (nc0 != GEO::NO_CELL) {
            assert(mesh.cells.find_tet_facet(nc0, v1, v2, v3) != GEO::NO_INDEX);
            mesh.cells.set_adjacent(nc0, mesh.cells.find_tet_facet(nc0, v1, v2, v3), new_c0);
        }
        if (nc1 != GEO::NO_CELL) {
            assert(mesh.cells.find_tet_facet(nc1, v0, v3, v2) != GEO::NO_INDEX);
            mesh.cells.set_adjacent(nc1, mesh.cells.find_tet_facet(nc1, v0, v3, v2), new_c1);
        }
        if (nc2 != GEO::NO_CELL) {
            assert(mesh.cells.find_tet_facet(nc2, v0, v1, v3) != GEO::NO_INDEX);
            mesh.cells.set_adjacent(nc2, mesh.cells.find_tet_facet(nc2, v0, v1, v3), new_c2);
        }

        if (update_attributes) {
            /* Cells */
            mesh.cells.attributes().copy_item(new_c0, c);
            mesh.cells.attributes().copy_item(new_c1, c);
            mesh.cells.attributes().copy_item(new_c2, c);

            /* Cell corners */
            mesh.cell_corners.attributes().zero_item(mesh.cells.corner(new_c0, 0));
            mesh.cell_corners.attributes().copy_item(mesh.cells.corner(new_c0, 1), mesh.cells.corner(c, 1));
            mesh.cell_corners.attributes().copy_item(mesh.cells.corner(new_c0, 2), mesh.cells.corner(c, 2));
            mesh.cell_corners.attributes().copy_item(mesh.cells.corner(new_c0, 3), mesh.cells.corner(c, 3));
            mesh.cell_corners.attributes().copy_item(mesh.cells.corner(new_c1, 0), mesh.cells.corner(c, 0));
            mesh.cell_corners.attributes().zero_item(mesh.cells.corner(new_c1, 1));
            mesh.cell_corners.attributes().copy_item(mesh.cells.corner(new_c1, 2), mesh.cells.corner(c, 2));
            mesh.cell_corners.attributes().copy_item(mesh.cells.corner(new_c1, 3), mesh.cells.corner(c, 3));
            mesh.cell_corners.attributes().copy_item(mesh.cells.corner(new_c2, 0), mesh.cells.corner(c, 0));
            mesh.cell_corners.attributes().copy_item(mesh.cells.corner(new_c2, 1), mesh.cells.corner(c, 1));
            mesh.cell_corners.attributes().zero_item(mesh.cells.corner(new_c2, 2));
            mesh.cell_corners.attributes().copy_item(mesh.cells.corner(new_c2, 3), mesh.cells.corner(c, 3));
            mesh.cell_corners.attributes().zero_item(mesh.cells.corner(c, 3));

            /* Cell facets */
            mesh.cell_facets.attributes().copy_item(mesh.cells.facet(new_c0, 0), mesh.cells.facet(c, 0));
            mesh.cell_facets.attributes().zero_item(mesh.cells.facet(new_c0, 1));
            mesh.cell_facets.attributes().zero_item(mesh.cells.facet(new_c0, 2));
            mesh.cell_facets.attributes().zero_item(mesh.cells.facet(new_c0, 3));
            mesh.cell_facets.attributes().zero_item(mesh.cells.facet(new_c1, 0));
            mesh.cell_facets.attributes().copy_item(mesh.cells.facet(new_c1, 1), mesh.cells.facet(c, 1));
            mesh.cell_facets.attributes().zero_item(mesh.cells.facet(new_c1, 2));
            mesh.cell_facets.attributes().zero_item(mesh.cells.facet(new_c1, 3));
            mesh.cell_facets.attributes().zero_item(mesh.cells.facet(new_c2, 0));
            mesh.cell_facets.attributes().zero_item(mesh.cells.facet(new_c2, 1));
            mesh.cell_facets.attributes().copy_item(mesh.cells.facet(new_c2, 2), mesh.cells.facet(c, 2));
            mesh.cell_facets.attributes().zero_item(mesh.cells.facet(new_c2, 3));
            mesh.cell_facets.attributes().zero_item(mesh.cells.facet(c, 0));
            mesh.cell_facets.attributes().zero_item(mesh.cells.facet(c, 1));
            mesh.cell_facets.attributes().zero_item(mesh.cells.facet(c, 2));
        }
    }

    void tet_facet_split(
        GEO::Mesh& mesh,
        const GEO::index_t c,
        const GEO::index_t lf,
        const GEO::index_t new_v,
        const GEO::index_t new_c0,
        const GEO::index_t new_c1,
        const GEO::index_t new_c2,
        const GEO::index_t new_c3,
        const bool update_attributes
        ) {
        assert(c < mesh.cells.nb());
        assert(lf < 4);
        assert(new_v < mesh.vertices.nb());
        assert(new_c0 < mesh.cells.nb());
        assert(new_c1 < mesh.cells.nb());

        /* Set new vertex */
        mesh.vertices.point(new_v) = (mesh.vertices.point(mesh.cells.facet_vertex(c, lf, 0)) +
                                    mesh.vertices.point(mesh.cells.facet_vertex(c, lf, 1)) +
                                    mesh.vertices.point(mesh.cells.facet_vertex(c, lf, 2))) / 3;


        const GEO::index_t v0 = mesh.cells.vertex(c, TET_LF_INCIDENT_LV[lf][0]);
        const GEO::index_t v1 = mesh.cells.vertex(c, TET_LF_INCIDENT_LV[lf][1]);
        const GEO::index_t v2 = mesh.cells.vertex(c, TET_LF_INCIDENT_LV[lf][2]);
        {
            const GEO::index_t lv0 = TET_LF_INCIDENT_LV[lf][0];
            const GEO::index_t lv1 = TET_LF_INCIDENT_LV[lf][1];
            const GEO::index_t lv2 = TET_LF_INCIDENT_LV[lf][2];
            const GEO::index_t lv3 = (0^1^2^3)^lv0^lv1^lv2;
            assert(lv3 < 4 && lv3 != lv0 && lv3 != lv1 && lv3 != lv2);

            const GEO::index_t v3 = mesh.cells.vertex(c, lv3);

            const GEO::index_t nc0 = mesh.cells.adjacent(c, lv0);
            const GEO::index_t nc1 = mesh.cells.adjacent(c, lv1);

            /*
             *            lv1 (v1)
             *            /  |   \
             *          /    |     \
             *        /    new_v     \
             *      /        /\        \
             *    /   c   /      \ new_c0\
             *  /      /   new_c1   \      \
             * lv0 (v0) ------------- lv2 (v2)
             */

            /* Set cells vertices */
            mesh.cells.set_vertex(c, lv2, new_v);
            mesh.cells.set_vertex(new_c0, lv0, new_v);
            mesh.cells.set_vertex(new_c0, lv1, v1);
            mesh.cells.set_vertex(new_c0, lv2, v2);
            mesh.cells.set_vertex(new_c0, lv3, v3);
            mesh.cells.set_vertex(new_c1, lv0, v0);
            mesh.cells.set_vertex(new_c1, lv1, new_v);
            mesh.cells.set_vertex(new_c1, lv2, v2);
            mesh.cells.set_vertex(new_c1, lv3, v3);

            /* Set cells adjacent */
            mesh.cells.set_adjacent(c, lv0, new_c0);
            mesh.cells.set_adjacent(c, lv1, new_c1);
            mesh.cells.set_adjacent(new_c0, lv0, nc0);
            mesh.cells.set_adjacent(new_c0, lv1, new_c1);
            mesh.cells.set_adjacent(new_c0, lv2, c);
            mesh.cells.set_adjacent(new_c0, lv3, GEO::NO_CELL); // border facet, unless the split facet has an opposite cell
            mesh.cells.set_adjacent(new_c1, lv0, new_c0);
            mesh.cells.set_adjacent(new_c1, lv1, nc1);
            mesh.cells.set_adjacent(new_c1, lv2, c);
            mesh.cells.set_adjacent(new_c1, lv3, GEO::NO_CELL);
            if (nc0 != GEO::NO_CELL) {
                const GEO::index_t nlf = mesh.cells.find_tet_facet(
                    nc0,
                    mesh.cells.facet_vertex(new_c0, lv0, 2),
                    mesh.cells.facet_vertex(new_c0, lv0, 1),
                    mesh.cells.facet_vertex(new_c0, lv0, 0));
                assert(nlf != GEO::NO_INDEX);
                mesh.cells.set_adjacent(nc0, nlf, new_c0);
            }
            if (nc1 != GEO::NO_CELL) {
                const GEO::index_t nlf = mesh.cells.find_tet_facet(
                    nc1,
                    mesh.cells.facet_vertex(new_c1, lv1, 2),
                    mesh.cells.facet_vertex(new_c1, lv1, 1),
                    mesh.cells.facet_vertex(new_c1, lv1, 0));
                assert(nlf != GEO::NO_INDEX);
                mesh.cells.set_adjacent(nc1, nlf, new_c1);
            }

            if (update_attributes) {
                /* Cells */
                mesh.cells.attributes().copy_item(new_c0, c);
                mesh.cells.attributes().copy_item(new_c1, c);

                /* Cell corners */
                mesh.cell_corners.attributes().zero_item(mesh.cells.corner(new_c0, lv0));
                mesh.cell_corners.attributes().copy_item(mesh.cells.corner(new_c0, lv1), mesh.cells.corner(c, lv1));
                mesh.cell_corners.attributes().copy_item(mesh.cells.corner(new_c0, lv2), mesh.cells.corner(c, lv2));
                mesh.cell_corners.attributes().copy_item(mesh.cells.corner(new_c0, lv3), mesh.cells.corner(c, lv3));
                mesh.cell_corners.attributes().copy_item(mesh.cells.corner(new_c1, lv0), mesh.cells.corner(c, lv0));
                mesh.cell_corners.attributes().zero_item(mesh.cells.corner(new_c1, lv1));
                mesh.cell_corners.attributes().copy_item(mesh.cells.corner(new_c1, lv2), mesh.cells.corner(c, lv2));
                mesh.cell_corners.attributes().copy_item(mesh.cells.corner(new_c1, lv3), mesh.cells.corner(c, lv3));
                mesh.cell_corners.attributes().zero_item(mesh.cells.corner(c, lv2));

                /* Cell facets */
                mesh.cell_facets.attributes().copy_item(mesh.cells.facet(new_c0, lv0), mesh.cells.facet(c, lv0));
                mesh.cell_facets.attributes().zero_item(mesh.cells.facet(new_c0, lv1));
                mesh.cell_facets.attributes().zero_item(mesh.cells.facet(new_c0, lv2));
                mesh.cell_facets.attributes().copy_item(mesh.cells.facet(new_c0, lv3), mesh.cells.facet(c, lv3));
                mesh.cell_facets.attributes().zero_item(mesh.cells.facet(new_c1, lv0));
                mesh.cell_facets.attributes().copy_item(mesh.cells.facet(new_c1, lv1), mesh.cells.facet(c, lv1));
                mesh.cell_facets.attributes().zero_item(mesh.cells.facet(new_c1, lv2));
                mesh.cell_facets.attributes().copy_item(mesh.cells.facet(new_c1, lv3), mesh.cells.facet(c, lv3));
                mesh.cell_facets.attributes().zero_item(mesh.cells.facet(c, lv0));
                mesh.cell_facets.attributes().zero_item(mesh.cells.facet(c, lv1));
            }
        }

        if (const auto& ac = mesh.cells.adjacent(c, lf);
            ac != GEO::NO_CELL
            ) {
            assert(new_c2 < mesh.cells.nb());
            assert(new_c3 < mesh.cells.nb());

            const GEO::index_t lv0 = mesh.cells.find_tet_vertex(ac, v0);
            const GEO::index_t lv1 = mesh.cells.find_tet_vertex(ac, v2);
            const GEO::index_t lv2 = mesh.cells.find_tet_vertex(ac, v1);
            const GEO::index_t lv3 = (0^1^2^3)^lv0^lv1^lv2;
            assert(lv3 < 4 && lv3 != lv0 && lv3 != lv1 && lv3 != lv2);

            const GEO::index_t v3 = mesh.cells.vertex(ac, lv3);

            const GEO::index_t nc0 = mesh.cells.adjacent(ac, lv0);
            const GEO::index_t nc2 = mesh.cells.adjacent(ac, lv2);

            /*
             *            lv2 (v1)
             *            /  |   \
             *          /    |     \
             *        /    new_v     \
             *      /        /\        \
             *    /  ac   /      \ new_c2\
             *  /      /   new_c3   \      \
             * lv0 (v0) ------------- lv1 (v2)
             */

            /* Set cells vertices */
            mesh.cells.set_vertex(ac, lv1, new_v);
            mesh.cells.set_vertex(new_c2, lv0, new_v);
            mesh.cells.set_vertex(new_c2, lv1, v2);
            mesh.cells.set_vertex(new_c2, lv2, v1);
            mesh.cells.set_vertex(new_c2, lv3, v3);
            mesh.cells.set_vertex(new_c3, lv0, v0);
            mesh.cells.set_vertex(new_c3, lv1, v2);
            mesh.cells.set_vertex(new_c3, lv2, new_v);
            mesh.cells.set_vertex(new_c3, lv3, v3);

            /* Set cells adjacent */
            mesh.cells.set_adjacent(ac, lv0, new_c2);
            mesh.cells.set_adjacent(ac, lv2, new_c3);
            mesh.cells.set_adjacent(new_c2, lv0, nc0);
            mesh.cells.set_adjacent(new_c2, lv1, ac);
            mesh.cells.set_adjacent(new_c2, lv2, new_c3);
            mesh.cells.set_adjacent(new_c2, lv3, new_c0);
            mesh.cells.set_adjacent(new_c3, lv0, new_c2);
            mesh.cells.set_adjacent(new_c3, lv1, ac);
            mesh.cells.set_adjacent(new_c3, lv2, nc2);
            mesh.cells.set_adjacent(new_c3, lv3, new_c1);
            /* The split facet is interior: the new cells of both sides face each other. */
            mesh.cells.set_adjacent(new_c0, lf, new_c2);
            mesh.cells.set_adjacent(new_c1, lf, new_c3);
            if (nc0 != GEO::NO_CELL) {
                const GEO::index_t nlf = mesh.cells.find_tet_facet(
                    nc0,
                    mesh.cells.facet_vertex(new_c2, lv0, 2),
                    mesh.cells.facet_vertex(new_c2, lv0, 1),
                    mesh.cells.facet_vertex(new_c2, lv0, 0));
                assert(nlf != GEO::NO_INDEX);
                mesh.cells.set_adjacent(nc0, nlf, new_c2);
            }
            if (nc2 != GEO::NO_CELL) {
                const GEO::index_t nlf = mesh.cells.find_tet_facet(
                    nc2,
                    mesh.cells.facet_vertex(new_c3, lv2, 2),
                    mesh.cells.facet_vertex(new_c3, lv2, 1),
                    mesh.cells.facet_vertex(new_c3, lv2, 0));
                assert(nlf != GEO::NO_INDEX);
                mesh.cells.set_adjacent(nc2, nlf, new_c3);
            }

            if (update_attributes) {
                /* Cells */
                mesh.cells.attributes().copy_item(new_c2, ac);
                mesh.cells.attributes().copy_item(new_c3, ac);

                /* Cell corners */
                mesh.cell_corners.attributes().zero_item(mesh.cells.corner(new_c2, lv0));
                mesh.cell_corners.attributes().copy_item(mesh.cells.corner(new_c2, lv1), mesh.cells.corner(ac, lv1));
                mesh.cell_corners.attributes().copy_item(mesh.cells.corner(new_c2, lv2), mesh.cells.corner(ac, lv2));
                mesh.cell_corners.attributes().copy_item(mesh.cells.corner(new_c2, lv3), mesh.cells.corner(ac, lv3));
                mesh.cell_corners.attributes().copy_item(mesh.cells.corner(new_c3, lv0), mesh.cells.corner(ac, lv0));
                mesh.cell_corners.attributes().copy_item(mesh.cells.corner(new_c3, lv1), mesh.cells.corner(ac, lv1));
                mesh.cell_corners.attributes().zero_item(mesh.cells.corner(new_c3, lv2));
                mesh.cell_corners.attributes().copy_item(mesh.cells.corner(new_c3, lv3), mesh.cells.corner(ac, lv3));
                mesh.cell_corners.attributes().zero_item(mesh.cells.corner(ac, lv1));

                /* Cell facets */
                mesh.cell_facets.attributes().copy_item(mesh.cells.facet(new_c2, lv0), mesh.cells.facet(ac, lv0));
                mesh.cell_facets.attributes().zero_item(mesh.cells.facet(new_c2, lv1));
                mesh.cell_facets.attributes().zero_item(mesh.cells.facet(new_c2, lv2));
                mesh.cell_facets.attributes().copy_item(mesh.cells.facet(new_c2, lv3), mesh.cells.facet(ac, lv3));
                mesh.cell_facets.attributes().zero_item(mesh.cells.facet(new_c3, lv0));
                mesh.cell_facets.attributes().zero_item(mesh.cells.facet(new_c3, lv1));
                mesh.cell_facets.attributes().copy_item(mesh.cells.facet(new_c3, lv2), mesh.cells.facet(ac, lv2));
                mesh.cell_facets.attributes().copy_item(mesh.cells.facet(new_c3, lv3), mesh.cells.facet(ac, lv3));
                mesh.cell_facets.attributes().zero_item(mesh.cells.facet(ac, lv0));
                mesh.cell_facets.attributes().zero_item(mesh.cells.facet(ac, lv2));
            }
        }
    }

    void tet_edge_split(
        GEO::Mesh& mesh,
        const std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>>& ordered_c_le_lf,
        const GEO::index_t new_v,
        const std::vector<GEO::index_t>& new_cs,
        const bool update_attributes
        ) {
        assert(new_v < mesh.vertices.nb());
        assert(new_cs.size() == ordered_c_le_lf.size());

        /* Find all adjacent cells */
        const GEO::index_t INCIDENT_CELLS_NB = ordered_c_le_lf.size();

        const auto start_c = get<0>(ordered_c_le_lf[0]);
        const auto start_le = get<1>(ordered_c_le_lf[0]);
        const GEO::index_t ev0 = mesh.cells.edge_vertex(start_c, start_le, 0);
        const GEO::index_t ev1 = mesh.cells.edge_vertex(start_c, start_le, 1);

        /* Set new vertex */
        mesh.vertices.point(new_v) = 0.5 * (mesh.vertices.point(ev0) + mesh.vertices.point(ev1));

        for (GEO::index_t i = 0; i < INCIDENT_CELLS_NB; ++i) {
            const auto& [c, _, lf0] = ordered_c_le_lf[i];
            const auto& new_c = new_cs[i];

            const GEO::index_t lv0 = mesh.cells.find_tet_vertex(c, ev0);
            const GEO::index_t lv1 = mesh.cells.find_tet_vertex(c, ev1);
            assert(lv0 != GEO::NO_INDEX);
            assert(lv1 != GEO::NO_INDEX);
            const GEO::index_t lv2 = TET_LF_INCIDENT_LV[lf0][0]^TET_LF_INCIDENT_LV[lf0][1]^TET_LF_INCIDENT_LV[lf0][2]^lv0^lv1;
            assert(lv2 < 4 && lv2 != lv0 && lv2 != lv1);
            const GEO::index_t lv3 = 0^1^2^3^lv0^lv1^lv2;
            assert(lv3 < 4 && lv3 != lv0 && lv3 != lv1 && lv3 != lv2);

            const GEO::index_t v2 = mesh.cells.vertex(c, lv2);
            const GEO::index_t v3 = mesh.cells.vertex(c, lv3);

            // const GEO::index_t nc0 = M.cells.adjacent(c, lv0);
            const GEO::index_t nc1 = mesh.cells.adjacent(c, lv1);

            /* Set cell vertices */
            mesh.cells.set_vertex(c, lv0, new_v);
            mesh.cells.set_vertex(new_c, lv0, ev0);
            mesh.cells.set_vertex(new_c, lv1, new_v);
            mesh.cells.set_vertex(new_c, lv2, v2);
            mesh.cells.set_vertex(new_c, lv3, v3);

            /* Set cell adjacent */
            mesh.cells.set_adjacent(c, lv1, new_c);
            mesh.cells.set_adjacent(new_c, lv0, c);
            mesh.cells.set_adjacent(new_c, lv1, nc1);
            mesh.cells.set_adjacent(new_c, lv2,
                mesh.cells.adjacent(c, lv2) == GEO::NO_CELL ?
                    GEO::NO_CELL :
                    new_cs[(i+INCIDENT_CELLS_NB-1)%INCIDENT_CELLS_NB]);
            mesh.cells.set_adjacent(new_c, lv3,
                mesh.cells.adjacent(c, lv3) == GEO::NO_CELL ?
                    GEO::NO_CELL :
                    new_cs[(i+1)%INCIDENT_CELLS_NB]);

            if (nc1 != GEO::NO_CELL) {
                const GEO::index_t nlf = mesh.cells.find_tet_facet(
                    nc1,
                    mesh.cells.facet_vertex(new_c, lv1, 2),
                    mesh.cells.facet_vertex(new_c, lv1, 1),
                    mesh.cells.facet_vertex(new_c, lv1, 0));
                assert(nlf != GEO::NO_INDEX);
                mesh.cells.set_adjacent(nc1, nlf, new_c);
            }

            if (update_attributes) {
                /* Cells */
                mesh.cells.attributes().copy_item(new_c, c);

                /* Cell corners */
                mesh.cell_corners.attributes().copy_item(mesh.cells.corner(new_c, lv0), mesh.cells.corner(c, lv0));
                mesh.cell_corners.attributes().zero_item(mesh.cells.corner(new_c, lv1));
                mesh.cell_corners.attributes().copy_item(mesh.cells.corner(new_c, lv2), mesh.cells.corner(c, lv2));
                mesh.cell_corners.attributes().copy_item(mesh.cells.corner(new_c, lv3), mesh.cells.corner(c, lv3));
                mesh.cell_corners.attributes().zero_item(mesh.cells.corner(c, lv0));

                /* Cell facets */
                mesh.cell_facets.attributes().zero_item(mesh.cells.facet(new_c, lv0));
                mesh.cell_facets.attributes().copy_item(mesh.cells.facet(new_c, lv1), mesh.cells.facet(c, lv1));
                mesh.cell_facets.attributes().copy_item(mesh.cells.facet(new_c, lv2), mesh.cells.facet(c, lv2));
                mesh.cell_facets.attributes().copy_item(mesh.cells.facet(new_c, lv3), mesh.cells.facet(c, lv3));
                mesh.cell_facets.attributes().zero_item(mesh.cells.facet(c, lv1));
            }
        }
    }

    bool is_tet_edge_collapse_valid(
        const GEO::Mesh& mesh,
        const GEO::index_t c,
        const GEO::index_t le
        ) {
        assert(c < mesh.cells.nb());
        assert(mesh.cells.type(c) == GEO::MeshCellType::MESH_TET);
        assert(le < 6);

        const auto& v0 = mesh.cells.edge_vertex(c, le, 0);
        const auto& v1 = mesh.cells.edge_vertex(c, le, 1);

        /* Find all incident cells */
        std::vector<std::pair<GEO::index_t, GEO::index_t>> v0_c_and_lv;
        const bool v0_on_boundary = get_vertex_incident_cells(mesh, c, TET_LE_INCIDENT_LV[le][0], v0_c_and_lv);

        std::vector<std::pair<GEO::index_t, GEO::index_t>> v1_c_and_lv;
        const bool v1_on_boundary = get_vertex_incident_cells(mesh, c, TET_LE_INCIDENT_LV[le][1], v1_c_and_lv);

        std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>> ordered_c_le_lf;
        const bool edge_on_boundary = get_edge_incident_cells(mesh, c, le, ordered_c_le_lf);

        if (v0_on_boundary && v1_on_boundary && !edge_on_boundary) // will create a new non-manifold vertex
            return false;

        /* Build adjacent cells set */
        std::unordered_set<GEO::index_t> adjacent_cells;
        for (const auto& [c, _, __] : ordered_c_le_lf)
            adjacent_cells.insert(c);

        /* The vertices opposite the collapsed edge in the cells that contain it. */
        std::unordered_set<GEO::index_t> edge_ring_vertices;
        for (const auto& [nc, _, __] : ordered_c_le_lf) {
            for (GEO::index_t lv = 0; lv < 4; ++lv) {
                const auto nv = mesh.cells.vertex(nc, lv);
                if (nv != v0 && nv != v1)
                    edge_ring_vertices.insert(nv);
            }
        }

        /* The neighbours of a vertex are the three other vertices of every cell incident to it. */
        const auto ring_neighbours = [&mesh](const std::vector<std::pair<GEO::index_t, GEO::index_t>>& ring) {
            std::unordered_set<GEO::index_t> neighbours;
            for (const auto& [nc, nlv] : ring) {
                neighbours.insert(mesh.cells.vertex(nc, (nlv+1)%4));
                neighbours.insert(mesh.cells.vertex(nc, (nlv+2)%4));
                neighbours.insert(mesh.cells.vertex(nc, (nlv+3)%4));
            }
            return neighbours;
        };

        /* The collapse merges the star of v1 into the star of v0, so the edge (v1, u) of every neighbour
         * u of v1 becomes the edge (v0, u). When u is also a neighbour of v0, that edge is already there
         * and the mesh would end up holding it twice, which breaks its 3-manifoldness: v0 and v1 may only
         * have in common the vertices opposite the collapsed edge (link condition). */
        const auto v1_neighbours = ring_neighbours(v1_c_and_lv);
        for (const auto& nv : ring_neighbours(v0_c_and_lv)) {
            if (!v1_neighbours.contains(nv))
                continue;
            if (!edge_ring_vertices.contains(nv))
                return false; // v0 and v1 share a neighbour: the collapse would duplicate the edge to it.
        }

        /* After collapse, no identical tetrahedra can exist */
        std::unordered_set<std::array<GEO::index_t, 3>, ArrayHash<GEO::index_t, 3>> other_vertices_array;
        for (const auto& [nc, nlv] : v0_c_and_lv) {
            const auto& nv1 = mesh.cells.vertex(nc, (nlv+1)%4);
            const auto& nv2 = mesh.cells.vertex(nc, (nlv+2)%4);
            const auto& nv3 = mesh.cells.vertex(nc, (nlv+3)%4);
            assert(nv1 != v0);
            assert(nv2 != v0);
            assert(nv3 != v0);
            if (nv1 == nv2 || nv2 == nv3 || nv3 == nv1) // Exist degenerate adjacent facet.
                return false;

            if (nv1 == v1 || nv2 == v1 || nv3 == v1) {
                if (!adjacent_cells.contains(nc)) // Non-manifold edge.
                    return false;
            }

            std::array<GEO::index_t, 3> cvs = {nv1, nv2, nv3};
            std::ranges::sort(cvs);
            if (!other_vertices_array.insert(cvs).second) // Identical triangles in an adjacent facet group.
                return false;
        }
        for (const auto& [nc, nlv] : v1_c_and_lv) {
            const auto& nv1 = mesh.cells.vertex(nc, (nlv+1)%4);
            const auto& nv2 = mesh.cells.vertex(nc, (nlv+2)%4);
            const auto& nv3 = mesh.cells.vertex(nc, (nlv+3)%4);
            assert(nv1 != v1);
            assert(nv2 != v1);
            assert(nv3 != v1);
            if (nv1 == nv2 || nv2 == nv3 || nv3 == nv1) // Exist degenerate adjacent facet.
                return false;

            if (nv1 == v0 || nv2 == v0 || nv3 == v0) {
                if (!adjacent_cells.contains(nc)) // Non-manifold edge.
                    return false;
            }

            std::array<GEO::index_t, 3> cvs = {nv1, nv2, nv3};
            std::ranges::sort(cvs);
            if (!other_vertices_array.insert(cvs).second) // Identical triangles in an adjacent facet group.
                return false;
        }

        return true;
    }

    void tet_edge_collapse(
        GEO::Mesh& mesh,
        const GEO::index_t _c,
        const GEO::index_t _le,
        GEO::index_t& disuse_v,
        std::vector<GEO::index_t>& disuse_cs
        ) {
        assert(_c < mesh.cells.nb());
        assert(mesh.cells.type(_c) == GEO::MeshCellType::MESH_TET);
        assert(_le < 6);

        const auto& ev0 = mesh.cells.edge_vertex(_c, _le, 0);
        const auto& ev1 = mesh.cells.edge_vertex(_c, _le, 1);

        /* Move vertex */
        auto& ep0 = mesh.vertices.point(ev0);
        const auto& ep1 = mesh.vertices.point(ev1);
        ep0 = 0.5 * (ep0 + ep1);
        disuse_v = ev1;

        /* Find all adjacent tets */
        std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>> ordered_c_le_lf;
        get_edge_incident_cells(mesh, _c, _le, ordered_c_le_lf);

        std::vector<std::pair<GEO::index_t, GEO::index_t>> ev1_incident_c_and_lv;
        get_vertex_incident_cells(mesh, _c, TET_LE_INCIDENT_LV[_le][1], ev1_incident_c_and_lv);

        /* Collapse */
        for (const auto& c: ordered_c_le_lf | std::views::keys) {
            const auto lf0 = mesh.cells.find_tet_vertex(c, ev0);
            const auto lf1 = mesh.cells.find_tet_vertex(c, ev1);
            const auto nc0 = mesh.cells.adjacent(c, lf0);
            const auto nc1 = mesh.cells.adjacent(c, lf1);

            if (nc0 != GEO::NO_CELL) {
                /* Set adjacent */
                const auto nlf = mesh.cells.find_tet_facet(
                    nc0,
                    mesh.cells.facet_vertex(c, lf0, 2),
                    mesh.cells.facet_vertex(c, lf0, 1),
                    mesh.cells.facet_vertex(c, lf0, 0));
                assert(nlf != GEO::NO_INDEX);
                mesh.cells.set_adjacent(nc0, nlf, nc1);
            }
            if (nc1 != GEO::NO_CELL) {
                /* Set adjacent */
                const auto nlf = mesh.cells.find_tet_facet(
                    nc1,
                    mesh.cells.facet_vertex(c, lf1, 2),
                    mesh.cells.facet_vertex(c, lf1, 1),
                    mesh.cells.facet_vertex(c, lf1, 0));
                assert(nlf != GEO::NO_INDEX);
                mesh.cells.set_adjacent(nc1, nlf, nc0);
            }

            disuse_cs.push_back(c);
        }

        /* Update vertex of other cells */
        for (const auto& [c, lv] :ev1_incident_c_and_lv)
            mesh.cells.set_vertex(c, lv, ev0);
    }

    bool is_tet_edge_swap_2_3_valid(
        const GEO::Mesh& mesh,
        const GEO::index_t c,
        const GEO::index_t lf
        ) {
        assert(c < mesh.cells.nb());
        assert(mesh.cells.type(c) == GEO::MeshCellType::MESH_TET);
        assert(lf < 4);

        const GEO::index_t nc = mesh.cells.adjacent(c, lf);
        if (nc == GEO::NO_CELL) // Cannot swap border facet
            return false;

        const GEO::index_t v = mesh.cells.vertex(c, lf);
        const GEO::index_t v0 = mesh.cells.vertex(c, TET_LF_INCIDENT_LV[lf][0]);
        const GEO::index_t v1 = mesh.cells.vertex(c, TET_LF_INCIDENT_LV[lf][1]);
        const GEO::index_t v2 = mesh.cells.vertex(c, TET_LF_INCIDENT_LV[lf][2]);
        const GEO::index_t nlf = mesh.cells.find_tet_facet(nc, v2, v1, v0);
        const GEO::index_t nv = mesh.cells.vertex(nc, nlf);

        /* Find all incident cells */
        std::vector<std::pair<GEO::index_t, GEO::index_t>> v_c_and_lv;
        get_vertex_incident_cells(mesh, c, lf, v_c_and_lv);
        for (const auto& ac: v_c_and_lv | std::views::keys) {
            for (GEO::index_t lv = 0; lv < 4; ++lv) {
                if (mesh.cells.vertex(ac, lv) == nv)
                    return false;
            }
        }

        std::vector<std::pair<GEO::index_t, GEO::index_t>> nv_c_and_lv;
        get_vertex_incident_cells(mesh, nc, nlf, nv_c_and_lv);
        for (const auto& ac: nv_c_and_lv | std::views::keys) {
            for (GEO::index_t lv = 0; lv < 4; ++lv) {
                if (mesh.cells.vertex(ac, lv) == v)
                    return false;
            }
        }

        return true;
    }

    void tet_edge_swap_2_3(
        GEO::Mesh& mesh,
        const GEO::index_t c,
        const GEO::index_t lf,
        const GEO::index_t new_c,
        const bool update_attributes
        ) {
        assert(c < mesh.cells.nb());
        assert(mesh.cells.type(c) == GEO::MeshCellType::MESH_TET);
        assert(lf < 4);
        assert(new_c < mesh.cells.nb());

        const GEO::index_t nc = mesh.cells.adjacent(c, lf);
        assert(nc != GEO::NO_CELL);

        const GEO::index_t v = mesh.cells.vertex(c, lf);
        const GEO::index_t lv0 = TET_LF_INCIDENT_LV[lf][0];
        const GEO::index_t lv1 = TET_LF_INCIDENT_LV[lf][1];
        const GEO::index_t lv2 = TET_LF_INCIDENT_LV[lf][2];
        const GEO::index_t v0 = mesh.cells.vertex(c, lv0);
        const GEO::index_t v1 = mesh.cells.vertex(c, lv1);
        const GEO::index_t v2 = mesh.cells.vertex(c, lv2);
        const GEO::index_t nc0 = mesh.cells.adjacent(c, lv0);
        const GEO::index_t nc1 = mesh.cells.adjacent(c, lv1);
        const GEO::index_t nc2 = mesh.cells.adjacent(c, lv2);

        const GEO::index_t nlf = mesh.cells.find_tet_facet(nc, v2, v1, v0);
        assert(nlf != GEO::NO_INDEX);
        GEO::index_t nlv0{GEO::NO_INDEX}, nlv1{GEO::NO_INDEX}, nlv2{GEO::NO_INDEX};
        for (GEO::index_t i = 0; i < 3; ++i) {
            if (mesh.cells.facet_vertex(nc, nlf, i) == v0) { // cell_vertex(nc, nlv0) == cell_vertex(c, lv0)
                nlv0 = TET_LF_INCIDENT_LV[nlf][i];
                nlv1 = TET_LF_INCIDENT_LV[nlf][(i+1)%3];
                nlv2 = TET_LF_INCIDENT_LV[nlf][(i+2)%3];
                break;
            }
        }
        assert(nlv0 != GEO::NO_INDEX && nlv1 != GEO::NO_INDEX && nlv2 != GEO::NO_INDEX);
        const GEO::index_t nv = mesh.cells.vertex(nc, nlf);
        const GEO::index_t nv0 = mesh.cells.vertex(nc, nlv0);
        assert(nv0 == v0);
        const GEO::index_t nv1 = mesh.cells.vertex(nc, nlv1);
        assert(nv1 == v2);
        const GEO::index_t nv2 = mesh.cells.vertex(nc, nlv2);
        assert(nv2 == v1);
        const GEO::index_t nc_nc0 = mesh.cells.adjacent(nc, nlv0);
        const GEO::index_t nc_nc1 = mesh.cells.adjacent(nc, nlv1);
        const GEO::index_t nc_nc2 = mesh.cells.adjacent(nc, nlv2);

        /* Set cell vertices */
        mesh.cells.set_vertex(c, 0, v);
        mesh.cells.set_vertex(c, 1, nv);
        mesh.cells.set_vertex(c, 2, v1);
        mesh.cells.set_vertex(c, 3, v0);
        mesh.cells.set_vertex(nc, 0, v);
        mesh.cells.set_vertex(nc, 1, nv);
        mesh.cells.set_vertex(nc, 2, v2);
        mesh.cells.set_vertex(nc, 3, v1);
        mesh.cells.set_vertex(new_c, 0, v);
        mesh.cells.set_vertex(new_c, 1, nv);
        mesh.cells.set_vertex(new_c, 2, v0);
        mesh.cells.set_vertex(new_c, 3, v2);

        /* Set adjacency */
        mesh.cells.set_adjacent(c, 0, nc_nc1);
        mesh.cells.set_adjacent(c, 1, nc2);
        mesh.cells.set_adjacent(c, 2, new_c);
        mesh.cells.set_adjacent(c, 3, nc);
        mesh.cells.set_adjacent(nc, 0, nc_nc0);
        mesh.cells.set_adjacent(nc, 1, nc0);
        mesh.cells.set_adjacent(nc, 2, c);
        mesh.cells.set_adjacent(nc, 3, new_c);
        mesh.cells.set_adjacent(new_c, 0, nc_nc2);
        mesh.cells.set_adjacent(new_c, 1, nc1);
        mesh.cells.set_adjacent(new_c, 2, nc);
        mesh.cells.set_adjacent(new_c, 3, c);
        if (nc0 != GEO::NO_CELL) {
            assert(mesh.cells.find_tet_facet(nc0, v, v1, v2) != GEO::NO_INDEX);
            mesh.cells.set_adjacent(nc0, mesh.cells.find_tet_facet(nc0, v, v1, v2), nc);
        }
        if (nc1 != GEO::NO_CELL) {
            assert(mesh.cells.find_tet_facet(nc1, v, v2, v0) != GEO::NO_INDEX);
            mesh.cells.set_adjacent(nc1, mesh.cells.find_tet_facet(nc1, v, v2, v0), new_c);
        }
        if (nc2 != GEO::NO_CELL) {
            assert(mesh.cells.find_tet_facet(nc2, v, v0, v1) != GEO::NO_INDEX);
            mesh.cells.set_adjacent(nc2, mesh.cells.find_tet_facet(nc2, v, v0, v1), c);
        }
        if (nc_nc0 != GEO::NO_CELL) {
            assert(mesh.cells.find_tet_facet(nc_nc0, nv, nv1, nv2) != GEO::NO_INDEX);
            mesh.cells.set_adjacent(nc_nc0, mesh.cells.find_tet_facet(nc_nc0, nv, nv1, nv2), nc);
        }
        if (nc_nc1 != GEO::NO_CELL) {
            assert(mesh.cells.find_tet_facet(nc_nc1, nv, nv2, nv0) != GEO::NO_INDEX);
            mesh.cells.set_adjacent(nc_nc1, mesh.cells.find_tet_facet(nc_nc1, nv, nv2, nv0), c);
        }
        if (nc_nc2 != GEO::NO_CELL) {
            assert(mesh.cells.find_tet_facet(nc_nc2, nv, nv0, nv1) != GEO::NO_INDEX);
            mesh.cells.set_adjacent(nc_nc2, mesh.cells.find_tet_facet(nc_nc2, nv, nv0, nv1), new_c);
        }

        if (update_attributes) {
            /* Cells */
            mesh.cells.attributes().zero_item(c);
            mesh.cells.attributes().zero_item(nc);
            mesh.cells.attributes().zero_item(new_c);

            /* Cell corners */
            mesh.cell_corners.attributes().copy_item(mesh.cells.corner(new_c, 0), mesh.cells.corner(c, lf)); // need to set new_c first
            mesh.cell_corners.attributes().copy_item(mesh.cells.corner(new_c, 1), mesh.cells.corner(nc, nlf)); // need to set new_c first
            mesh.cell_corners.attributes().zero_item(mesh.cells.corner(new_c, 2));
            mesh.cell_corners.attributes().zero_item(mesh.cells.corner(new_c, 3));
            mesh.cell_corners.attributes().copy_item(mesh.cells.corner(c, 0), mesh.cells.corner(new_c, 0));
            mesh.cell_corners.attributes().copy_item(mesh.cells.corner(c, 1), mesh.cells.corner(new_c, 1));
            mesh.cell_corners.attributes().zero_item(mesh.cells.corner(c, 2));
            mesh.cell_corners.attributes().zero_item(mesh.cells.corner(c, 3));
            mesh.cell_corners.attributes().copy_item(mesh.cells.corner(nc, 0), mesh.cells.corner(new_c, 0));
            mesh.cell_corners.attributes().copy_item(mesh.cells.corner(nc, 1), mesh.cells.corner(new_c, 1));
            mesh.cell_corners.attributes().zero_item(mesh.cells.corner(nc, 2));
            mesh.cell_corners.attributes().zero_item(mesh.cells.corner(nc, 3));

            /* Cell facets */
            mesh.cell_facets.attributes().copy_item(mesh.cells.facet(new_c, 0), mesh.cells.facet(nc, nlv2));
            mesh.cell_facets.attributes().copy_item(mesh.cells.facet(new_c, 1), mesh.cells.facet(c, lv1));
            mesh.cell_facets.attributes().zero_item(mesh.cells.facet(new_c, 2));
            mesh.cell_facets.attributes().zero_item(mesh.cells.facet(new_c, 3));
            // `nc` remains M.cells.facet(nc, nlv0) and M.cells.facet(nc, nlv1) are useful, temporarily move to safety index (2, 3)
            GEO::index_t new_nlv0 = 2;
            GEO::index_t new_nlv1 = 3;
            if (nlv1 == 2) { // nv0 -> 3
                new_nlv0 = 3;
                new_nlv1 = 2;
            }
            else if (nlv0 == 3) { // nv1 -> 2
                new_nlv0 = 3;
                new_nlv1 = 2;
            }
            if (nlv0 != new_nlv0)
                mesh.cell_facets.attributes().copy_item(mesh.cells.facet(nc, new_nlv0), mesh.cells.facet(nc, nlv0));
            if (nlv1 != new_nlv1)
                mesh.cell_facets.attributes().copy_item(mesh.cells.facet(nc, new_nlv1), mesh.cells.facet(nc, nlv1));
            // `c` remains M.cells.facet(c, lv0) and M.cells.facet(c, lv2) are useful, temporarily move to safety index (2, 3)
            GEO::index_t new_lv0 = 2;
            GEO::index_t new_lv2 = 3;
            if (lv2 == 2) {
                new_lv0 = 3;
                new_lv2 = 2;
            }
            else if (lv0 == 3) {
                new_lv0 = 3;
                new_lv2 = 2;
            }
            if (lv0 != new_lv0)
                mesh.cell_facets.attributes().copy_item(mesh.cells.facet(c, new_lv0), mesh.cells.facet(c, lv0));
            if (lv2 != new_lv2)
                mesh.cell_facets.attributes().copy_item(mesh.cells.facet(c, new_lv2), mesh.cells.facet(c, lv2));
            mesh.cell_facets.attributes().copy_item(mesh.cells.facet(nc, 0), mesh.cells.facet(nc, new_nlv0));
            mesh.cell_facets.attributes().copy_item(mesh.cells.facet(nc, 1), mesh.cells.facet(c, new_lv0));
            mesh.cell_facets.attributes().copy_item(mesh.cells.facet(c, 0), mesh.cells.facet(nc, new_nlv1));
            mesh.cell_facets.attributes().copy_item(mesh.cells.facet(c, 1), mesh.cells.facet(c, new_lv2));
            mesh.cell_facets.attributes().zero_item(mesh.cells.facet(c, new_lv0));
            mesh.cell_facets.attributes().zero_item(mesh.cells.facet(c, new_lv2));
            mesh.cell_facets.attributes().zero_item(mesh.cells.facet(nc, new_nlv0));
            mesh.cell_facets.attributes().zero_item(mesh.cells.facet(nc, new_nlv1));
        }
    }

    bool tet_edge_swap_3_2(
        GEO::Mesh& mesh,
        const std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>>& ordered_c_le_lf,
        GEO::index_t& disuse_c,
        const bool update_attributes
        ) {
        /* The swap requires the three distinct cells that share an interior edge, in ring order. */
        if (ordered_c_le_lf.size() != 3)
            return false;

        const GEO::index_t c0 = get<0>(ordered_c_le_lf[0]);
        const GEO::index_t c1 = get<0>(ordered_c_le_lf[1]);
        const GEO::index_t c2 = get<0>(ordered_c_le_lf[2]);
        if (c0 == c1 || c1 == c2 || c2 == c0)
            return false;
        for (const auto cc : {c0, c1, c2}) {
            if (cc >= mesh.cells.nb() || mesh.cells.type(cc) != GEO::MeshCellType::MESH_TET)
                return false;
        }
        if (mesh.cells.adjacent(c2, get<2>(ordered_c_le_lf[2])) == GEO::NO_CELL) // edge on the boundary: no ring to flip
            return false;

        const GEO::index_t v0 = mesh.cells.edge_vertex(c0, get<1>(ordered_c_le_lf[0]), 0);
        const GEO::index_t v1 = mesh.cells.edge_vertex(c0, get<1>(ordered_c_le_lf[0]), 1);
        for (const auto cc : {c1, c2}) { // the three cells must share the edge
            if (mesh.cells.find_tet_vertex(cc, v0) == GEO::NO_INDEX ||
                mesh.cells.find_tet_vertex(cc, v1) == GEO::NO_INDEX)
                return false;
        }

        const GEO::index_t v2 = get_tet_facet_another_vertex(mesh, c0, get<2>(ordered_c_le_lf[0]), v0, v1);
        const GEO::index_t v4 = get_tet_facet_another_vertex(mesh, c1, get<2>(ordered_c_le_lf[1]), v0, v1);
        if (v2 == GEO::NO_INDEX || v4 == GEO::NO_INDEX)
            return false;

        const GEO::index_t c0_lv0 = mesh.cells.find_tet_vertex(c0, v0);
        const GEO::index_t c0_lv1 = mesh.cells.find_tet_vertex(c0, v1);
        const GEO::index_t c0_lv2 = mesh.cells.find_tet_vertex(c0, v2);
        if (c0_lv0 == GEO::NO_INDEX || c0_lv1 == GEO::NO_INDEX || c0_lv2 == GEO::NO_INDEX)
            return false;
        const GEO::index_t c0_lv3 = 0^1^2^3^c0_lv0^c0_lv1^c0_lv2;

        const GEO::index_t v3 = mesh.cells.vertex(c0, c0_lv3);

        const GEO::index_t c1_lv0 = mesh.cells.find_tet_vertex(c1, v0);
        const GEO::index_t c1_lv1 = mesh.cells.find_tet_vertex(c1, v1);
        const GEO::index_t c1_lv4 = mesh.cells.find_tet_vertex(c1, v4);
        if (c1_lv0 == GEO::NO_INDEX || c1_lv1 == GEO::NO_INDEX || c1_lv4 == GEO::NO_INDEX)
            return false;
        const GEO::index_t c1_lv2 = 0^1^2^3^c1_lv0^c1_lv1^c1_lv4;

        /* The two resulting cells share the facet (v2, v3, v4): if a cell already holds those three
         * vertices, adding them would duplicate existing tetrahedra and break the manifoldness. */
        std::vector<std::pair<GEO::index_t, GEO::index_t>> v2_incident_c_and_lv;
        get_vertex_incident_cells(mesh, c0, c0_lv2, v2_incident_c_and_lv);
        for (const auto& [nc, nlv] : v2_incident_c_and_lv) {
            if (mesh.cells.find_tet_vertex(nc, v3) != GEO::NO_INDEX &&
                mesh.cells.find_tet_vertex(nc, v4) != GEO::NO_INDEX)
                return false;
        }

        disuse_c = c2;

        // const GEO::index_t nc00 = M.cells.adjacent(c0, c0_lv0);
        const GEO::index_t nc01 = mesh.cells.adjacent(c0, c0_lv1);
        const GEO::index_t nc10 = mesh.cells.adjacent(c1, c1_lv0);
        // const GEO::index_t nc11 = M.cells.adjacent(c1, c1_lv1);
        const GEO::index_t nc20 = mesh.cells.adjacent(c2, mesh.cells.find_tet_vertex(c2, v0));
        const GEO::index_t nc21 = mesh.cells.adjacent(c2, mesh.cells.find_tet_vertex(c2, v1));

        /* Set vertices */
        mesh.cells.set_vertex(c0, c0_lv0, v4);
        mesh.cells.set_vertex(c1, c1_lv1, v3);

        /* Set adjacent */
        // M.cells.set_adjacent(c0, c0_lv0, nc00);
        mesh.cells.set_adjacent(c0, c0_lv1, c1);
        mesh.cells.set_adjacent(c0, c0_lv2, nc20);
        mesh.cells.set_adjacent(c0, c0_lv3, nc10);
        mesh.cells.set_adjacent(c1, c1_lv0, c0);
        // M.cells.set_adjacent(c1, c1_lv1, nc11);
        mesh.cells.set_adjacent(c1, c1_lv2, nc21);
        mesh.cells.set_adjacent(c1, c1_lv4, nc01);
        if (nc01 != GEO::NO_CELL) {
            const GEO::index_t nlf = mesh.cells.find_tet_facet(nc01, v0, v2, v3);
            assert(nlf != GEO::NO_INDEX);
            mesh.cells.set_adjacent(nc01, nlf, c1);
        }
        if (nc10 != GEO::NO_CELL) {
            const GEO::index_t nlf = mesh.cells.find_tet_facet(nc10, v1, v2, v4);
            assert(nlf != GEO::NO_INDEX);
            mesh.cells.set_adjacent(nc10, nlf, c0);
        }
        if (nc20 != GEO::NO_CELL) {
            const GEO::index_t nlf = mesh.cells.find_tet_facet(nc20, v1, v4, v3);
            assert(nlf != GEO::NO_INDEX);
            mesh.cells.set_adjacent(nc20, nlf, c0);
        }
        if (nc21 != GEO::NO_CELL) {
            const GEO::index_t nlf = mesh.cells.find_tet_facet(nc21, v0, v3, v4);
            assert(nlf != GEO::NO_INDEX);
            mesh.cells.set_adjacent(nc21, nlf, c1);
        }

        if (update_attributes) {
            /* Cells */
            mesh.cells.attributes().zero_item(c0);
            mesh.cells.attributes().zero_item(c1);

            /* Cell corners */
            for (GEO::index_t lv = 0; lv < 4; ++lv) {
                mesh.cell_corners.attributes().zero_item(mesh.cells.corner(c0, lv));
                mesh.cell_corners.attributes().zero_item(mesh.cells.corner(c1, lv));
            }

            /* Cell facets */
            const GEO::index_t c2_lv0 = mesh.cells.find_tet_vertex(c2, v0);
            assert(c2_lv0 != GEO::NO_INDEX);
            mesh.cell_facets.attributes().copy_item(mesh.cells.facet(c0, c0_lv2), mesh.cells.facet(c2, c2_lv0));
            mesh.cell_facets.attributes().copy_item(mesh.cells.facet(c0, c0_lv3), mesh.cells.facet(c1, c1_lv0));
            const GEO::index_t c2_lv1 = mesh.cells.find_tet_vertex(c2, v1);
            assert(c2_lv1 != GEO::NO_INDEX);
            mesh.cell_facets.attributes().copy_item(mesh.cells.facet(c1, c1_lv2), mesh.cells.facet(c2, c2_lv1));
            mesh.cell_facets.attributes().copy_item(mesh.cells.facet(c1, c1_lv4), mesh.cells.facet(c0, c0_lv1));
            mesh.cell_facets.attributes().zero_item(mesh.cells.facet(c0, c0_lv1));
            mesh.cell_facets.attributes().zero_item(mesh.cells.facet(c1, c1_lv0));
        }

        return true;
    }
}
