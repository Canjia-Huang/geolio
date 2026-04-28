//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/3/23.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOGRAM_MESH_UTILS_TETRAHEDRON_OPERATIONS_H
#define GEOGRAM_MESH_UTILS_TETRAHEDRON_OPERATIONS_H

#include <geogram/mesh/mesh.h>
#include <cassert>
#include <ranges>
#include <unordered_set>
#include "tet_descriptor.h"

namespace GEO::MeshUtils::Tet
{
    /**
     * Returns the third vertex of a tetrahedron facet from two known facet vertices.
     *
     * @param[in] M  Input tetrahedral mesh.
     * @param[in] c  Cell index.
     * @param[in] lf Local facet index (0..3) in cell @p c.
     * @param[in] v0 First known vertex on facet (@p c, @p lf).
     * @param[in] v1 Second known vertex on facet (@p c, @p lf); must be different from @p v0.
     * @return The facet vertex in (@p c, @p lf) that is different from @p v0 and @p v1.
     *
     * @note Preconditions (debug-checked with assertions): @p c and @p lf are valid,
     *       and both @p v0 and @p v1 belong to facet (@p c, @p lf).
     */
    inline GEO::index_t get_cell_facet_another_vertex(
        const GEO::Mesh& M,
        const GEO::index_t c,
        const GEO::index_t lf,
        const GEO::index_t v0,
        const GEO::index_t v1
        ) {
        assert(c < M.cells.nb());
        assert(lf < 4);
        assert(M.cells.facet_vertex(c, lf, 0) == v0 || M.cells.facet_vertex(c, lf, 1) == v0 || M.cells.facet_vertex(c, lf, 2) == v0);
        assert(M.cells.facet_vertex(c, lf, 0) == v1 || M.cells.facet_vertex(c, lf, 1) == v1 || M.cells.facet_vertex(c, lf, 2) == v1);

        return M.cells.facet_vertex(c, lf, 0)^
               M.cells.facet_vertex(c, lf, 1)^
               M.cells.facet_vertex(c, lf, 2)^
               v0^
               v1;

        GEO::index_t v2 = GEO::NO_VERTEX;
        for (GEO::index_t lv = 0; lv < 3; ++lv) {
            v2 = M.cells.facet_vertex(c, lf, lv);
            if (v2 != v0 && v2 != v1)
                break;
        }
        return v2;
    }

    /**
     * Collects tetrahedra incident to a mesh vertex in traversal order.
     *
     * Starting from a cell-local-vertex seed, this function traces the
     * incident tetrahedra around the queried vertex and outputs an ordered
     * list of (cell, local-vertex) pairs.
     *
     * @param[in]     M              The tetrahedral mesh to query.
     * @param[in]     _c        Index of the seed cell.
     * @param[in]     _lv       Local vertex index (0-3) in @p start_c.
     * @param[in,out] c_and_lv       Output ordered (cell, local-vertex) list; existing contents are cleared.
     * @return true if the queried vertex is on the mesh border; false if it is an interior vertex.
     */
    inline bool get_vertex_incident_tetrahedra(
        const GEO::Mesh& M,
        const GEO::index_t _c,
        const GEO::index_t _lv,
        std::vector<std::pair<GEO::index_t, GEO::index_t>>& c_and_lv
        ) {
        assert(_c < M.cells.nb());
        assert(_lv < 4);

        c_and_lv.clear();

        const auto v = M.cells.vertex(_c, _lv);

        std::unordered_set<GEO::index_t> processed_cells;
        bool is_on_border = false;

        std::stack<std::pair<GEO::index_t, GEO::index_t>> stack;
        stack.emplace(_c, _lv);
        while (!stack.empty()) {
            const auto [c, lv] = stack.top();
            stack.pop();

            if (!processed_cells.insert(c).second)
                continue;
            c_and_lv.emplace_back(c, lv);

            for (const auto& lf : TET_LV_INCIDENT_LF[lv]) {
                if (const auto nc = M.cells.adjacent(c, lf);
                    nc != GEO::NO_CELL
                    ) {
                    const auto nlv = M.cells.find_tet_vertex(nc, v);
                    assert(nlv != GEO::NO_INDEX);
                    stack.emplace(nc, nlv);
                    }
                else
                    is_on_border = true;
            }
        }

        return is_on_border;
    }

    /**
     * Collects tetrahedra incident to a mesh edge in traversal order.
     *
     * Starting from a cell-local-edge seed, this function traces the edge
     * ring (or border chain) and outputs an ordered list of (cell, local-facet)
     * pairs, where each local facet contains the queried edge.
     *
     * @param[in]     M                 The tetrahedral mesh to query
     * @param[in]     _c           Index of the seed cell
     * @param[in]     _le          Local edge index (0-5) in @p start_c
     * @param[in,out] ordered_c_and_lf  Output ordered (cell, local-facet) list; existing contents are cleared
     * @return true if the queried edge is on the mesh border; false if it is an interior edge
     */
    inline bool get_edge_incident_tetrahedra(
        const GEO::Mesh& M,
        const GEO::index_t _c,
        const GEO::index_t _le,
        std::vector<std::pair<GEO::index_t, GEO::index_t>>& ordered_c_and_lf
        ) {
        assert(_c < M.cells.nb());
        assert(_le < 6);

        const auto ev0 = M.cells.edge_vertex(_c, _le, 0);
        const auto ev1 = M.cells.edge_vertex(_c, _le, 1);
        bool is_on_border = false;

        std::vector<std::pair<GEO::index_t, GEO::index_t>> next_ordered_c_and_lf;
        std::vector<std::pair<GEO::index_t, GEO::index_t>> prev_ordered_c_and_lf;
        {
            GEO::index_t c = _c;
            GEO::index_t lf = M.cells.edge_adjacent_facet(_c, _le, 0);
            for (;;) {
                next_ordered_c_and_lf.emplace_back(c, lf);

                const GEO::index_t nc = M.cells.adjacent(c, lf);
                if (nc == GEO::NO_CELL) {
                    is_on_border = true;
                    break;
                }
                if (nc == _c) // a loop
                    break;

                /* Get next lf */
                const GEO::index_t oppo_v = get_cell_facet_another_vertex(M, c, lf, ev0, ev1); // (oppo_v, ev0, ev1) form the cell c's lf
                assert(oppo_v != GEO::NO_VERTEX);

                lf = M.cells.find_tet_vertex(nc, oppo_v);
                assert(lf != GEO::NO_INDEX);
                c = nc;
            }
        }

        if (is_on_border) {
            GEO::index_t c = _c;
            GEO::index_t lf = M.cells.edge_adjacent_facet(_c, _le, 1);
            for (;;) {
                const GEO::index_t nc = M.cells.adjacent(c, lf);
                if (nc == GEO::NO_CELL)
                    break;

                const GEO::index_t nlf = M.cells.find_tet_facet(
                    nc,
                    M.cells.facet_vertex(c, lf, 2),
                    M.cells.facet_vertex(c, lf, 1),
                    M.cells.facet_vertex(c, lf, 0));
                assert(nlf != GEO::NO_INDEX);

                prev_ordered_c_and_lf.emplace_back(nc, nlf);

                /* Get next lf */
                const GEO::index_t oppo_v = get_cell_facet_another_vertex(M, nc, nlf, ev0, ev1); // (oppo_v, ev0, ev1) form the cell c's lf
                assert(oppo_v != GEO::NO_VERTEX);

                lf = M.cells.find_tet_vertex(nc, oppo_v);
                assert(lf != GEO::NO_INDEX);
                c = nc;
            }
        }

        /* Output */
        ordered_c_and_lf.clear();
        ordered_c_and_lf.reserve(next_ordered_c_and_lf.size() + prev_ordered_c_and_lf.size());
        for (GEO::index_t i = 0, i_end = prev_ordered_c_and_lf.size(); i < i_end; ++i)
            ordered_c_and_lf.push_back(prev_ordered_c_and_lf[i_end-i-1]);
        for (const auto& c_lf : next_ordered_c_and_lf)
            ordered_c_and_lf.push_back(c_lf);

        return is_on_border;
    }

    /**
     * Collects tetrahedra incident to a mesh edge in traversal order.
     *
     * Starting from a cell-facet-local-vertex seed, this function traces the edge
     * ring (or border chain) and outputs an ordered list of (cell, local-facet)
     * pairs, where each local facet contains the queried edge.
     *
     * @param[in]     M                 The tetrahedral mesh to query
     * @param[in]     _c           Index of the seed cell
     * @param[in]     _lf          Local facet index (0-3) in @p start_c
     * @param[in]     _lv          Local vertex index (0-2) in facet @p start_lf; with
     *                                  `(start_lv + 1) % 3` it defines the queried edge
     * @param[in,out] ordered_c_and_lf  Output ordered (cell, local-facet) list; existing contents are cleared
     * @return true if the queried edge is on the mesh border; false if it is an interior edge
     */
    inline bool get_edge_incident_tetrahedra(
        const GEO::Mesh& M,
        const GEO::index_t _c,
        const GEO::index_t _lf,
        const GEO::index_t _lv,
        std::vector<std::pair<GEO::index_t, GEO::index_t>>& ordered_c_and_lf
        ) {
        assert(_c < M.cells.nb());
        assert(_lf < 4);
        assert(_lv < 3);

        const auto ev0 = M.cells.facet_vertex(_c, _lf, _lv);
        const auto ev1 = M.cells.facet_vertex(_c, _lf, (_lv+1)%3);

        for (const auto& start_le : TET_LF_INCIDENT_LE[_lf]) {
            const auto cev0 = M.cells.edge_vertex(_c, start_le, 0);
            const auto cev1 = M.cells.edge_vertex(_c, start_le, 1);
            if ((cev0 == ev0 && cev1 == ev1) ||
                (cev0 == ev1 && cev1 == ev0))
                return get_edge_incident_tetrahedra(M, _c, start_le, ordered_c_and_lf);
        }
        assert(0);
    }

    /**
     * Splits one tetrahedron into four tetrahedra by inserting an interior vertex.
     *
     * The vertex index @p new_v is expected to be pre-allocated. Its position will be set
     * to the barycenter of cell @p c. The original cell @p c is updated in-place and
     * three additional tetrahedra (@p new_c0, @p new_c1, @p new_c2) are filled.
     * Adjacency between the four resulting cells and neighboring cells is updated.
     *
     * @param[in,out] M      The tetrahedral mesh to modify
     * @param[in]     c      Index of the tetrahedron to split
     * @param[in]     new_v  Index of a pre-allocated vertex used as the split vertex
     * @param[in]     new_c0 Index of the first pre-allocated tetrahedron created by the split
     * @param[in]     new_c1 Index of the second pre-allocated tetrahedron created by the split
     * @param[in]     new_c2 Index of the third pre-allocated tetrahedron created by the split
     */
    inline void cell_split(
        GEO::Mesh& M,
        const GEO::index_t c,
        const GEO::index_t new_v,
        const GEO::index_t new_c0,
        const GEO::index_t new_c1,
        const GEO::index_t new_c2
        ) {
        assert(c < M.cells.nb());
        assert(new_v < M.cells.nb());
        assert(new_c0 < M.cells.nb());
        assert(new_c1 < M.cells.nb());
        assert(new_c2 < M.cells.nb());

        const GEO::index_t v0 = M.cells.vertex(c, 0);
        const GEO::index_t v1 = M.cells.vertex(c, 1);
        const GEO::index_t v2 = M.cells.vertex(c, 2);
        const GEO::index_t v3 = M.cells.vertex(c, 3);
        const GEO::index_t nc0 = M.cells.adjacent(c, 0);
        const GEO::index_t nc1 = M.cells.adjacent(c, 1);
        const GEO::index_t nc2 = M.cells.adjacent(c, 2);
        // const GEO::index_t nc3 = M.cells.adjacent(c, 3);

        /* Create new vertex */
        M.vertices.point(new_v) = 0.25 * (
            M.cells.point(c, 0) + M.cells.point(c, 1) + M.cells.point(c, 2) + M.cells.point(c, 3));

        /* Set vertices */
        M.cells.set_vertex(new_c0, 0, new_v);
        M.cells.set_vertex(new_c0, 1, v1);
        M.cells.set_vertex(new_c0, 2, v2);
        M.cells.set_vertex(new_c0, 3, v3);
        M.cells.set_vertex(new_c1, 0, v0);
        M.cells.set_vertex(new_c1, 1, new_v);
        M.cells.set_vertex(new_c1, 2, v2);
        M.cells.set_vertex(new_c1, 3, v3);
        M.cells.set_vertex(new_c2, 0, v0);
        M.cells.set_vertex(new_c2, 1, v1);
        M.cells.set_vertex(new_c2, 2, new_v);
        M.cells.set_vertex(new_c2, 3, v3);
        M.cells.set_vertex(c, 3, new_v);

        /* Set adjacency */
        M.cells.set_adjacent(new_c0, 0, nc0);
        M.cells.set_adjacent(new_c0, 1, new_c1);
        M.cells.set_adjacent(new_c0, 2, new_c2);
        M.cells.set_adjacent(new_c0, 3, c);
        M.cells.set_adjacent(new_c1, 0, new_c0);
        M.cells.set_adjacent(new_c1, 1, nc1);
        M.cells.set_adjacent(new_c1, 2, new_c2);
        M.cells.set_adjacent(new_c1, 3, c);
        M.cells.set_adjacent(new_c2, 0, new_c0);
        M.cells.set_adjacent(new_c2, 1, new_c1);
        M.cells.set_adjacent(new_c2, 2, nc2);
        M.cells.set_adjacent(new_c2, 3, c);
        M.cells.set_adjacent(c, 0, new_c0);
        M.cells.set_adjacent(c, 1, new_c1);
        M.cells.set_adjacent(c, 2, new_c2);
        if (nc0 != GEO::NO_CELL) {
            assert(M.cells.find_tet_facet(nc0, v1, v2, v3) != GEO::NO_INDEX);
            M.cells.set_adjacent(nc0, M.cells.find_tet_facet(nc0, v1, v2, v3), new_c0);
        }
        if (nc1 != GEO::NO_CELL) {
            assert(M.cells.find_tet_facet(nc1, v0, v3, v2) != GEO::NO_INDEX);
            M.cells.set_adjacent(nc1, M.cells.find_tet_facet(nc1, v0, v3, v2), new_c1);
        }
        if (nc2 != GEO::NO_CELL) {
            assert(M.cells.find_tet_facet(nc2, v0, v1, v3) != GEO::NO_INDEX);
            M.cells.set_adjacent(nc2, M.cells.find_tet_facet(nc2, v0, v1, v3), new_c2);
        }
    }

    /**
     * Splits a tetrahedral facet by inserting a new vertex on the facet.
     *
     * The vertex index @p new_v is expected to be pre-allocated. Its position will be set
     * to the barycenter of facet @p lf in cell @p c. The incident tetrahedral connectivity
     * is updated, and the resulting tetrahedra are written into the pre-allocated cells.
     * For a boundary facet, only @p new_c0 and @p new_c1 are used. For an interior facet,
     * @p new_c2 and @p new_c3 are also used.
     *
     * @param[in,out] M      The tetrahedral mesh to modify.
     * @param[in]     c      Index of the tetrahedron that owns the target facet.
     * @param[in]     lf     Local facet index (0-3) in cell @p c.
     * @param[in]     new_v  Index of a pre-allocated vertex used as the split vertex.
     * @param[in]     new_c0 Index of the first pre-allocated tetrahedron created by the split.
     * @param[in]     new_c1 Index of the second pre-allocated tetrahedron created by the split.
     * @param[in]     new_c2 Optional; index of the third pre-allocated tetrahedron used for interior facets.
     * @param[in]     new_c3 Optional; index of the fourth pre-allocated tetrahedron used for interior facets.
     */
    inline void cell_facet_split(
        GEO::Mesh& M,
        const GEO::index_t c,
        const GEO::index_t lf,
        const GEO::index_t new_v,
        const GEO::index_t new_c0,
        const GEO::index_t new_c1,
        const GEO::index_t new_c2 = GEO::NO_CELL,
        const GEO::index_t new_c3 = GEO::NO_CELL
        ) {
        assert(c < M.cells.nb());
        assert(lf < 4);
        assert(new_v < M.vertices.nb());
        assert(new_c0 < M.cells.nb());
        assert(new_c1 < M.cells.nb());

        /* Set new vertex */
        M.vertices.point(new_v) = (M.vertices.point(M.cells.facet_vertex(c, lf, 0)) +
                                    M.vertices.point(M.cells.facet_vertex(c, lf, 1)) +
                                    M.vertices.point(M.cells.facet_vertex(c, lf, 2))) / 3;


        const GEO::index_t v0 = M.cells.vertex(c, TET_LF_INCIDENT_LV[lf][0]);
        const GEO::index_t v1 = M.cells.vertex(c, TET_LF_INCIDENT_LV[lf][1]);
        const GEO::index_t v2 = M.cells.vertex(c, TET_LF_INCIDENT_LV[lf][2]);
        {
            const GEO::index_t lv0 = TET_LF_INCIDENT_LV[lf][0];
            const GEO::index_t lv1 = TET_LF_INCIDENT_LV[lf][1];
            const GEO::index_t lv2 = TET_LF_INCIDENT_LV[lf][2];
            const GEO::index_t lv3 = (0^1^2^3)^lv0^lv1^lv2;
            assert(lv3 < 4 && lv3 != lv0 && lv3 != lv1 && lv3 != lv2);

            const GEO::index_t v3 = M.cells.vertex(c, lv3);

            const GEO::index_t nc0 = M.cells.adjacent(c, lv0);
            const GEO::index_t nc1 = M.cells.adjacent(c, lv1);

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
            M.cells.set_vertex(c, lv2, new_v);
            M.cells.set_vertex(new_c0, lv0, new_v);
            M.cells.set_vertex(new_c0, lv1, v1);
            M.cells.set_vertex(new_c0, lv2, v2);
            M.cells.set_vertex(new_c0, lv3, v3);
            M.cells.set_vertex(new_c1, lv0, v0);
            M.cells.set_vertex(new_c1, lv1, new_v);
            M.cells.set_vertex(new_c1, lv2, v2);
            M.cells.set_vertex(new_c1, lv3, v3);

            /* Set cells adjacent */
            M.cells.set_adjacent(c, lv0, new_c0);
            M.cells.set_adjacent(c, lv1, new_c1);
            M.cells.set_adjacent(new_c0, lv0, nc0);
            M.cells.set_adjacent(new_c0, lv1, new_c1);
            M.cells.set_adjacent(new_c0, lv2, c);
            M.cells.set_adjacent(new_c0, lv3, new_c2);
            M.cells.set_adjacent(new_c1, lv0, new_c0);
            M.cells.set_adjacent(new_c1, lv1, nc1);
            M.cells.set_adjacent(new_c1, lv2, c);
            M.cells.set_adjacent(new_c1, lv3, new_c3);
            if (nc0 != GEO::NO_CELL) {
                const GEO::index_t nlf = M.cells.find_tet_facet(
                    nc0,
                    M.cells.facet_vertex(new_c0, lv0, 2),
                    M.cells.facet_vertex(new_c0, lv0, 1),
                    M.cells.facet_vertex(new_c0, lv0, 0));
                assert(nlf != GEO::NO_INDEX);
                M.cells.set_adjacent(nc0, nlf, new_c0);
            }
            if (nc1 != GEO::NO_CELL) {
                const GEO::index_t nlf = M.cells.find_tet_facet(
                    nc1,
                    M.cells.facet_vertex(new_c1, lv1, 2),
                    M.cells.facet_vertex(new_c1, lv1, 1),
                    M.cells.facet_vertex(new_c1, lv1, 0));
                assert(nlf != GEO::NO_INDEX);
                M.cells.set_adjacent(nc1, nlf, new_c1);
            }
        }

        if (const auto& ac = M.cells.adjacent(c, lf);
            ac != GEO::NO_CELL
            ) {
            assert(new_c2 < M.cells.nb());
            assert(new_c3 < M.cells.nb());

            const GEO::index_t lv0 = M.cells.find_tet_vertex(ac, v0);
            const GEO::index_t lv1 = M.cells.find_tet_vertex(ac, v2);
            const GEO::index_t lv2 = M.cells.find_tet_vertex(ac, v1);
            const GEO::index_t lv3 = (0^1^2^3)^lv0^lv1^lv2;
            assert(lv3 < 4 && lv3 != lv0 && lv3 != lv1 && lv3 != lv2);

            const GEO::index_t v3 = M.cells.vertex(ac, lv3);

            const GEO::index_t nc0 = M.cells.adjacent(ac, lv0);
            const GEO::index_t nc2 = M.cells.adjacent(ac, lv2);

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
            M.cells.set_vertex(ac, lv1, new_v);
            M.cells.set_vertex(new_c2, lv0, new_v);
            M.cells.set_vertex(new_c2, lv1, v2);
            M.cells.set_vertex(new_c2, lv2, v1);
            M.cells.set_vertex(new_c2, lv3, v3);
            M.cells.set_vertex(new_c3, lv0, v0);
            M.cells.set_vertex(new_c3, lv1, v2);
            M.cells.set_vertex(new_c3, lv2, new_v);
            M.cells.set_vertex(new_c3, lv3, v3);

            /* Set cells adjacent */
            M.cells.set_adjacent(ac, lv0, new_c2);
            M.cells.set_adjacent(ac, lv2, new_c3);
            M.cells.set_adjacent(new_c2, lv0, nc0);
            M.cells.set_adjacent(new_c2, lv1, ac);
            M.cells.set_adjacent(new_c2, lv2, new_c3);
            M.cells.set_adjacent(new_c2, lv3, new_c0);
            M.cells.set_adjacent(new_c3, lv0, new_c2);
            M.cells.set_adjacent(new_c3, lv1, ac);
            M.cells.set_adjacent(new_c3, lv2, nc2);
            M.cells.set_adjacent(new_c3, lv3, new_c1);
            if (nc0 != GEO::NO_CELL) {
                const GEO::index_t nlf = M.cells.find_tet_facet(
                    nc0,
                    M.cells.facet_vertex(new_c2, lv0, 2),
                    M.cells.facet_vertex(new_c2, lv0, 1),
                    M.cells.facet_vertex(new_c2, lv0, 0));
                assert(nlf != GEO::NO_INDEX);
                M.cells.set_adjacent(nc0, nlf, new_c2);
            }
            if (nc2 != GEO::NO_CELL) {
                const GEO::index_t nlf = M.cells.find_tet_facet(
                    nc2,
                    M.cells.facet_vertex(new_c3, lv2, 2),
                    M.cells.facet_vertex(new_c3, lv2, 1),
                    M.cells.facet_vertex(new_c3, lv2, 0));
                assert(nlf != GEO::NO_INDEX);
                M.cells.set_adjacent(nc2, nlf, new_c3);
            }
        }
    }

    /**
     * Splits a tetrahedral edge by inserting one vertex and splitting all incident cells.
     *
     * The edge is identified by local edge index @p le in seed cell @p _c.
     * The function traverses all tetrahedra incident to that edge, places @p new_v
     * on the edge using interpolation ratio @p r, and splits each incident tetrahedron
     * into two tetrahedra while updating adjacency relations.
     *
     * New tetrahedra are written starting at cell index @p _new_c. If the available
     * range `[_new_c, M.cells.nb())` is not large enough, additional cells are created.
     * On return, @p _new_c is advanced by the number of incident tetrahedra consumed.
     *
     * @param[in,out] M      The tetrahedral mesh to modify.
     * @param[in]     _c     Index of a seed cell containing the target edge.
     * @param[in]     le     Local edge index (0-5) in cell @p _c.
     * @param[in]     new_v  Index of a pre-allocated vertex used as the split vertex.
     * @param[in,out] _new_c Input: first cell index available for writing new tetrahedra;
     *                       output: advanced to the next free cell index.
     * @param[in]     r      Interpolation ratio for placing @p new_v on the edge
     *                       (`0` at the first endpoint, `1` at the second).
     */
    inline void cell_edge_split(
        GEO::Mesh& M,
        const GEO::index_t _c,
        const GEO::index_t le,
        const GEO::index_t new_v,
        GEO::index_t& _new_c,
        const double r = 0.5
        ) {
        assert(_c < M.cells.nb());
        assert(le < 6);
        assert(new_v < M.vertices.nb());
        assert(_new_c < M.cells.nb());

        /* Find all adjacent cells */
        std::vector<std::pair<GEO::index_t, GEO::index_t>> ordered_c_and_lf;
        get_edge_incident_tetrahedra(M, _c, le, ordered_c_and_lf);
        const GEO::index_t INCIDENT_CELLS_NB = ordered_c_and_lf.size();
        if (_new_c+INCIDENT_CELLS_NB > M.cells.nb()) {
            const GEO::index_t new_cells_nb = INCIDENT_CELLS_NB-(M.cells.nb()-_new_c);
            M.cells.create_tets(new_cells_nb);
        }
        assert(_new_c+INCIDENT_CELLS_NB <= M.cells.nb());

        const GEO::index_t ev0 = M.cells.edge_vertex(_c, le, 0);
        const GEO::index_t ev1 = M.cells.edge_vertex(_c, le, 1);

        /* Set new vertex */
        M.vertices.point(new_v) = (1-r)*M.vertices.point(ev0) + r*M.vertices.point(ev1);

        for (GEO::index_t i = 0; i < INCIDENT_CELLS_NB; ++i) {
            const auto& [c, lf0] = ordered_c_and_lf[i];
            const auto& new_c = _new_c+i;

            const GEO::index_t lv0 = M.cells.find_tet_vertex(c, ev0);
            const GEO::index_t lv1 = M.cells.find_tet_vertex(c, ev1);
            const GEO::index_t lv2 = TET_LF_INCIDENT_LV[lf0][0]^TET_LF_INCIDENT_LV[lf0][1]^TET_LF_INCIDENT_LV[lf0][2]^lv0^lv1;
            assert(lv2 < 4 && lv2 != lv0 && lv2 != lv1);
            const GEO::index_t lv3 = 0^1^2^3^lv0^lv1^lv2;
            assert(lv3 < 4 && lv3 != lv0 && lv3 != lv1 && lv3 != lv2);

            const GEO::index_t v2 = M.cells.vertex(c, lv2);
            const GEO::index_t v3 = M.cells.vertex(c, lv3);

            // const GEO::index_t nc0 = M.cells.adjacent(c, lv0);
            const GEO::index_t nc1 = M.cells.adjacent(c, lv1);

            /* Set cell vertices */
            M.cells.set_vertex(c, lv0, new_v);
            M.cells.set_vertex(new_c, lv0, ev0);
            M.cells.set_vertex(new_c, lv1, new_v);
            M.cells.set_vertex(new_c, lv2, v2);
            M.cells.set_vertex(new_c, lv3, v3);

            /* Set cell adjacent */
            M.cells.set_adjacent(c, lv1, new_c);
            M.cells.set_adjacent(new_c, lv0, c);
            M.cells.set_adjacent(new_c, lv1, nc1);
            if (M.cells.adjacent(c, lv2) != GEO::NO_CELL)
                M.cells.set_adjacent(new_c, lv2, _new_c+(i+INCIDENT_CELLS_NB-1)%INCIDENT_CELLS_NB);
            if (M.cells.adjacent(c, lv3) != GEO::NO_CELL)
                M.cells.set_adjacent(new_c, lv3, _new_c+(i+1)%INCIDENT_CELLS_NB);
            if (nc1 != GEO::NO_CELL) {
                const GEO::index_t nlf = M.cells.find_tet_facet(
                    nc1,
                    M.cells.facet_vertex(new_c, lv1, 2),
                    M.cells.facet_vertex(new_c, lv1, 1),
                    M.cells.facet_vertex(new_c, lv1, 0));
                assert(nlf != GEO::NO_INDEX);
                M.cells.set_adjacent(nc1, nlf, new_c);
            }
        }

        _new_c += INCIDENT_CELLS_NB;
    }

    /**
     * Collapses a tetrahedral edge by moving one endpoint along the edge and
     * updating the local cavity connectivity.
     *
     * The edge is identified by local edge index @p le in cell @p c.
     * Parameter @p r controls the new endpoint position by interpolation on the
     * edge segment (`0` keeps the first endpoint, `1` keeps the second endpoint).
     * Collapsed cells/vertices are reported through optional output arguments.
     *
     * @param[in,out] M         The tetrahedral mesh to modify.
     * @param[in]     _c         Index of a cell containing the target edge.
     * @param[in]     _le        Local edge index (0-5) in cell @p c.
     * @param[in]     r         Interpolation ratio for the kept vertex position on the edge.
     * @param[out]    disuse_v  Receives the removed vertex index.
     * @param[out]    disuse_cs Receives indices of cells removed by the collapse.
     */
    inline void cell_edge_collapse(
        GEO::Mesh& M,
        const GEO::index_t _c,
        const GEO::index_t _le,
        GEO::index_t& disuse_v,
        std::vector<GEO::index_t>& disuse_cs,
        const double r = 0.5
        ) {
        assert(_c < M.cells.nb());
        assert(_le < 6);
        assert(r >= 0 && r <= 1);

        const auto& ev0 = M.cells.edge_vertex(_c, _le, 0);
        const auto& ev1 = M.cells.edge_vertex(_c, _le, 1);

        /* Move vertex */
        auto& ep0 = M.vertices.point(ev0);
        const auto& ep1 = M.vertices.point(ev1);
        ep0 = (1-r)*ep0 + r*ep1;
        disuse_v = ev1;

        /* Find all adjacent tets */
        std::vector<std::pair<GEO::index_t, GEO::index_t>> ordered_c_and_lf;
        get_edge_incident_tetrahedra(M, _c, _le, ordered_c_and_lf);

        std::vector<std::pair<GEO::index_t, GEO::index_t>> ev1_incident_c_and_lv;
        get_vertex_incident_tetrahedra(M, _c, TET_LE_INCIDENT_LV[_le][1], ev1_incident_c_and_lv);

        /* Collapse */
        for (const auto& c: ordered_c_and_lf | std::views::keys) {
            const auto lf0 = M.cells.find_tet_vertex(c, ev0);
            const auto lf1 = M.cells.find_tet_vertex(c, ev1);
            const auto nc0 = M.cells.adjacent(c, lf0);
            const auto nc1 = M.cells.adjacent(c, lf1);

            if (nc0 != GEO::NO_CELL) {
                /* Set adjacent */
                const auto nlf = M.cells.find_tet_facet(
                    nc0,
                    M.cells.facet_vertex(c, lf0, 2),
                    M.cells.facet_vertex(c, lf0, 1),
                    M.cells.facet_vertex(c, lf0, 0));
                assert(nlf != GEO::NO_INDEX);
                M.cells.set_adjacent(nc0, nlf, nc1);
            }
            if (nc1 != GEO::NO_CELL) {
                /* Set adjacent */
                const auto nlf = M.cells.find_tet_facet(
                    nc1,
                    M.cells.facet_vertex(c, lf1, 2),
                    M.cells.facet_vertex(c, lf1, 1),
                    M.cells.facet_vertex(c, lf1, 0));
                assert(nlf != GEO::NO_INDEX);
                M.cells.set_adjacent(nc1, nlf, nc0);
            }

            disuse_cs.push_back(c);
        }

        /* Update vertex of other cells */
        for (const auto& [c, lv] :ev1_incident_c_and_lv)
            M.cells.set_vertex(c, lv, ev0);
    }

    /**
     * Performs a 2-3 facet swap operation on a tetrahedral mesh.
     *
     * This operation replaces two tetrahedra that share a common facet with
     * three tetrahedra by flipping that facet. Given a seed cell @p c and its
     * local facet @p lf, this function identifies the adjacent tetrahedron
     * across the facet and updates the local connectivity accordingly.
     *
     * @param[in,out] M      The tetrahedral mesh to modify.
     * @param[in]     c      Index of the seed cell containing the target facet.
     * @param[in]     lf     Local facet index (0-3) of cell @p c.
     * @param[in,out] new_c  Index of the pre-allocated cell used to store the newly created tetrahedron.
     * @return true if the swap is performed successfully; false if the target facet is on the border or the operation cannot be applied.
     */
    inline bool cell_edge_swap_2_3(
        GEO::Mesh& M,
        const GEO::index_t c,
        const GEO::index_t lf,
        const GEO::index_t new_c
        ) {
        assert(c < M.cells.nb());
        assert(lf < 4);

        const GEO::index_t nc = M.cells.adjacent(c, lf);
        if (nc == GEO::NO_CELL)
            return false;

        assert(new_c < M.cells.nb());

        const GEO::index_t v = M.cells.vertex(c, lf);
        const GEO::index_t v0 = M.cells.facet_vertex(c, lf, 0);
        const GEO::index_t v1 = M.cells.facet_vertex(c, lf, 1);
        const GEO::index_t v2 = M.cells.facet_vertex(c, lf, 2);
        const GEO::index_t lv0 = TET_LF_INCIDENT_LV[lf][0];
        const GEO::index_t lv1 = TET_LF_INCIDENT_LV[lf][1];
        const GEO::index_t lv2 = TET_LF_INCIDENT_LV[lf][2];
        const GEO::index_t nc0 = M.cells.adjacent(c, lv0);
        const GEO::index_t nc1 = M.cells.adjacent(c, lv1);
        const GEO::index_t nc2 = M.cells.adjacent(c, lv2);

        const GEO::index_t nlf = M.cells.find_tet_facet(nc, v2, v1, v0);
        assert(nlf != GEO::NO_INDEX);
        GEO::index_t nlv0{GEO::NO_INDEX}, nlv1{GEO::NO_INDEX}, nlv2{GEO::NO_INDEX};
        for (GEO::index_t i = 0; i < 3; ++i) {
            if (M.cells.facet_vertex(nc, nlf, i) == v0) { // cell_vertex(nc, nlv0) == cell_vertex(c, lv0)
                nlv0 = TET_LF_INCIDENT_LV[nlf][i];
                nlv1 = TET_LF_INCIDENT_LV[nlf][(i+1)%3];
                nlv2 = TET_LF_INCIDENT_LV[nlf][(i+2)%3];
                break;
            }
        }
        assert(nlv0 != GEO::NO_INDEX && nlv1 != GEO::NO_INDEX && nlv2 != GEO::NO_INDEX);
        const GEO::index_t nv = M.cells.vertex(nc, nlf);
        const GEO::index_t nv0 = M.cells.vertex(nc, nlv0);
        assert(nv0 == v0);
        const GEO::index_t nv1 = M.cells.vertex(nc, nlv1);
        assert(nv1 == v2);
        const GEO::index_t nv2 = M.cells.vertex(nc, nlv2);
        assert(nv2 == v1);
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

        return true;
    }

    /**
     * Performs a 3-2 edge swap operation on a tetrahedral mesh.
     *
     * This operation replaces 3 tetrahedra sharing a common edge with 2 tetrahedra
     * by removing the shared edge. Given a cell @p _c with a local edge @p _le,
     * this function identifies all incident cells (expected to be exactly 3),
     * performs the topological transformation, and reports the removed cell.
     *
     * @param[in,out] M        The tetrahedral mesh to modify.
     * @param[in]     _c       Index of a seed cell containing the target edge.
     * @param[in]     _le      Local edge index (0-5) in cell @p _c.
     * @param[out]    disuse_c Reference to receive the index of the removed cell.
     * @return true if the swap was performed successfully; false if preconditions are not met.
     */
    inline bool cell_edge_swap_3_2(
        GEO::Mesh& M,
        const GEO::index_t _c,
        const GEO::index_t _le,
        GEO::index_t& disuse_c
        ) {
        assert(_c < M.cells.nb());
        assert(_le < 6);

        std::vector<std::pair<GEO::index_t, GEO::index_t>> ordered_c_and_lf;
        if(const bool is_on_border = get_edge_incident_tetrahedra(M, _c, _le, ordered_c_and_lf);
            is_on_border ||
            ordered_c_and_lf.size() != 3)
            return false;

        const GEO::index_t v0 = M.cells.edge_vertex(_c, _le, 0);
        const GEO::index_t v1 = M.cells.edge_vertex(_c, _le, 1);

        const GEO::index_t c0 = ordered_c_and_lf[0].first;
        const GEO::index_t c1 = ordered_c_and_lf[1].first;
        const GEO::index_t c2 = ordered_c_and_lf[2].first;
        disuse_c = c2;

        const GEO::index_t v2 = get_cell_facet_another_vertex(M, c0, ordered_c_and_lf[0].second, v0, v1);
        const GEO::index_t v4 = get_cell_facet_another_vertex(M, c1, ordered_c_and_lf[1].second, v0, v1);

        const GEO::index_t c0_lv0 = M.cells.find_tet_vertex(c0, v0);
        const GEO::index_t c0_lv1 = M.cells.find_tet_vertex(c0, v1);
        const GEO::index_t c0_lv2 = M.cells.find_tet_vertex(c0, v2);
        assert(c0_lv0 != GEO::NO_INDEX && c0_lv1 != GEO::NO_INDEX && c0_lv2 != GEO::NO_INDEX);
        const GEO::index_t c0_lv3 = 0^1^2^3^c0_lv0^c0_lv1^c0_lv2;
        assert(c0_lv3 < 4 && c0_lv3 != c0_lv0 && c0_lv3 != c0_lv1 && c0_lv3 != c0_lv2);

        const GEO::index_t v3 = M.cells.vertex(c0, c0_lv3);

        const GEO::index_t c1_lv0 = M.cells.find_tet_vertex(c1, v0);
        const GEO::index_t c1_lv1 = M.cells.find_tet_vertex(c1, v1);
        const GEO::index_t c1_lv4 = M.cells.find_tet_vertex(c1, v4);
        assert(c1_lv0 != GEO::NO_INDEX && c1_lv1 != GEO::NO_INDEX && c1_lv4 != GEO::NO_INDEX);
        const GEO::index_t c1_lv2 = 0^1^2^3^c1_lv0^c1_lv1^c1_lv4;
        assert(c1_lv2 < 4 && c1_lv2 != c1_lv0 && c1_lv2 != c1_lv1 && c1_lv2 != c1_lv4);

        // const GEO::index_t nc00 = M.cells.adjacent(c0, c0_lv0);
        const GEO::index_t nc01 = M.cells.adjacent(c0, c0_lv1);
        const GEO::index_t nc10 = M.cells.adjacent(c1, c1_lv0);
        // const GEO::index_t nc11 = M.cells.adjacent(c1, c1_lv1);
        const GEO::index_t nc20 = M.cells.adjacent(c2, M.cells.find_tet_vertex(c2, v0));
        const GEO::index_t nc21 = M.cells.adjacent(c2, M.cells.find_tet_vertex(c2, v1));

        /* Set vertices */
        M.cells.set_vertex(c0, c0_lv0, v4);
        M.cells.set_vertex(c1, c1_lv1, v3);

        /* Set adjacent */
        // M.cells.set_adjacent(c0, c0_lv0, nc00);
        M.cells.set_adjacent(c0, c0_lv1, c1);
        M.cells.set_adjacent(c0, c0_lv2, nc20);
        M.cells.set_adjacent(c0, c0_lv3, nc10);
        M.cells.set_adjacent(c1, c1_lv0, c0);
        // M.cells.set_adjacent(c1, c1_lv1, nc11);
        M.cells.set_adjacent(c1, c1_lv2, nc21);
        M.cells.set_adjacent(c1, c1_lv4, nc01);
        if (nc01 != GEO::NO_CELL) {
            const GEO::index_t nlf = M.cells.find_tet_facet(nc01, v0, v2, v3);
            assert(nlf != GEO::NO_INDEX);
            M.cells.set_adjacent(nc01, nlf, c1);
        }
        if (nc10 != GEO::NO_CELL) {
            const GEO::index_t nlf = M.cells.find_tet_facet(nc10, v1, v2, v4);
            assert(nlf != GEO::NO_INDEX);
            M.cells.set_adjacent(nc10, nlf, c0);
        }
        if (nc20 != GEO::NO_CELL) {
            const GEO::index_t nlf = M.cells.find_tet_facet(nc20, v1, v4, v3);
            assert(nlf != GEO::NO_INDEX);
            M.cells.set_adjacent(nc20, nlf, c0);
        }
        if (nc21 != GEO::NO_CELL) {
            const GEO::index_t nlf = M.cells.find_tet_facet(nc21, v0, v3, v4);
            assert(nlf != GEO::NO_INDEX);
            M.cells.set_adjacent(nc21, nlf, c1);
        }

        return true;
    }
}

#endif //GEOGRAM_MESH_UTILS_TETRAHEDRON_OPERATIONS_H

