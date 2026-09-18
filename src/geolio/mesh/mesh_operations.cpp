//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/6/25.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "mesh_operations.h"
#include <algorithm>
#include <cassert>
#include <stack>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>
#include <geogram/mesh/mesh_io.h>
#include "hex_operations.h"
#include "tet_operations.h"

namespace geolio
{
    bool get_vertex_incident_facets(
        const GEO::Mesh& mesh,
        const GEO::index_t start_f,
        const GEO::index_t start_lv,
        std::vector<std::pair<GEO::index_t, GEO::index_t>>& ordered_f_lv
        ) {
        assert(start_f < mesh.facets.nb());
        assert(start_lv < mesh.facets.nb_vertices(start_f));

        const GEO::index_t v = mesh.facets.vertex(start_f, start_lv);
        bool is_on_border = false;

        /* Filled in place (see get_edge_incident_cells()): forward ring first, then the inverse
         * travel for border vertices, reversed and rotated to the front. No local temporaries,
         * and the capacity of a caller-provided vector is preserved across calls. */
        ordered_f_lv.clear();
        if (ordered_f_lv.capacity() < 4)
            ordered_f_lv.reserve(4);

        /* Forward travel */
        {
            GEO::index_t f = start_f;
            GEO::index_t lv = start_lv;
            do {
                ordered_f_lv.emplace_back(f, lv);

                const GEO::index_t next_f = mesh.facets.adjacent(f, lv);
                if (next_f == GEO::NO_FACET) { // is not 2-manifold vertex
                    is_on_border = true;
                    break;
                }
                f = next_f;
                lv = mesh.facets.find_vertex(f, v);
                assert(lv != GEO::NO_INDEX);
            } while (f != start_f);
        }
        const auto forward_nb = static_cast<std::ptrdiff_t>(ordered_f_lv.size());

        /* Inverse travel */
        if (is_on_border) {
            GEO::index_t f = start_f;
            GEO::index_t lv = (start_lv+mesh.facets.nb_vertices(f)-1)%mesh.facets.nb_vertices(f);

            for (;;) {
                const GEO::index_t next_f = mesh.facets.adjacent(f, lv);
                if (next_f == GEO::NO_FACET)
                    break;
                f = next_f;
                lv = mesh.facets.find_vertex(f, v);
                ordered_f_lv.emplace_back(f, lv);
                lv = (lv+mesh.facets.nb_vertices(f)-1)%mesh.facets.nb_vertices(f);
            }

            /* [forward..., reversed inverse...] -> [inverse..., forward...] */
            std::reverse(ordered_f_lv.begin()+forward_nb, ordered_f_lv.end());
            std::rotate(ordered_f_lv.begin(), ordered_f_lv.begin()+forward_nb, ordered_f_lv.end());
        }

        return is_on_border;
    }

    bool get_vertex_incident_cells(
        const GEO::Mesh& mesh,
        const GEO::index_t start_c,
        const GEO::index_t start_lv,
        std::vector<std::pair<GEO::index_t, GEO::index_t>>& c_and_lv
        ) {
        assert(start_c < mesh.cells.nb());
        assert([&]() {
            switch (mesh.cells.type(start_c)) {
                case GEO::MeshCellType::MESH_TET:
                    if (start_lv >= 4)
                        return false;
                    break;
                case GEO::MeshCellType::MESH_HEX:
                    if (start_lv >= 8)
                        return false;
                    break;
                case GEO::MeshCellType::MESH_PRISM:
                    return false; // TODO: support
                    break;
                case GEO::MeshCellType::MESH_PYRAMID:
                    return false; // TODO: support
                    break;
                default: break;
            }
            return true;
        }());

        c_and_lv.clear();

        const auto v = mesh.cells.vertex(start_c, start_lv);

        std::unordered_set<GEO::index_t> processed_cells;
        bool is_on_border = false;

        std::stack<std::pair<GEO::index_t, GEO::index_t>> stack;
        stack.emplace(start_c, start_lv);
        while (!stack.empty()) {
            const auto [c, lv] = stack.top();
            stack.pop();

            if (!processed_cells.insert(c).second)
                continue;
            c_and_lv.emplace_back(c, lv);

            if (const auto& CELL_TYPE = mesh.cells.type(c);
                CELL_TYPE == GEO::MeshCellType::MESH_TET
                ) {
                for (const auto& lf : TET_LV_INCIDENT_LF[lv]) {
                    if (const auto nc = mesh.cells.adjacent(c, lf);
                        nc != GEO::NO_CELL
                        ) {
                        const auto nlv = mesh.cells.find_tet_vertex(nc, v);
                        assert(nlv != GEO::NO_INDEX);
                        stack.emplace(nc, nlv);
                    }
                    else
                        is_on_border = true;
                }
            }
            else {
                assert(CELL_TYPE == GEO::MeshCellType::MESH_HEX);

                for (const auto& lf : HEX_LV_INCIDENT_LF[lv]) {
                    if (const auto nc = mesh.cells.adjacent(c, lf);
                        nc != GEO::NO_CELL
                        ) {
                        const auto nlv = find_hex_vertex(mesh, nc, v);
                        assert(nlv != GEO::NO_INDEX);
                        stack.emplace(nc, nlv);
                    }
                    else
                        is_on_border = true;
                }
            }
        }

        return is_on_border;
    }

    bool get_edge_incident_cells(
        const GEO::Mesh& mesh,
        const GEO::index_t start_c,
        const GEO::index_t start_le,
        std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>>& ordered_c_le_lf
        ) {
        assert(start_c < mesh.cells.nb());
        assert([&]() {
            switch (mesh.cells.type(start_c)) {
                case GEO::MeshCellType::MESH_TET:
                    if (start_le >= 6)
                        return false;
                    break;
                case GEO::MeshCellType::MESH_HEX:
                    if (start_le >= 12)
                        return false;
                    break;
                case GEO::MeshCellType::MESH_PRISM:
                    [[fallthrough]];
                case GEO::MeshCellType::MESH_PYRAMID:
                    throw std::logic_error("Not support yet!");
                default:
                    return false;
            }
            return true;
        }());

        const auto ev0 = mesh.cells.edge_vertex(start_c, start_le, 0);
        const auto ev1 = mesh.cells.edge_vertex(start_c, start_le, 1);
        bool is_on_border = false;

        /* The output is filled in place: the forward chain is appended first, then (for border
         * edges) the backward chain, which is finally reversed and rotated to the front. Building
         * the two chains in local temporaries and swapping an empty vector into the output (as
         * this used to do) allocates and frees on every call, and it also throws away the
         * capacity of a caller-provided vector, so callers cannot reuse one across a loop. */
        ordered_c_le_lf.clear();
        if (ordered_c_le_lf.capacity() < 4)
            ordered_c_le_lf.reserve(4);

        /* Forward travel */
        {
            GEO::index_t c = start_c;
            GEO::index_t le = start_le;
            GEO::index_t lf = mesh.cells.edge_adjacent_facet(start_c, start_le, 0);
            for (;;) {
                ordered_c_le_lf.emplace_back(c, le, lf);

                const GEO::index_t nc = mesh.cells.adjacent(c, lf);
                if (nc == GEO::NO_CELL) {
                    is_on_border = true;
                    break;
                }
                if (nc == start_c) // a loop
                    break;

                /* Get next lf */
                if (mesh.cells.type(nc) == GEO::MeshCellType::MESH_TET)
                    le = find_tet_edge(mesh, nc, ev0, ev1);
                else if (mesh.cells.type(nc) == GEO::MeshCellType::MESH_HEX)
                    le = find_hex_edge(mesh, nc, ev0, ev1);
                else
                    assert(0); // TODO: support MESH_PRISM and MESH_PYRAMID
                assert(le != GEO::NO_INDEX);
                lf = mesh.cells.edge_adjacent_facet(nc, le, 0);
                if (mesh.cells.adjacent(nc, lf) == c)
                    lf = mesh.cells.edge_adjacent_facet(nc, le, 1);
                assert(mesh.cells.adjacent(nc, lf) != c);
                c = nc;
            }
        }
        const auto forward_nb = static_cast<std::ptrdiff_t>(ordered_c_le_lf.size());

        /* Inverse travel */
        if (is_on_border) {
            GEO::index_t c = start_c;
            GEO::index_t lf = mesh.cells.edge_adjacent_facet(start_c, start_le, 1);
            for (;;) {
                const GEO::index_t nc = mesh.cells.adjacent(c, lf);
                if (nc == GEO::NO_CELL)
                    break;

                /* Get next lf */
                GEO::index_t le;
                if (mesh.cells.type(nc) == GEO::MeshCellType::MESH_TET)
                    le = find_tet_edge(mesh, nc, ev0, ev1);
                else if (mesh.cells.type(nc) == GEO::MeshCellType::MESH_HEX)
                    le = find_hex_edge(mesh, nc, ev0, ev1);
                else
                    assert(0); // TODO: support MESH_PRISM and MESH_PYRAMID
                assert(le != GEO::NO_INDEX);
                lf = mesh.cells.edge_adjacent_facet(nc, le, 0);
                GEO::index_t lf1 = mesh.cells.edge_adjacent_facet(nc, le, 1);
                if (mesh.cells.adjacent(nc, lf) == c)
                    std::swap(lf, lf1);
                assert(mesh.cells.adjacent(nc, lf) != c && mesh.cells.adjacent(nc, lf1) == c);
                c = nc;

                ordered_c_le_lf.emplace_back(c, le, lf1);
            }

            /* [forward..., reversed inverse...] -> [inverse..., forward...] */
            std::reverse(ordered_c_le_lf.begin()+forward_nb, ordered_c_le_lf.end());
            std::rotate(ordered_c_le_lf.begin(), ordered_c_le_lf.begin()+forward_nb, ordered_c_le_lf.end());
        }

        return is_on_border;
    }

    bool get_edge_incident_cells(
        const GEO::Mesh& mesh,
        const GEO::index_t start_c,
        const GEO::index_t start_lf,
        const GEO::index_t start_lv,
        std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>>& ordered_c_le_lf
        ) {
        assert(start_c < mesh.cells.nb());

        if (mesh.cells.type(start_c) == GEO::MeshCellType::MESH_TET) {
            assert(start_lf < 4);
            assert(start_lv < 3);

            const auto ev0 = mesh.cells.facet_vertex(start_c, start_lf, start_lv);
            const auto ev1 = mesh.cells.facet_vertex(start_c, start_lf, (start_lv+1)%3);

            for (const auto& start_le : TET_LF_INCIDENT_LE[start_lf]) {
                const auto cev0 = mesh.cells.edge_vertex(start_c, start_le, 0);
                const auto cev1 = mesh.cells.edge_vertex(start_c, start_le, 1);
                if ((cev0 == ev0 && cev1 == ev1) ||
                    (cev0 == ev1 && cev1 == ev0))
                    return get_edge_incident_cells(mesh, start_c, start_le, ordered_c_le_lf);
            }
            assert(0);
        }
        else {
            assert(mesh.cells.type(start_c) == GEO::MeshCellType::MESH_HEX);
            assert(start_lf < 6);
            assert(start_lv < 4);

            const auto ev0 = mesh.cells.facet_vertex(start_c, start_lf, start_lv);
            const auto ev1 = mesh.cells.facet_vertex(start_c, start_lf, (start_lv+1)%4);

            for (const auto& start_le : HEX_LF_INCIDENT_LE[start_lf]) {
                const auto cev0 = mesh.cells.edge_vertex(start_c, start_le, 0);
                const auto cev1 = mesh.cells.edge_vertex(start_c, start_le, 1);
                if ((cev0 == ev0 && cev1 == ev1) ||
                    (cev0 == ev1 && cev1 == ev0))
                    return get_edge_incident_cells(mesh, start_c, start_le, ordered_c_le_lf);
            }
            assert(0);
        }
        assert(0);
        return false;
    }
}
