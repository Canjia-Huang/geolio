//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/6/25.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "tri_operations.h"
#include <cassert>
#include <unordered_set>
#include <utility>
#include <vector>
#include <geogram/mesh/mesh_io.h>
#include <geolio/common/pair_hash.h>
#include <geolio/common/vecg.h>
#include "mesh_operations.h"

namespace geolio
{
    template <GEO::index_t DIM>
    void tri_edge_split(
        GEO::Mesh& mesh,
        const GEO::index_t f,
        const GEO::index_t lv,
        const GEO::index_t new_v,
        const GEO::index_t new_f0,
        const GEO::index_t new_f1,
        const bool update_attributes
        ) {
        assert(f < mesh.facets.nb());
        assert(mesh.facets.nb_vertices(f) == 3);
        assert(lv < 3);
        assert(new_v < mesh.vertices.nb());
        assert(new_f0 < mesh.facets.nb());

        /*
         *  +-------- v0        +-------- v0
         *  |       / |         | \     / |
         *  |     /   |    ->   |   \ /   |
         *  |   /  f  |         |   / \ f |
         *  | /       |         | /newf0\ |
         *  v1 ------ v2        v1 ------ v2
         */
        const GEO::index_t lv0 = lv;
        const GEO::index_t lv1 = (lv+1)%3;
        const GEO::index_t lv2 = (lv+2)%3;
        // const GEO::index_t v0 = M.facets.vertex(f, lv0);
        const GEO::index_t v1 = mesh.facets.vertex(f, lv1);
        const GEO::index_t v2 = mesh.facets.vertex(f, lv2);
        const GEO::index_t nf0 = mesh.facets.adjacent(f, lv0);
        const GEO::index_t nf1 = mesh.facets.adjacent(f, lv1);

        /* Set new point */
        const auto& p0 = mesh.facets.point<DIM>(f, lv0);
        const auto& p1 = mesh.facets.point<DIM>(f, lv1);
        mesh.vertices.point<DIM>(new_v) = 0.5 * (p0 + p1);

        /* Set facet vertices  */
        mesh.facets.set_vertex(f, lv1, new_v);
        mesh.facets.set_vertex(new_f0, lv0, new_v);
        mesh.facets.set_vertex(new_f0, lv1, v1);
        mesh.facets.set_vertex(new_f0, lv2, v2);

        /* Set facet adjacency */
        mesh.facets.set_adjacent(f, lv1, new_f0);
        mesh.facets.set_adjacent(new_f0, lv1, nf1);
        mesh.facets.set_adjacent(new_f0, lv2, f);
        if (nf1 != GEO::NO_FACET) {
            assert(mesh.facets.find_vertex(nf1, v2) != GEO::NO_INDEX);
            mesh.facets.set_adjacent(nf1, mesh.facets.find_vertex(nf1, v2), new_f0);
        }

        if (update_attributes) {
            /* Facet */
            mesh.facets.attributes().copy_item(new_f0, f);

            /* Facet corners */
            mesh.facet_corners.attributes().zero_item(mesh.facets.corner(new_f0, lv0));
            mesh.facet_corners.attributes().copy_item(mesh.facets.corner(new_f0, lv1), mesh.facets.corner(f, lv1));
            mesh.facet_corners.attributes().copy_item(mesh.facets.corner(new_f0, lv2), mesh.facets.corner(f, lv2));
            mesh.facet_corners.attributes().zero_item(mesh.facets.corner(f, lv1));
        }

        /* == Split adjacent facet ================================================================================= */
        if (nf0 != GEO::NO_FACET) {
            assert(new_f1 < mesh.facets.nb());
            assert(mesh.facets.nb_vertices(new_f1) == 3);

            /*
             * nv2 ----- nv1       nv2 ----- nv1
             *  |       / |         | \ nf0 / |
             *  | nf0 /   |    ->   |new\ /   |
             *  |   /     |         |f1 / \   |
             *  | /       |         | /     \ |
             * nv0 -------+        nv0 -------+
             */
            const GEO::index_t nlv0 = mesh.facets.find_vertex(nf0, v1);
            assert(nlv0 != GEO::NO_INDEX);
            const GEO::index_t nlv1 = (nlv0+1)%3;
            const GEO::index_t nlv2 = (nlv0+2)%3;
            const GEO::index_t nv0 = mesh.facets.vertex(nf0, nlv0);
            // const GEO::index_t nv1 = M.facets.vertex(af0, nlv1);
            const GEO::index_t nv2 = mesh.facets.vertex(nf0, nlv2);
            const GEO::index_t nnf2 = mesh.facets.adjacent(nf0, nlv2);

            /* Set facet vertices */
            mesh.facets.set_vertex(nf0, nlv0, new_v);
            mesh.facets.set_vertex(new_f1, nlv0, nv0);
            mesh.facets.set_vertex(new_f1, nlv1, new_v);
            mesh.facets.set_vertex(new_f1, nlv2, nv2);

            /* Set facet adjacency */
            mesh.facets.set_adjacent(new_f0, lv0, new_f1);
            mesh.facets.set_adjacent(nf0, nlv2, new_f1);
            mesh.facets.set_adjacent(new_f1, nlv0, new_f0);
            mesh.facets.set_adjacent(new_f1, nlv1, nf0);
            mesh.facets.set_adjacent(new_f1, nlv2, nnf2);
            if (nnf2 != GEO::NO_FACET) {
                assert(mesh.facets.find_vertex(new_f1, nv0) != GEO::NO_INDEX);
                mesh.facets.set_adjacent(nnf2, mesh.facets.find_vertex(nnf2, nv0), new_f1);
            }

            if (update_attributes) {
                /* Facets */
                mesh.facets.attributes().copy_item(new_f1, nf0);

                /* Facet corners */
                mesh.facet_corners.attributes().copy_item(mesh.facets.corner(new_f1, nlv0), mesh.facets.corner(nf0, nlv0));
                mesh.facet_corners.attributes().zero_item(mesh.facets.corner(new_f1, nlv1));
                mesh.facet_corners.attributes().copy_item(mesh.facets.corner(new_f1, nlv2), mesh.facets.corner(nf0, nlv2));
                mesh.facet_corners.attributes().zero_item(mesh.facets.corner(nf0, nlv0));
            }
        }
        else
            mesh.facets.set_adjacent(new_f0, lv0, GEO::NO_FACET);
    }

    template void tri_edge_split<2>(
        GEO::Mesh& mesh, GEO::index_t f, GEO::index_t lv, GEO::index_t new_v, GEO::index_t new_f0, GEO::index_t new_f1, bool update_attributes);
    template void tri_edge_split<3>(
        GEO::Mesh& mesh, GEO::index_t f, GEO::index_t lv, GEO::index_t new_v, GEO::index_t new_f0, GEO::index_t new_f1, bool update_attributes);

    bool is_tri_edge_collapse_valid(
        const GEO::Mesh& mesh,
        const GEO::index_t f,
        const GEO::index_t lv
        ) {
        assert(f < mesh.facets.nb());
        assert(mesh.facets.nb_vertices(f) == 3);
        assert(lv < 3);

        const GEO::index_t lv0 = lv;
        const GEO::index_t lv1 = (lv+1)%3;
        // const GEO::index_t lv2 = (lv+2)%3;
        const GEO::index_t v0 = mesh.facets.vertex(f, lv0);
        const GEO::index_t v1 = mesh.facets.vertex(f, lv1);
        const auto af = mesh.facets.adjacent(f, lv);

        /* Find all incident facets */
        std::vector<std::pair<GEO::index_t, GEO::index_t>> v0_ordered_f_and_lv;
        const bool v0_on_boundary = get_vertex_incident_facets(mesh, f, lv0, v0_ordered_f_and_lv);

        std::vector<std::pair<GEO::index_t, GEO::index_t>> v1_ordered_f_and_lv;
        const bool v1_on_boundary = get_vertex_incident_facets(mesh, f, lv1, v1_ordered_f_and_lv);

        if (v0_on_boundary && v1_on_boundary && af != GEO::NO_FACET) // will create a new non-manifold vertex
            return false;

        /* After collapse, no identical triangles can exist */
        std::unordered_set<std::pair<GEO::index_t, GEO::index_t>, PairHash> other_vertices_pair;
        for (const auto& [nf, nlv] : v0_ordered_f_and_lv) {
            const auto& nv1 = mesh.facets.vertex(nf, (nlv+1)%3);
            const auto& nv2 = mesh.facets.vertex(nf, (nlv+2)%3);
            assert(nv1 != v0);
            assert(nv2 != v0);
            if (nv1 == nv2) // Exist degenerate adjacent facet.
                return false;
            if (nv1 == v1 || nv2 == v1) {
                if (nf != f && nf != af) // Non-manifold edge.
                    return false;
            }
            if (const std::pair<GEO::index_t, GEO::index_t> other_vertices = std::minmax(nv1, nv2);
                !other_vertices_pair.insert(other_vertices).second) // Identical triangles in an adjacent face group.
                return false;
        }
        for (const auto& [nf, nlv] : v1_ordered_f_and_lv) {
            const auto& nv1 = mesh.facets.vertex(nf, (nlv+1)%3);
            const auto& nv2 = mesh.facets.vertex(nf, (nlv+2)%3);
            assert(nv1 != v1);
            assert(nv2 != v1);
            if (nv1 == nv2) // Exist degenerate adjacent facet.
                return false;
            if (nv1 == v0 || nv2 == v0) {
                if (nf != f && nf != af) // Non-manifold edge.
                    return false;
            }
            if (const std::pair<GEO::index_t, GEO::index_t> other_vertices = std::minmax(nv1, nv2);
                !other_vertices_pair.insert(other_vertices).second) // Identical triangles in an adjacent face group or two adjacent face group.
                return false;
        }

        return true;
    }

    template <GEO::index_t DIM>
    void tri_edge_collapse(
        GEO::Mesh& mesh,
        const GEO::index_t f,
        const GEO::index_t lv,
        GEO::index_t& disuse_v,
        GEO::index_t& disuse_f0,
        GEO::index_t& disuse_f1
        ) {
        assert(f < mesh.facets.nb());
        assert(mesh.facets.nb_vertices(f) == 3);
        assert(lv < 3);

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
        const GEO::index_t v0 = mesh.facets.vertex(f, lv0);
        const GEO::index_t v1 = mesh.facets.vertex(f, lv1);
        const GEO::index_t v2 = mesh.facets.vertex(f, lv2);
        const GEO::index_t af0 = mesh.facets.adjacent(f, lv0);
        const GEO::index_t af1 = mesh.facets.adjacent(f, lv1);
        const GEO::index_t af2 = mesh.facets.adjacent(f, lv2);

        /* Set collapsed point (v0) */
        const auto& p0 = mesh.vertices.point<DIM>(v0);
        const auto& p1 = mesh.vertices.point<DIM>(v1);
        mesh.vertices.point<DIM>(v0) = 0.5 * (p0 + p1);
        disuse_v = v1;
        disuse_f0 = f;
        disuse_f1 = af0; // facet or GEO::NO_FACET

        /* Find all (f, lv) that incident to v1 (before performing collapse) */
        std::vector<std::pair<GEO::index_t, GEO::index_t>> ordered_f_and_lv;
        get_vertex_incident_facets(mesh, f, lv1, ordered_f_and_lv);

        /* Set facet adjacency */
        if (af1 != GEO::NO_FACET) {
            assert(mesh.facets.find_vertex(af1, v2) != GEO::NO_INDEX);
            mesh.facets.set_adjacent(af1, mesh.facets.find_vertex(af1, v2), af2);
        }
        if (af2 != GEO::NO_FACET) {
            assert(mesh.facets.find_vertex(af2, v0) != GEO::NO_INDEX);
            mesh.facets.set_adjacent(af2, mesh.facets.find_vertex(af2, v0), af1);
        }

        /* == Collapse adjacent facet ============================================================================== */
        if (af0 != GEO::NO_FACET) {
            assert(mesh.facets.nb_vertices(af0) == 3);

            /*
             *  +-------- nv1                 ++
             *    \ naf1 / |                 / |
             *      \  /   |              /naf1|
             *      nv2 af0|     ->    nv2 --- nv1
             *      /  \   |              \naf2|
             *    / naf2 \ |                \  |
             *  +-------- nv0                 ++
             */
            const GEO::index_t nlv0 = mesh.facets.find_vertex(af0, v1);
            assert(nlv0 != GEO::NO_INDEX);
            const GEO::index_t nlv1 = (nlv0+1)%3;
            const GEO::index_t nlv2 = (nlv0+2)%3;
            const GEO::index_t nv0 = mesh.facets.vertex(af0, nlv0);
            // const GEO::index_t nv1 = M.facets.vertex(af0, nlv1);
            const GEO::index_t nv2 = mesh.facets.vertex(af0, nlv2);
            const GEO::index_t naf1 = mesh.facets.adjacent(af0, nlv1);
            const GEO::index_t naf2 = mesh.facets.adjacent(af0, nlv2);

            /* Set facet adjacency */
            if (naf1 != GEO::NO_FACET) {
                assert(mesh.facets.find_vertex(naf1, nv2) != GEO::NO_INDEX);
                mesh.facets.set_adjacent(naf1, mesh.facets.find_vertex(naf1, nv2), naf2);
            }
            if (naf2 != GEO::NO_FACET) {
                assert(mesh.facets.find_vertex(naf2, nv0) != GEO::NO_INDEX);
                mesh.facets.set_adjacent(naf2, mesh.facets.find_vertex(naf2, nv0), naf1);
            }
        }

        /* Set facet vertices */
        for (const auto& [adj_f, adj_lv] : ordered_f_and_lv)
            mesh.facets.set_vertex(adj_f, adj_lv, v0);
    }

    template void tri_edge_collapse<2>(
        GEO::Mesh& mesh, GEO::index_t f, GEO::index_t lv, GEO::index_t& disuse_v, GEO::index_t& disuse_f0,
        GEO::index_t& disuse_f1);
    template void tri_edge_collapse<3>(
        GEO::Mesh& mesh, GEO::index_t f, GEO::index_t lv, GEO::index_t& disuse_v, GEO::index_t& disuse_f0,
        GEO::index_t& disuse_f1);

    bool is_tri_edge_swap_valid(
        const GEO::Mesh& mesh,
        const GEO::index_t f,
        const GEO::index_t lv
        ) {
        assert(f < mesh.facets.nb());
        assert(mesh.facets.nb_vertices(f) == 3);
        assert(lv < 3);

        const GEO::index_t af = mesh.facets.adjacent(f, lv);
        if (af == GEO::NO_FACET)
            return false;

        const GEO::index_t lv1 = (lv+1)%3;
        const GEO::index_t lv2 = (lv+2)%3;
        const GEO::index_t v1 = mesh.facets.vertex(f, lv1);

        const GEO::index_t nlv0 = mesh.facets.find_vertex(af, v1);
        assert(nlv0 != GEO::NO_INDEX);
        const GEO::index_t v3 = mesh.facets.vertex(af, (nlv0+2)%3);

        /* The flip replaces the diagonal (v1, v2) by the other diagonal (v2, v3), v2 being the vertex
         * opposite to lv2. The new diagonal must not be an edge of the mesh yet: the two flipped
         * facets would otherwise share it with the facets that already own it, and the mesh would
         * stop being 2-manifold. Every facet that may own that edge contains v2, so walking the
         * one-ring of v2 is enough. */
        std::vector<std::pair<GEO::index_t, GEO::index_t>> v2_incident_f_lv;
        get_vertex_incident_facets(mesh, f, lv2, v2_incident_f_lv);
        for (const auto& [nf, nlv] : v2_incident_f_lv) {
            if (mesh.facets.find_vertex(nf, v3) != GEO::NO_INDEX)
                return false;
        }

        return true;
    }

    bool tri_edge_swap(
        GEO::Mesh& mesh,
        const GEO::index_t f,
        const GEO::index_t lv,
        const bool update_attributes
        ) {
        assert(f < mesh.facets.nb());
        assert(mesh.facets.nb_vertices(f) == 3);
        assert(lv < 3);

        const GEO::index_t af = mesh.facets.adjacent(f, lv);
        if (af == GEO::NO_FACET)
            return false;

        /* Refuse a flip that would leave the mesh with an inconsistent topology. */
        if (!is_tri_edge_swap_valid(mesh, f, lv))
            return false;

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
        const GEO::index_t v1 = mesh.facets.vertex(f, lv1);
        const GEO::index_t v2 = mesh.facets.vertex(f, lv2);

        const GEO::index_t af1 = mesh.facets.adjacent(f, lv1);
        // const GEO::index_t af2 = M.facets.adjacent(f, lv2);

        const GEO::index_t nlv0 = mesh.facets.find_vertex(af, v1);
        assert(nlv0 != GEO::NO_INDEX);
        const GEO::index_t nlv1 = (nlv0+1)%3;
        const GEO::index_t nlv2 = (nlv0+2)%3;
        const GEO::index_t v3 = mesh.facets.vertex(af, nlv2);

        const GEO::index_t af0 = mesh.facets.adjacent(af, nlv1);
        // const GEO::index_t af3 = M.facets.adjacent(af, nlv2);

        /* Set vertices */
        mesh.facets.set_vertex(f, lv1, v3);
        mesh.facets.set_vertex(af, nlv1, v2);

        /* Set adjacency */
        mesh.facets.set_adjacent(f, lv, af0);
        mesh.facets.set_adjacent(f, lv1, af);
        mesh.facets.set_adjacent(af, nlv0, af1);
        mesh.facets.set_adjacent(af, nlv1, f);
        if (af0 != GEO::NO_FACET) {
            assert(mesh.facets.find_vertex(af0, v3) != GEO::NO_INDEX);
            mesh.facets.set_adjacent(af0, mesh.facets.find_vertex(af0, v3), f);
        }
        if (af1 != GEO::NO_FACET) {
            assert(mesh.facets.find_vertex(af1, v2) != GEO::NO_INDEX);
            mesh.facets.set_adjacent(af1, mesh.facets.find_vertex(af1, v2), af);
        }

        if (update_attributes) {
            /* Facets */
            mesh.facets.attributes().zero_item(f);
            mesh.facets.attributes().zero_item(af);

            /* Facet corners */
            mesh.facet_corners.attributes().copy_item(mesh.facets.corner(f, lv1), mesh.facets.corner(af, nlv2));
            mesh.facet_corners.attributes().copy_item(mesh.facets.corner(af, nlv1), mesh.facets.corner(f, lv2));
            mesh.facet_corners.attributes().zero_item(mesh.facets.corner(f, lv));
            mesh.facet_corners.attributes().zero_item(mesh.facets.corner(af, nlv0));
        }

        return true;
    }
}
