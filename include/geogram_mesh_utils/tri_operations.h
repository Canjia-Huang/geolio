//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/3/13.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOGRAM_MESH_UTILS_TRIANGLE_OPERATIONS_H
#define GEOGRAM_MESH_UTILS_TRIANGLE_OPERATIONS_H

#include <geogram/mesh/mesh.h>
#include <cassert>

namespace GEO::MeshUtils::Tri
{
    /**
     * @brief Collect triangles incident to a vertex in one-ring order.
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
    inline bool get_vertex_incident_triangles(
        const GEO::Mesh& M,
        const GEO::index_t _f,
        const GEO::index_t _lv,
        std::vector<std::pair<GEO::index_t, GEO::index_t>>& ordered_f_and_lv
        ) {
        assert(_f < M.facets.nb());
        assert(_lv < 3);

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
            GEO::index_t lv = (_lv+2)%3;

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
        ordered_f_and_lv.clear();
        ordered_f_and_lv.reserve(next_ordered_f_and_lv.size() + prev_ordered_f_and_lv.size());
        for (GEO::index_t i = 0, i_end = prev_ordered_f_and_lv.size(); i < i_end; ++i)
            ordered_f_and_lv.push_back(prev_ordered_f_and_lv[i_end-i-1]);
        for (const auto& f_lv : next_ordered_f_and_lv)
            ordered_f_and_lv.push_back(f_lv);

        return is_on_border;
    }

    /**
     * @brief Split an edge of a triangle in a mesh and update the adjacency topology accordingly.
     *
     * Given triangle facet @p f and local vertex index @p lv, a new vertex @p new_v is inserted
     * on the edge (lv → lv+1) at interpolation ratio @p r, splitting @p f into two triangles
     * (@p f and @p new_f1). If the adjacent facet across that edge exists (af != NO_FACET),
     * it is simultaneously split into two triangles (@p af and @p new_f2) to keep the mesh
     * topology consistent.
     *
     * @param[in, out] M The target mesh. Vertex and facet storage must be pre-allocated.
     * @param[in] f Index of the triangle facet to split.
     * @param[in] lv Local vertex index (0, 1, or 2) that identifies the edge to split (lv → lv+1).
     * @param[in] r Interpolation ratio in [0, 1] controlling where the new vertex is placed along the edge.
     * @param[in] new_v Index of the pre-allocated new vertex. Its position will be set to (1-r)*p(lv) + r*p(lv+1).
     * @param[in] new_f0 Index of the pre-allocated new facet produced by splitting @p f.
     * @param[in] new_f1 Index of the pre-allocated new facet produced by splitting the adjacent facet @p af.
     * Ignored when @p af does not exist (NO_FACET).
     */
    inline void facet_edge_split(
         GEO::Mesh& M,
        const GEO::index_t f,
        const GEO::index_t lv,
        const GEO::index_t new_v,
        const GEO::index_t new_f0,
        const GEO::index_t new_f1,
        const double r = 0.5
        ) {
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
         *  | /       |         | /newf0\ |
         *  v1 ------ v2        v1 ------ v2
         */
        const GEO::index_t lv0 = lv;
        const GEO::index_t lv1 = (lv+1)%3;
        const GEO::index_t lv2 = (lv+2)%3;
        // const GEO::index_t v0 = M.facets.vertex(f, lv0);
        const GEO::index_t v1 = M.facets.vertex(f, lv1);
        const GEO::index_t v2 = M.facets.vertex(f, lv2);
        const GEO::index_t nf0 = M.facets.adjacent(f, lv0);
        const GEO::index_t nf1 = M.facets.adjacent(f, lv1);

        /* Set new point */
        const auto& p0 = M.facets.point(f, lv0);
        const auto& p1 = M.facets.point(f, lv1);
        M.vertices.point(new_v) = (1-r)*p0 + r*p1;

        /* Set facet vertices  */
        M.facets.set_vertex(f, lv1, new_v);
        M.facets.set_vertex(new_f0, lv0, new_v);
        M.facets.set_vertex(new_f0, lv1, v1);
        M.facets.set_vertex(new_f0, lv2, v2);

        /* Set facet adjacency */
        M.facets.set_adjacent(f, lv1, new_f0);
        assert(M.facets.adjacent(new_f0, lv0) == GEO::NO_FACET); // will set later
        M.facets.set_adjacent(new_f0, lv1, nf1);
        M.facets.set_adjacent(new_f0, lv2, f);
        if (nf1 != GEO::NO_FACET) {
            assert(M.facets.find_vertex(nf1, v2) != GEO::NO_INDEX);
            M.facets.set_adjacent(nf1, M.facets.find_vertex(nf1, v2), new_f0);
        }

        /* == Split adjacent facet ================================================================================= */
        if (nf0 != GEO::NO_FACET) {
            assert(new_f1 < M.facets.nb());

            /*
             * nv2 ----- nv1       nv2 ----- nv1
             *  |       / |         | \ nf0 / |
             *  | nf0 /   |    ->   |new\ /   |
             *  |   /     |         |f1 / \   |
             *  | /       |         | /     \ |
             * nv0 -------+        nv0 -------+
             */
            const GEO::index_t nlv0 = M.facets.find_vertex(nf0, v1);
            assert(nlv0 != GEO::NO_INDEX);
            const GEO::index_t nlv1 = (nlv0+1)%3;
            const GEO::index_t nlv2 = (nlv0+2)%3;
            const GEO::index_t nv0 = M.facets.vertex(nf0, nlv0);
            // const GEO::index_t nv1 = M.facets.vertex(af0, nlv1);
            const GEO::index_t nv2 = M.facets.vertex(nf0, nlv2);
            const GEO::index_t nnf2 = M.facets.adjacent(nf0, nlv2);

            /* Set facet vertices */
            M.facets.set_vertex(nf0, nlv0, new_v);
            M.facets.set_vertex(new_f1, nlv0, nv0);
            M.facets.set_vertex(new_f1, nlv1, new_v);
            M.facets.set_vertex(new_f1, nlv2, nv2);

            /* Set facet adjacency */
            M.facets.set_adjacent(new_f0, lv0, new_f1);
            M.facets.set_adjacent(nf0, nlv2, new_f1);
            M.facets.set_adjacent(new_f1, nlv0, new_f0);
            M.facets.set_adjacent(new_f1, nlv1, nf0);
            M.facets.set_adjacent(new_f1, nlv2, nnf2);
            if (nnf2 != GEO::NO_FACET) {
                assert(M.facets.find_vertex(new_f1, nv0) != GEO::NO_INDEX);
                M.facets.set_adjacent(nnf2, M.facets.find_vertex(nnf2, nv0), new_f1);
            }
        }
    }

    /**
     * @brief Collapse an edge of a triangle and update local connectivity.
     *
     * Given facet @p f and local vertex index @p lv, this function collapses edge (lv -> lv+1)
     * by moving vertex v(lv) to (1-r)*p(lv) + r*p(lv+1), then merging v(lv+1) into v(lv).
     * The two incident triangles on that edge become unused: @p f and, if it exists, the adjacent
     * facet across local edge @p lv.
     *
     * @param[in, out] M The target mesh topology/geometry to update.
     * @param[in] f Index of the triangle facet that owns the collapsed edge.
     * @param[in] lv Local vertex index (0, 1, or 2) identifying the edge to collapse (lv -> lv+1).
     * @param[in] r Interpolation ratio in [0, 1] for the new position of v(lv).
     * @param[out] disuse_v Index of the merged-away vertex (original v(lv+1)).
     * @param[out] disuse_f0 Index of the first unused facet after collapse (always @p f).
     * @param[out] disuse_f1 Index of the second unused facet across the collapsed edge;
     *                       set to GEO::NO_FACET when the edge is on the border.
     */
    inline void facet_edge_collapse(
        GEO::Mesh& M,
        const GEO::index_t f,
        const GEO::index_t lv,
        GEO::index_t& disuse_v,
        GEO::index_t& disuse_f0,
        GEO::index_t& disuse_f1,
        const double r
        ) {
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
        get_vertex_incident_triangles(M, f, lv1, ordered_f_and_lv);

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

    /**
     * @brief Swap an interior edge shared by two triangles.
     *
     * For facet @p f and local edge @p lv, this operation replaces the shared diagonal of the
     * two incident triangles with the other diagonal of the local quadrilateral. The two facet
     * indices are kept unchanged, while their vertex connectivity and adjacency links are updated
     * in-place.
     *
     * @param[in,out] M Target triangle mesh whose facet connectivity and adjacency are modified.
     * @param[in] f Index of one incident facet of the edge to flip.
     * @param[in] lv Local edge index (0, 1, or 2) in facet @p f identifying the edge opposite
     *               local vertex @p lv.
     * @return true if the swap is performed successfully; false if the target edge is on the border
     *         or the operation cannot be applied.
     */
    inline bool facet_edge_swap(
        GEO::Mesh& M,
        const GEO::index_t f,
        const GEO::index_t lv
        ) {
        assert(f < M.facets.nb());
        assert(lv < 3);

        const GEO::index_t af = M.facets.adjacent(f, lv);
        if (af == GEO::NO_FACET)
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
        const GEO::index_t v1 = M.facets.vertex(f, lv1);
        const GEO::index_t v2 = M.facets.vertex(f, lv2);

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

        return true;
    }
}

#endif //GEOGRAM_MESH_UTILS_TRIANGLE_OPERATIONS_H

