//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/5.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "quad_motorcycle_graph.h"
#include <cassert>
#include <geolio/common/utils.h>

#include "geolio/mesh/mesh_operations.h"

namespace
{
    /* Upper bound on the number of artificial cuts performed before giving up. Every cut adds a
     * wall inside a loop block, so only a few are needed in practice. */
    constexpr GEO::index_t MAX_CUT_ATTEMPTS = 16;

    /* Tag value of the artificial walls created by a cut. Outside of the burning, tags are only
     * interpreted as "is this edge a wall?", and 0 is also the value used for border edges. */
    constexpr GEO::index_t SEAM_TAG = 0;
}

namespace geolio
{
    QuadMotorCycleGraph::QuadMotorCycleGraph(
        const GEO::Mesh& mesh
        ) : attribute_id_(geolio::generate_random_string(22)),
            mesh_(mesh)
    {
        assert([&]() {
            for (const auto& f : mesh_.facets) {
                if (mesh_.facets.nb_vertices(f) != 4)
                    return false;
            }
            return true;
        }()); // check all-quad mesh

        mesh_fc_tagged_.bind(mesh_.facet_corners.attributes(), attribute_id_+":tagged");
        mesh_fc_tagged_.fill(GEO::NO_INDEX);

        find_all_singular_and_border_vertices();
    }

    QuadMotorCycleGraph::~QuadMotorCycleGraph(
        ) {
        if (mesh_fc_tagged_.is_bound())
            mesh_fc_tagged_.destroy();
        if (mesh_v_singular_.is_bound())
            mesh_v_singular_.destroy();
        if (mesh_v_border_.is_bound())
            mesh_v_border_.destroy();
    }

    GEO::index_t QuadMotorCycleGraph::compute(
        const MotorCycleType complex_type
        ) {
        std::priority_queue<Fire> queue;

        /* Ignite */
        ignite(queue);

        /* Scratch buffer for the facets around a vertex; reused across iterations so that the
         * traversal does not allocate on every call. */
        std::vector<std::pair<GEO::index_t, GEO::index_t>> ordered_f_lv;

        /* Burning */
        while (!queue.empty()) {
            const auto& fire = queue.top();
            const auto F_d = fire.d;
            const auto F_f = fire.f;
            const auto F_lv = fire.lv;
            queue.pop();

            /* Already burnt: the same edge was already reached by an earlier (closer) fire.
             * An edge only tags itself when it is popped, so an edge pushed by several of its
             * neighbours is queued several times. Re-processing it burns nothing new but
             * re-propagates the fire, which makes the queue never settle. */
            if (mesh_fc_tagged_[mesh_.facets.corner(F_f, F_lv)] != GEO::NO_INDEX)
                continue;

            /* Alive */
            bool alive = false;
            if (complex_type == BASE_COMPLEX)
                alive = true;
            else {
                assert(complex_type == MOTORCYCLE_COMPLEX);
                if (mesh_v_singular_[mesh_.facets.vertex(F_f, F_lv)])
                    alive = true;
                else {
                    /* Find all incident edges */
                    if (get_vertex_incident_facets(mesh_, F_f, F_lv, ordered_f_lv)) { // append the preceding border edge
                        // Copy the pair instead of binding a reference to it: emplace_back() may
                        // reallocate the vector and would read the referenced element from the
                        // freed buffer.
                        const auto [f, lv] = ordered_f_lv[0];
                        ordered_f_lv.emplace_back(f, (lv+3)%4);
                    }

                    GEO::index_t tagged_edges_nb = 0;
                    for (const auto& [f, lv] : ordered_f_lv) {
                        if (mesh_fc_tagged_[mesh_.facets.corner(f, lv)] != GEO::NO_INDEX)
                            ++tagged_edges_nb;
                    }

                    if (tagged_edges_nb < 3)
                        alive = true;
                }
            }
            if (!alive)
                continue;

            /* Mark edge as burnt (both sides) */
            tag_edge(F_f, F_lv, F_d);

            /* Burning */
            const GEO::index_t F_lv1 = (F_lv+1)%4;
            if (const auto& v = mesh_.facets.vertex(F_f, F_lv1);
                !mesh_v_singular_[v] && !mesh_v_border_[v]
                ) {
                /* Find all incident edges */
                const auto on_border = get_vertex_incident_facets(mesh_, F_f, F_lv1, ordered_f_lv);
                assert(!on_border);
                assert(ordered_f_lv[0].first == F_f);
                assert(ordered_f_lv.size() == 4); // regular

                if (const auto& [opp_f, opp_lv] = ordered_f_lv[1];
                    mesh_fc_tagged_[mesh_.facets.corner(opp_f, opp_lv)] == GEO::NO_INDEX
                    ) {
                    assert(mesh_.facets.vertex(F_f, F_lv1) == mesh_.facets.vertex(opp_f, opp_lv));
                    assert(mesh_.facets.adjacent(F_f, F_lv) != opp_f);

                    Fire new_F{};
                    new_F.d = F_d+1;
                    new_F.f = opp_f;
                    new_F.lv = opp_lv;

                    queue.push(new_F);
                }
            }
        }

        /* Label border edge */
        for (const auto& f : mesh_.facets) {
            for (GEO::index_t lv = 0; lv < 4; ++lv) {
                if (mesh_.facets.adjacent(f, lv) == GEO::NO_FACET)
                    mesh_fc_tagged_[mesh_.facets.corner(f, lv)] = 0;
            }
        }
        // DEBUG
        if constexpr (false) {
            mesh_.save("debug.geogram");
            throw std::logic_error("im here");
        }

        /* Decompose */
        GEO::index_t blocks_nb = decompose_into_blocks();

        /* A block wraps around when the region it covers has no singular vertex at all (e.g. an
         * annulus, or a torus). Such a block cannot be unrolled into a cuboid: its layout is not
         * dense and a coarse quad built from it would be degenerate. Cut every such block open
         * along grid lines (artificial motorcycles) and decompose again until all blocks are
         * valid cuboids.
         *
         * NOTE: two cuts are enough to turn an open loop (an annulus) into cuboids. A closed loop
         * (a torus) is split into cuboids as well, but its coarse quads then share corner
         * vertices, so the coarse mesh is not a valid quad mesh yet: it needs more cuts, which
         * would be driven by the coarse edge incidence. */
        for (GEO::index_t attempt = 0; ; ++attempt) {
            GEO::index_t block_id = GEO::NO_INDEX;
            if (!find_invalid_block(block_id))
                break;

            if (attempt >= MAX_CUT_ATTEMPTS)
                throw std::logic_error(
                    "QuadMotorCycleGraph: a loop block could not be cut into cuboids. "
                    "The block may not be a simple loop, or cutting it needs more cuts.");

            assert(block_id < blocks_.size());
            if (!cut_block(blocks_[block_id], block_id))
                throw std::logic_error(
                    "QuadMotorCycleGraph: no grid line can cut a loop block open.");

            blocks_nb = decompose_into_blocks();
        }

        return blocks_nb;
    }

    void QuadMotorCycleGraph::label_blocks(
        GEO::Attribute<GEO::index_t>& mesh_f_block
        ) const {
        assert(mesh_f_block.is_bound());
        assert(mesh_f_block.size() == mesh_.facets.nb());
        if (blocks_.empty())
            throw std::logic_error("Need to call compute() first!");

        for (GEO::index_t i = 0, i_end = blocks_.size(); i < i_end; ++i) {
            const auto& block = blocks_[i];
            for (const auto& bfs = block.block_facets();
                const auto& bf : bfs)
                mesh_f_block[bf.f] = i;
        }
    }

    void QuadMotorCycleGraph::create_coarse_mesh(
        GEO::Mesh& mesh_out,
        std::vector<GEO::index_t>* old_fc_to_new_fc
        ) const {
        if (blocks_.empty())
            throw std::logic_error("Need to call compute() first!");

        mesh_out.clear(false);
        mesh_out.copy(mesh_, false, GEO::MESH_VERTICES);

        GEO::index_t new_f = mesh_out.facets.create_quads(blocks_.size());
        for (const auto& block : blocks_) {
            for (GEO::index_t lv = 0; lv < 4; ++lv)
                mesh_out.facets.set_vertex(new_f, lv, block.facet_corner_vertex(lv));
            ++new_f;
        }
        mesh_out.facets.connect();

        if (old_fc_to_new_fc != nullptr) {
            old_fc_to_new_fc->assign(mesh_.facet_corners.nb(), GEO::NO_INDEX);
            assert(mesh_out.facets.nb() == blocks_.size());
            for (const auto& f : mesh_out.facets) {
                for (const auto& BF : blocks_[f].block_facets()) {
                    for (GEO::index_t lv = 0; lv < 4; ++lv) {
                        if (const auto old_cf = mesh_.facets.corner(BF.f, (BF.lv+lv)%4);
                            mesh_fc_tagged_[old_cf] != GEO::NO_INDEX
                            )
                            (*old_fc_to_new_fc)[old_cf] = mesh_out.facets.corner(f, lv);
                    }
                }
            }
        }
    }

    void QuadMotorCycleGraph::find_all_singular_and_border_vertices(
        ) {
        if (!mesh_v_singular_.is_bound())
            mesh_v_singular_.bind(mesh_.vertices.attributes(), attribute_id_+":singular");
        mesh_v_singular_.fill(false);
        if (!mesh_v_border_.is_bound())
            mesh_v_border_.bind(mesh_.vertices.attributes(), attribute_id_+":border");
        mesh_v_border_.fill(false);

        std::vector<GEO::index_t> v_incident_facets_nb(mesh_.vertices.nb(), 0);
        for (const auto& f : mesh_.facets) {
            for (GEO::index_t lv = 0; lv < 4; ++lv) {
                ++v_incident_facets_nb[mesh_.facets.vertex(f, lv)];

                if (mesh_.facets.adjacent(f, lv) == GEO::NO_FACET) {
                    mesh_v_border_[mesh_.facets.vertex(f, lv)] = true;
                    mesh_v_border_[mesh_.facets.vertex(f, (lv+1)%4)] = true;
                }
            }
        }

        /* Label singular vertex */
        for (const auto& v : mesh_.vertices) {
            if (mesh_v_border_[v]) {
                if (v_incident_facets_nb[v] != 2)
                    mesh_v_singular_[v] = true;
            }
            else {
                if (v_incident_facets_nb[v] != 4)
                    mesh_v_singular_[v] = true;
            }
        }
    }

    void QuadMotorCycleGraph::ignite(
        std::priority_queue<Fire>& queue
        ) const {
        assert(mesh_v_singular_.size() == mesh_.vertices.nb());
        assert(queue.empty());

        while (!queue.empty())
            queue.pop();

        std::vector<bool> processed_vertices(mesh_.vertices.nb(), false);
        std::vector<std::pair<GEO::index_t, GEO::index_t>> ordered_f_lv; // reused, see compute()
        for (const auto& f : mesh_.facets) {
            for (GEO::index_t lv = 0; lv < 4; ++lv) {
                const auto& v = mesh_.facets.vertex(f, lv);
                if (processed_vertices[v] || !mesh_v_singular_[v]) // for all singular vertex
                    continue;

                processed_vertices[v] = true;

                /* Find all incident interior edges */
                get_vertex_incident_facets(mesh_, f, lv, ordered_f_lv);
                for (const auto& [adj_f, adj_lv] : ordered_f_lv) {
                    if (mesh_.facets.adjacent(adj_f, adj_lv) == GEO::NO_FACET)
                        continue;

                    Fire F{};
                    F.d = 0;
                    F.f = adj_f;
                    F.lv = adj_lv;

                    queue.push(F);
                }
            }
        }
    }

    GEO::index_t QuadMotorCycleGraph::decompose_into_blocks(
        ) {
        blocks_.clear();
        mesh_f_block_.assign(mesh_.facets.nb(), GEO::NO_INDEX);

        std::vector<bool> processed_facets(mesh_.facets.nb(), false);
        for (const auto& start_f : mesh_.facets) {
            if (processed_facets[start_f]) // labelled
                continue;

            /* Build motorcycle block */
            QuadMotorCycleBlock MC_block(mesh_, mesh_fc_tagged_);
            MC_block.flood_fill_facets(start_f);

            const auto block_id = static_cast<GEO::index_t>(blocks_.size());
            for (const auto& b_facet : MC_block.block_facets()) {
                processed_facets[b_facet.f] = true;
                mesh_f_block_[b_facet.f] = block_id;
            }

            blocks_.push_back(MC_block);
        }

        return blocks_.size();
    }

    GEO::index_t QuadMotorCycleGraph::tag_edge(
        const GEO::index_t f,
        const GEO::index_t lv,
        const GEO::index_t tag
        ) {
        GEO::index_t tagged_nb = 0;

        if (mesh_fc_tagged_[mesh_.facets.corner(f, lv)] == GEO::NO_INDEX) {
            mesh_fc_tagged_[mesh_.facets.corner(f, lv)] = tag;
            ++tagged_nb;
        }

        if (const auto nf = mesh_.facets.adjacent(f, lv); nf != GEO::NO_FACET) {
            const auto nlv = mesh_.facets.find_vertex(nf, mesh_.facets.vertex(f, (lv+1)%4));
            assert(nlv != GEO::NO_INDEX);

            if (mesh_fc_tagged_[mesh_.facets.corner(nf, nlv)] == GEO::NO_INDEX) {
                mesh_fc_tagged_[mesh_.facets.corner(nf, nlv)] = tag;
                ++tagged_nb;
            }
        }

        return tagged_nb;
    }

    bool QuadMotorCycleGraph::next_line_edge(
        const GEO::index_t f,
        const GEO::index_t lv,
        const GEO::index_t block_id,
        std::vector<std::pair<GEO::index_t, GEO::index_t>>& ordered_f_lv,
        GEO::index_t& nf,
        GEO::index_t& nlv,
        bool& reached_border
        ) const {
        reached_border = false;

        /* The line continues across the vertex at the tip of the edge. */
        const auto lv1 = (lv+1)%4;
        const auto v = mesh_.facets.vertex(f, lv1);

        /* Motorcycles stop at singular and border vertices (same rule as the burning). */
        if (mesh_v_singular_[v] || mesh_v_border_[v]) {
            reached_border = mesh_v_border_[v];
            return false;
        }

        /* Crossing a tagged edge runs into a wall, which also means leaving the block. */
        if (mesh_fc_tagged_[mesh_.facets.corner(f, lv1)] != GEO::NO_INDEX)
            return false;

        if (get_vertex_incident_facets(mesh_, f, lv1, ordered_f_lv)) {
            reached_border = true; // non-manifold vertex
            return false;
        }
        assert(ordered_f_lv.size() == 4); // regular interior vertex

        nf = ordered_f_lv[1].first;
        nlv = ordered_f_lv[1].second;

        if (mesh_fc_tagged_[mesh_.facets.corner(nf, nlv)] != GEO::NO_INDEX)
            return false; // this part of the line is already a wall

        if (mesh_f_block_[nf] != block_id)
            return false; // the line leaves the block

        return true;
    }

    QuadMotorCycleGraph::LineEnd QuadMotorCycleGraph::walk_line(
        GEO::index_t f,
        GEO::index_t lv,
        const GEO::index_t block_id,
        std::vector<std::pair<GEO::index_t, GEO::index_t>>& ordered_f_lv,
        std::vector<LineEdge>& edges
        ) const {
        const auto start_f = f;
        const auto start_lv = lv;
        const auto max_steps = static_cast<std::size_t>(4)*mesh_.facets.nb();

        for (;;) {
            /* Safety: a grid line never visits a facet corner twice, so a walk that keeps going
             * cannot be a ping-pong between two edges (non-manifold input). */
            if (edges.size() > max_steps)
                return LineEnd::CLOSED;

            edges.push_back({f, lv});

            GEO::index_t nf = GEO::NO_INDEX, nlv = GEO::NO_INDEX;
            bool reached_border = false;
            if (!next_line_edge(f, lv, block_id, ordered_f_lv, nf, nlv, reached_border))
                return reached_border ? LineEnd::MESH_BORDER : LineEnd::WALL;

            if (nf == start_f && nlv == start_lv)
                return LineEnd::CLOSED;

            f = nf;
            lv = nlv;
        }
    }

    bool QuadMotorCycleGraph::trace_line_family(
        const GEO::index_t f0,
        const GEO::index_t lv_ref,
        const GEO::index_t k,
        const GEO::index_t block_id,
        std::vector<std::pair<GEO::index_t, GEO::index_t>>& ordered_f_lv,
        LineTrace& trace
        ) const {
        trace = LineTrace{};

        /* The family k is made of the edges (lv_ref+k) and (lv_ref+k+2). Use an interior, still
         * untagged one: a mesh border edge cannot be walked in both directions, and a tagged one
         * is already a wall. */
        GEO::index_t seed_f = GEO::NO_INDEX, seed_lv = GEO::NO_INDEX;
        for (const auto& d : {k, (k+2)%4}) {
            const auto f_lv = (lv_ref+d)%4;

            if (mesh_fc_tagged_[mesh_.facets.corner(f0, f_lv)] != GEO::NO_INDEX)
                continue;
            if (mesh_.facets.adjacent(f0, f_lv) == GEO::NO_FACET)
                continue;

            seed_f = f0;
            seed_lv = f_lv;
            break;
        }
        if (seed_f == GEO::NO_INDEX)
            return false;

        trace.end_fwd = walk_line(seed_f, seed_lv, block_id, ordered_f_lv, trace.edges);
        if (trace.end_fwd == LineEnd::CLOSED) {
            trace.closed = true;
            return true;
        }

        /* Walk the opposite direction. The two facets adjacent to an edge traverse it in opposite
         * directions, so starting from the other side of the seed edge walks the other half. */
        const auto adj_f = mesh_.facets.adjacent(seed_f, seed_lv);
        assert(adj_f != GEO::NO_FACET);
        const auto adj_lv = mesh_.facets.find_vertex(
            adj_f, mesh_.facets.vertex(seed_f, (seed_lv+1)%4));
        assert(adj_lv != GEO::NO_INDEX);

        std::vector<LineEdge> backward;
        trace.end_bwd = walk_line(adj_f, adj_lv, block_id, ordered_f_lv, backward);

        /* The first edge of the backward walk is the other side of the seed edge, which already
         * belongs to the line. */
        trace.edges.insert(trace.edges.end(), backward.begin()+1, backward.end());

        /* A seam should cut the block open: ends on the mesh border are worth more than ends on
         * an existing wall, and a closed line does not cut anything. */
        const auto end_score = [](const LineEnd end) -> GEO::index_t {
            switch (end) {
                case LineEnd::MESH_BORDER: return 2;
                case LineEnd::WALL: return 1;
                default: return 0;
            }
        };
        trace.score = end_score(trace.end_fwd) + end_score(trace.end_bwd);

        return true;
    }

    bool QuadMotorCycleGraph::cut_block(
        const QuadMotorCycleBlock& block,
        const GEO::index_t block_id
        ) {
        assert(!block.block_facets().empty());

        /* Pick the seed facet. A cuboid block is cut through its middle so that the two halves
         * stay balanced; a loop block is still in flood-fill order, so its first facet (the
         * flood-fill seed) is as good as any. */
        GEO::index_t seed_f = block.block_facets().front().f;
        GEO::index_t seed_lv = block.block_facets().front().lv;
        GEO::index_t preferred = GEO::NO_INDEX;
        if (block.is_rectangular()) {
            const auto mid = block.block_facets()[
                static_cast<std::size_t>(block.len_x()/2) +
                static_cast<std::size_t>(block.len_y()/2)*block.len_x()];
            seed_f = mid.f;
            seed_lv = mid.lv;

            /* A cuboid block with repeating corners was cut open once and still closes up along
             * the cut: the collapsed coarse edges are the two sides of that cut, so the next
             * seam has to run across them. */
            const auto corners = block.corner_vertices();
            if (corners[0] == corners[1] && corners[2] == corners[3])
                preferred = 1; // coarse edges 0-1 and 2-3 run along the first chart axis
            else if (corners[0] == corners[3] && corners[1] == corners[2])
                preferred = 0; // coarse edges 3-0 and 1-2 run along the second chart axis
        }

        /* Trace both grid line families through the seed facet and keep the one that cuts the
         * block open best. */
        std::vector<std::pair<GEO::index_t, GEO::index_t>> ordered_f_lv;
        std::array<LineTrace, 2> traces;
        bool found = false;
        GEO::index_t best = 0;
        for (GEO::index_t k = 0; k < 2; ++k) {
            if (!trace_line_family(seed_f, seed_lv, k, block_id, ordered_f_lv, traces[k]))
                continue;

            if (!found) {
                best = k;
                found = true;
            }
            else if (traces[k].score > traces[best].score ||
                     (traces[k].score == traces[best].score && k == preferred)) {
                best = k;
            }
        }
        if (!found)
            return false;

        GEO::index_t tagged_nb = 0;
        for (const auto& edge : traces[best].edges)
            tagged_nb += tag_edge(edge.f, edge.lv, SEAM_TAG);

        return tagged_nb > 0;
    }

    bool QuadMotorCycleGraph::block_is_valid(
        const QuadMotorCycleBlock& block
        ) {
        if (!block.is_rectangular())
            return false;

        /* A cuboid whose corners repeat wraps around, so its coarse quad would be degenerate. */
        const auto corners = block.corner_vertices();
        for (GEO::index_t i = 0; i < 4; ++i) {
            for (GEO::index_t j = i+1; j < 4; ++j) {
                if (corners[i] == corners[j])
                    return false;
            }
        }

        return true;
    }

    bool QuadMotorCycleGraph::find_invalid_block(
        GEO::index_t& block_id
        ) const {
        for (GEO::index_t i = 0, i_end = blocks_.size(); i < i_end; ++i) {
            if (block_is_valid(blocks_[i]))
                continue;

            block_id = i;
            return true;
        }

        return false;
    }
}
