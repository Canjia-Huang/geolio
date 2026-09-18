//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/5.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "quad_motorcycle_graph.h"
#include <cassert>
#include <geolio/common/utils.h>

#include "geolio/mesh/mesh_operations.h"

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

            /* Mark edge as burnt */
            mesh_fc_tagged_[mesh_.facets.corner(F_f, F_lv)] = F_d;
            if (const auto& nf = mesh_.facets.adjacent(F_f, F_lv);
                nf != GEO::NO_FACET) {
                const auto& nlv = mesh_.facets.find_vertex(nf, mesh_.facets.vertex(F_f, (F_lv+1)%4));
                assert(nlv != GEO::NO_INDEX);
                mesh_fc_tagged_[mesh_.facets.corner(nf, nlv)] = F_d;
            }

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
        const GEO::index_t blocks_nb = decompose_into_blocks();

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

        std::vector<bool> processed_facets(mesh_.facets.nb(), false);
        for (const auto& start_f : mesh_.facets) {
            if (processed_facets[start_f]) // labelled
                continue;

            /* Build motorcycle block */
            QuadMotorCycleBlock MC_block(mesh_, mesh_fc_tagged_);
            MC_block.flood_fill_facets(start_f);

            for (const auto& b_facet : MC_block.block_facets())
                processed_facets[b_facet.f] = true;

            blocks_.push_back(MC_block);
        }

        return blocks_.size();
    }
}
