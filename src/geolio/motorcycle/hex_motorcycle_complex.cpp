//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/5.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "hex_motorcycle_complex.h"
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <geogram/mesh/mesh_io.h>
#include <geolio/common/utils.h>
#include <geolio/mesh/hex_operations.h>
#include "geolio/mesh/mesh_operations.h"

namespace geolio
{
    HexMotorCycleComplex::HexMotorCycleComplex(
        const GEO::Mesh& mesh
        ) : attribute_id_(generate_random_string(22)),
            mesh_(mesh)
    {
        assert(std::all_of(
            mesh_.cells.cell_type_ptr(0),
            mesh_.cells.cell_type_ptr(0)+mesh_.cells.nb(),
            [&](const auto cell_type) { return cell_type == GEO::MESH_HEX; })); // check all-hex mesh

        mesh_cf_tagged_.bind(mesh_.cell_facets.attributes(), attribute_id_+":tagged");
        mesh_cf_tagged_.fill(GEO::NO_INDEX);

        find_all_singular_and_border_edges();
    }

    HexMotorCycleComplex::~HexMotorCycleComplex(
        ) {
        if (mesh_cf_tagged_.is_bound())
            mesh_cf_tagged_.destroy();
    }

    GEO::index_t HexMotorCycleComplex::compute(
        const MotorCycleType complex_type
        ) {
        std::priority_queue<Fire> queue;

        /* Ignite */
        ignite(queue);

        /* Scratch buffer for the cells around an edge; reused across iterations so that the
         * traversal does not allocate on every call. */
        std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>> ordered_c_le_lf;

        /* Burning */
        while (!queue.empty()) {
            const auto& fire = queue.top();
            const auto F_d = fire.d;
            const auto F_c = fire.c;
            const auto F_le = fire.le;
            const auto F_lf = fire.lf;
            queue.pop();

            /* Already burnt: the same facet was already reached by an earlier (closer) fire.
             * Every facet only tags itself when it is popped, so a facet that is pushed by
             * several of its neighbours is queued several times. Re-processing it burns
             * nothing new but re-propagates the fire, which makes the queue never settle. */
            if (mesh_cf_tagged_[8*F_c+F_lf] != GEO::NO_INDEX)
                continue;

            /* Alive */
            bool alive = false;
            if (complex_type == BASE_COMPLEX)
                alive = true;
            else {
                assert(complex_type == MOTORCYCLE_COMPLEX);
                if (mesh_ce_singular_[12*F_c+F_le])
                    alive = true;
                else {
                    /* Find all incident facets */
                    if (get_edge_incident_cells(mesh_, F_c, F_le, ordered_c_le_lf)) { // append the preceding border facet
                        const auto pre_lf = get<2>(ordered_c_le_lf[0]);
                        const auto& lf0 = HEX_LE_INCIDENT_LF[F_le][0];
                        const auto& lf1 = HEX_LE_INCIDENT_LF[F_le][1];
                        if (pre_lf == lf0) // append lf1
                            ordered_c_le_lf.emplace_back(F_c, F_le, lf1);
                        else { // append lf0
                            assert(pre_lf == lf1);
                            ordered_c_le_lf.emplace_back(F_c, F_le, lf0);
                        }
                    }

                    GEO::index_t tagged_facets_nb = 0;
                    for (const auto& [c, _, lf] : ordered_c_le_lf) {
                        if (mesh_cf_tagged_[8*c+lf] != GEO::NO_INDEX)
                            ++tagged_facets_nb;
                    }

                    if (tagged_facets_nb < 3)
                        alive = true;
                }
            }
            if (!alive)
                continue;

            /* Mark facet as burnt */
            mesh_cf_tagged_[8*F_c+F_lf] = F_d;
            if (const auto& nc = mesh_.cells.adjacent(F_c, F_lf);
                nc != GEO::NO_CELL) {
                const auto& nlf = find_hex_facet(
                    mesh_,
                    nc,
                    mesh_.cells.facet_vertex(F_c, F_lf, 2),
                    mesh_.cells.facet_vertex(F_c, F_lf, 1),
                    mesh_.cells.facet_vertex(F_c, F_lf, 0));
                assert(nlf != GEO::NO_INDEX);
                mesh_cf_tagged_[8*nc+nlf] = F_d;
            }

            /* Burning */
            for (const auto& F_le1 : HEX_LF_INCIDENT_LE[F_lf]) {
                if (F_le1 == F_le || mesh_ce_singular_[12*F_c+F_le1] || mesh_ce_border_[12*F_c+F_le1]) // need to be regular and interior
                    continue;

                /* Find all incident facets */
                const auto on_border = get_edge_incident_cells(mesh_, F_c, F_le1, ordered_c_le_lf);
                assert(!on_border);
                assert(get<0>(ordered_c_le_lf[0]) == F_c);
                assert(ordered_c_le_lf.size() == 4); // regular

                const GEO::index_t opp_i = mesh_.cells.edge_adjacent_facet(F_c, F_le1, 0) == F_lf ? 2 : 1; // let ordered_c_le_lf[opp_i] the next to burn

                if (const auto& [opp_c, opp_le, opp_lf] = ordered_c_le_lf[opp_i];
                    mesh_cf_tagged_[8*opp_c+opp_lf] == GEO::NO_INDEX
                    ) {
                    assert((mesh_.cells.edge_vertex(F_c, F_le1, 0) == mesh_.cells.edge_vertex(opp_c, opp_le, 0) &&
                            mesh_.cells.edge_vertex(F_c, F_le1, 1) == mesh_.cells.edge_vertex(opp_c, opp_le, 1)) ||
                            mesh_.cells.edge_vertex(F_c, F_le1, 0) == mesh_.cells.edge_vertex(opp_c, opp_le, 1) &&
                            mesh_.cells.edge_vertex(F_c, F_le1, 1) == mesh_.cells.edge_vertex(opp_c, opp_le, 0));
                    assert(mesh_.cells.adjacent(F_c, F_lf) != opp_c);

                    Fire new_F{};
                    new_F.d = F_d+1;
                    new_F.c = opp_c;
                    new_F.le = opp_le;
                    new_F.lf = opp_lf;

                    queue.push(new_F);
                }
            }
        }

        /* Label border facets */
        for (const auto& c : mesh_.cells) {
            for (GEO::index_t lf = 0; lf < 6; ++lf) {
                if (mesh_.cells.adjacent(c, lf) == GEO::NO_CELL)
                    mesh_cf_tagged_[8*c+lf] = 0;
            }
        }

        /* Decompose */
        const GEO::index_t blocks_nb = decompose_into_blocks();

        return blocks_nb;
    }

    void HexMotorCycleComplex::label_blocks(
        GEO::Attribute<GEO::index_t>& mesh_c_block
        ) const {
        assert(mesh_c_block.is_bound());
        assert(mesh_c_block.size() == mesh_.cells.nb());
        if (blocks_.empty())
            throw std::logic_error("Need to call compute() first!");

        for (GEO::index_t i = 0, i_end = blocks_.size(); i < i_end; ++i) {
            const auto& block = blocks_[i];
            for (const auto& bcs = block.block_cells();
                const auto& bc : bcs)
                mesh_c_block[bc.c] = i;
        }
    }

    void HexMotorCycleComplex::create_coarse_mesh(
        GEO::Mesh& mesh_out,
        std::vector<GEO::index_t>* old_cf_to_new_cf
        ) const {
        if (blocks_.empty())
            throw std::logic_error("Need to call compute() first!");

        mesh_out.clear(false);
        mesh_out.copy(mesh_, false, GEO::MESH_VERTICES);

        GEO::index_t new_c = mesh_out.cells.create_hexes(blocks_.size());
        for (const auto& block : blocks_) {
            for (GEO::index_t lv = 0; lv < 8; ++lv)
                mesh_out.cells.set_vertex(new_c, lv, block.cell_corner_vertex(lv));
            ++new_c;
        }
        mesh_out.cells.connect();

        if (old_cf_to_new_cf != nullptr) {
            old_cf_to_new_cf->assign(8*mesh_.cells.nb(), GEO::NO_INDEX);
            assert(mesh_out.cells.nb() == blocks_.size());
            for (const auto& c : mesh_out.cells) {
                for (const auto& BC : blocks_[c].block_cells()) {
                    for (GEO::index_t lf = 0; lf < 6; ++lf) {
                        if (const auto old_cf = 8*BC.c + BC.lfs[lf];
                            mesh_cf_tagged_[old_cf] != GEO::NO_INDEX
                            )
                            (*old_cf_to_new_cf)[old_cf] = 8*c+lf;
                    }
                }
            }
        }
    }

    namespace
    {
        /* Hash of a mesh edge key (v0 < v1), used by the flat edge table in
         * find_all_singular_and_border_edges(). */
        GEO::index_t edge_key_hash(
            const GEO::index_t v0,
            const GEO::index_t v1
            ) {
            // Fibonacci hashing of the packed key: the low bits of v1 alone are too regular
            // for linear probing on a structured grid mesh.
            const auto key = (static_cast<uint64_t>(v0) << 32) | static_cast<uint64_t>(v1);
            return static_cast<GEO::index_t>((key * 0x9E3779B97F4A7C15ull) >> 32);
        }
    }

    void HexMotorCycleComplex::find_all_singular_and_border_edges(
        ) {
        const GEO::index_t cell_edges_nb = 12*mesh_.cells.nb();
        mesh_ce_singular_.assign(cell_edges_nb, false);
        mesh_ce_border_.assign(cell_edges_nb, false);

        /* Group the cell-edges by mesh edge (ev0, ev1), ev0 < ev1 -> (incident cells nb, is on boundary?).
         * A flat open-addressed table is used instead of std::unordered_map: the latter is node based,
         * so it allocates once per distinct edge and was the bottleneck of this pass. */
        GEO::index_t cap = 16;
        while (cap < cell_edges_nb)
            cap <<= 1; // cap >= nb of distinct edges, so the table is never full
        const GEO::index_t mask = cap - 1;
        std::vector<GEO::index_t> slot_v0(cap, GEO::NO_INDEX); // GEO::NO_INDEX -> free slot
        std::vector<GEO::index_t> slot_v1(cap, GEO::NO_INDEX);
        std::vector<GEO::index_t> slot_edge(cap, GEO::NO_INDEX); // mesh edge id
        std::vector<GEO::index_t> ce_edge(cell_edges_nb, GEO::NO_INDEX); // [12*c+le] -> mesh edge id
        std::vector<GEO::index_t> edge_incident_cells_nb;
        std::vector<bool> edge_on_boundary;
        edge_incident_cells_nb.reserve(cell_edges_nb/3);
        edge_on_boundary.reserve(cell_edges_nb/3);

        for (const auto& c : mesh_.cells) {
            for (GEO::index_t le = 0; le < 12; ++le) {
                const bool is_on_boundary =
                    (mesh_.cells.adjacent(c, mesh_.cells.edge_adjacent_facet(c, le, 0)) == GEO::NO_FACET) ||
                    (mesh_.cells.adjacent(c, mesh_.cells.edge_adjacent_facet(c, le, 1)) == GEO::NO_FACET);
                const auto ev0 = mesh_.cells.edge_vertex(c, le, 0);
                const auto ev1 = mesh_.cells.edge_vertex(c, le, 1);
                const auto v0 = std::min(ev0, ev1);
                const auto v1 = std::max(ev0, ev1);

                GEO::index_t slot = edge_key_hash(v0, v1) & mask;
                for (;;) {
                    if (slot_v0[slot] == GEO::NO_INDEX) { // new mesh edge
                        const GEO::index_t e = edge_incident_cells_nb.size();
                        slot_v0[slot] = v0;
                        slot_v1[slot] = v1;
                        slot_edge[slot] = e;
                        ce_edge[12*c+le] = e;
                        edge_incident_cells_nb.push_back(1);
                        edge_on_boundary.push_back(is_on_boundary);
                        break;
                    }
                    if (slot_v0[slot] == v0 && slot_v1[slot] == v1) { // known mesh edge
                        const auto e = slot_edge[slot];
                        ce_edge[12*c+le] = e;
                        ++edge_incident_cells_nb[e];
                        edge_on_boundary[e] = edge_on_boundary[e] || is_on_boundary;
                        break;
                    }
                    slot = (slot+1) & mask;
                }
            }
        }

        /* Label le */
        for (GEO::index_t c = 0, c_end = mesh_.cells.nb(); c < c_end; ++c) {
            for (GEO::index_t le = 0; le < 12; ++le) {
                const auto e = ce_edge[12*c+le];
                assert(e != GEO::NO_INDEX);

                if (edge_on_boundary[e]) {
                    mesh_ce_border_[12*c+le] = true;
                    if (edge_incident_cells_nb[e] != 2)
                        mesh_ce_singular_[12*c+le] = true;
                }
                else {
                    if (edge_incident_cells_nb[e] != 4)
                        mesh_ce_singular_[12*c+le] = true;
                }
            }
        }
    }

    void HexMotorCycleComplex::ignite(
        std::priority_queue<Fire>& queue
        ) const {
        assert(mesh_ce_singular_.size() == 12*mesh_.cells.nb());

        while (!queue.empty())
            queue.pop();

        std::vector<bool> processed_edges(12*mesh_.cells.nb(), false);
        std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>> ordered_c_le_lf; // reused, see compute()
        for (const auto& c : mesh_.cells) {
            for (GEO::index_t le = 0; le < 12; ++le) {
                if (processed_edges[12*c+le] || !mesh_ce_singular_[12*c+le]) // for all singular edge
                    continue;

                /* Find all incident interior facets */
                get_edge_incident_cells(mesh_, c, le, ordered_c_le_lf);
                for (const auto& [adj_c, adj_le, adj_lf] : ordered_c_le_lf) {
                    processed_edges[12*adj_c+adj_le] = true;

                    if (mesh_.cells.adjacent(adj_c, adj_lf) == GEO::NO_CELL)
                        continue;

                    Fire F{};
                    F.d = 0;
                    F.c = adj_c;
                    F.le = adj_le;
                    F.lf = adj_lf;

                    queue.push(F);
                }
            }
        }
    }

    GEO::index_t HexMotorCycleComplex::decompose_into_blocks(
        ) {
        blocks_.clear();

        std::vector<bool> prcessed_cells(mesh_.cells.nb(), false);
        for (const auto& start_c : mesh_.cells) {
            if (prcessed_cells[start_c]) // labelled
                continue;

            /* Build motorcycle block */
            HexMotorCycleBlock MC_block(mesh_, mesh_cf_tagged_);
            MC_block.flood_fill_cells(start_c);

            for (const auto& b_cell : MC_block.block_cells())
                prcessed_cells[b_cell.c] = true;

            blocks_.push_back(MC_block);
        }

        return blocks_.size();
    }
}