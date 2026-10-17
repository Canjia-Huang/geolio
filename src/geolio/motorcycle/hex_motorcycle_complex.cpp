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
#include <unordered_map>

#include "geolio/common/pair_hash.h"
#include "geolio/mesh/mesh_operations.h"

namespace
{
    /* Upper bound on the number of artificial cuts performed before giving up. Every cut adds a
     * wall inside a loop block, so only a few are needed in practice. */
    constexpr GEO::index_t MAX_CUT_ATTEMPTS = 16;

    /* Tag value of the artificial walls created by a cut. Outside of the burning, tags are only
     * interpreted as "is this facet a wall?", and 0 is also the value used for border facets. */
    constexpr GEO::index_t SEAM_TAG = 0;

    /* Counts the connected components (loops) of the boundary edges of a sheet. */
    GEO::index_t count_boundary_loops(
        const std::vector<std::pair<GEO::index_t, GEO::index_t>>& boundary_edges
        ) {
        if (boundary_edges.empty())
            return 0;

        /* Vertex -> indices of the incident boundary edges. */
        std::unordered_map<GEO::index_t, std::vector<GEO::index_t>> incident;
        incident.reserve(boundary_edges.size()*2);
        for (GEO::index_t i = 0, i_end = boundary_edges.size(); i < i_end; ++i) {
            incident[boundary_edges[i].first].push_back(i);
            incident[boundary_edges[i].second].push_back(i);
        }

        std::vector<bool> used(boundary_edges.size(), false);
        std::vector<GEO::index_t> stack;
        GEO::index_t loops_nb = 0;
        for (GEO::index_t i = 0, i_end = boundary_edges.size(); i < i_end; ++i) {
            if (used[i])
                continue;

            ++loops_nb;
            stack.push_back(i);
            used[i] = true;
            while (!stack.empty()) {
                const auto e = boundary_edges[stack.back()];
                stack.pop_back();

                for (const auto v : {e.first, e.second}) {
                    for (const auto next : incident[v]) {
                        if (used[next])
                            continue;
                        used[next] = true;
                        stack.push_back(next);
                    }
                }
            }
        }

        return loops_nb;
    }
}

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

            /* Mark facet as burnt (both sides) */
            tag_cell_facet(F_c, F_lf, F_d);

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
        GEO::index_t blocks_nb = decompose_into_blocks();

        /* A block wraps around when the region it covers has no singular edge at all (e.g. a ring
         * of hexahedra, or a periodic mesh). Such a block cannot be unrolled into a cuboid: its
         * layout is not dense and a coarse hex built from it would be degenerate. Cut every such
         * block open along sheets of cell facets (artificial motorcycles) and decompose again
         * until all blocks are valid cuboids.
         *
         * NOTE: two cuts are enough to turn an open loop (a ring) into cuboids. A closed loop
         * (a periodic mesh) is split into cuboids as well, but its coarse hexes then share corner
         * vertices, so the coarse mesh is not a valid hex mesh yet: it needs more cuts, which
         * would be driven by the coarse facet incidence. */
        for (GEO::index_t attempt = 0; ; ++attempt) {
            GEO::index_t block_id = GEO::NO_INDEX;
            if (!find_invalid_block(block_id))
                break;

            if (attempt >= MAX_CUT_ATTEMPTS)
                throw std::logic_error(
                    "HexMotorCycleComplex: a loop block could not be cut into cuboids. "
                    "The block may not be a simple loop, or cutting it needs more cuts.");

            assert(block_id < blocks_.size());
            if (!cut_block(blocks_[block_id], block_id))
                throw std::logic_error(
                    "HexMotorCycleComplex: no sheet can cut a loop block open.");

            blocks_nb = decompose_into_blocks();
        }

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

                GEO::index_t slot = static_cast<GEO::index_t>(compute_pair_hash(v0, v1)) & mask;
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
        mesh_c_block_.assign(mesh_.cells.nb(), GEO::NO_INDEX);

        std::vector<bool> processed_cells(mesh_.cells.nb(), false);
        for (const auto& start_c : mesh_.cells) {
            if (processed_cells[start_c]) // labelled
                continue;

            /* Build motorcycle block */
            HexMotorCycleBlock MC_block(mesh_, mesh_cf_tagged_);
            MC_block.flood_fill_cells(start_c);

            const auto block_id = static_cast<GEO::index_t>(blocks_.size());
            for (const auto& b_cell : MC_block.block_cells()) {
                processed_cells[b_cell.c] = true;
                mesh_c_block_[b_cell.c] = block_id;
            }

            blocks_.push_back(MC_block);
        }

        return blocks_.size();
    }

    GEO::index_t HexMotorCycleComplex::tag_cell_facet(
        const GEO::index_t c,
        const GEO::index_t lf,
        const GEO::index_t tag
        ) {
        GEO::index_t tagged_nb = 0;

        if (mesh_cf_tagged_[8*c+lf] == GEO::NO_INDEX) {
            mesh_cf_tagged_[8*c+lf] = tag;
            ++tagged_nb;
        }

        if (const auto nc = mesh_.cells.adjacent(c, lf); nc != GEO::NO_CELL) {
            const auto nlf = find_hex_facet(
                mesh_,
                nc,
                mesh_.cells.facet_vertex(c, lf, 2),
                mesh_.cells.facet_vertex(c, lf, 1),
                mesh_.cells.facet_vertex(c, lf, 0));
            assert(nlf != GEO::NO_INDEX);

            if (mesh_cf_tagged_[8*nc+nlf] == GEO::NO_INDEX) {
                mesh_cf_tagged_[8*nc+nlf] = tag;
                ++tagged_nb;
            }
        }

        return tagged_nb;
    }

    bool HexMotorCycleComplex::is_sheet_facet_usable(
        const GEO::index_t c,
        const GEO::index_t lf,
        const GEO::index_t block_id
        ) const {
        const auto nc = mesh_.cells.adjacent(c, lf);
        if (nc == GEO::NO_CELL) // on the mesh border
            return false;

        /* Facets are always tagged on both sides, so checking one side is enough. */
        if (mesh_cf_tagged_[8*c+lf] != GEO::NO_INDEX) // already a wall
            return false;

        return mesh_c_block_[nc] == block_id;
    }

    bool HexMotorCycleComplex::next_sheet_facet(
        const GEO::index_t c,
        const GEO::index_t lf,
        const GEO::index_t le,
        std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>>& ordered_c_le_lf,
        GEO::index_t& nc,
        GEO::index_t& nlf
        ) const {
        /* The sheet only continues through regular interior edges (same rule as the burning). */
        if (mesh_ce_singular_[12*c+le] || mesh_ce_border_[12*c+le])
            return false;
        if (get_edge_incident_cells(mesh_, c, le, ordered_c_le_lf))
            return false;
        if (ordered_c_le_lf.size() != 4) // regular interior edge
            return false;
        assert(std::get<0>(ordered_c_le_lf[0]) == c);

        /* The facets around an edge alternate between the planes through it, so the coplanar
         * facet is the one two cells away in the ring (or one, depending on which of the two
         * facets of `c` holds the sheet). */
        const auto opp_i = mesh_.cells.edge_adjacent_facet(c, le, 0) == lf ? 2 : 1;
        nc = std::get<0>(ordered_c_le_lf[opp_i]);
        nlf = std::get<2>(ordered_c_le_lf[opp_i]);

        assert(mesh_.cells.adjacent(c, lf) != nc);
        return true;
    }

    void HexMotorCycleComplex::trace_sheet(
        const GEO::index_t c,
        const GEO::index_t lf,
        const GEO::index_t block_id,
        std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>>& ordered_c_le_lf,
        SheetTrace& trace
        ) const {
        trace = SheetTrace{};

        std::vector<bool> visited(8*mesh_.cells.nb(), false);
        std::vector<SheetFacet> stack;

        stack.push_back({c, lf});
        visited[8*c+lf] = true;

        while (!stack.empty()) {
            const auto [cc, clf] = stack.back();
            stack.pop_back();
            trace.facets.push_back({cc, clf});

            /* Every edge of the facet either continues the sheet or bounds it. */
            for (const auto& le : HEX_LF_INCIDENT_LE[clf]) {
                GEO::index_t nc = GEO::NO_INDEX, nlf = GEO::NO_INDEX;
                if (!next_sheet_facet(cc, clf, le, ordered_c_le_lf, nc, nlf) ||
                    !is_sheet_facet_usable(nc, nlf, block_id)) {
                    trace.boundary_edges.emplace_back(
                        std::min(mesh_.cells.edge_vertex(cc, le, 0), mesh_.cells.edge_vertex(cc, le, 1)),
                        std::max(mesh_.cells.edge_vertex(cc, le, 0), mesh_.cells.edge_vertex(cc, le, 1)));
                    if (mesh_ce_border_[12*cc+le])
                        ++trace.border_edges;
                    continue;
                }

                if (visited[8*nc+nlf])
                    continue;

                visited[8*nc+nlf] = true;
                stack.push_back({nc, nlf});
            }
        }

        trace.closed = trace.boundary_edges.empty();
        trace.boundary_loops = count_boundary_loops(trace.boundary_edges);
    }

    bool HexMotorCycleComplex::cut_block(
        const HexMotorCycleBlock& block,
        const GEO::index_t block_id
        ) {
        assert(!block.block_cells().empty());

        /* Pick the seed cell. A cuboid block is cut through its middle so that the two halves stay
         * balanced; a loop block is still in flood-fill order, so its first cell (the flood-fill
         * seed) is as good as any. */
        GEO::index_t seed_c = block.block_cells().front().c;
        std::array<GEO::index_t, 6> seed_lfs = block.block_cells().front().lfs;
        GEO::index_t preferred = GEO::NO_INDEX;
        if (block.is_cuboid()) {
            const auto mid = block.block_cells()[
                static_cast<std::size_t>(block.len_x()/2) +
                static_cast<std::size_t>(block.len_y()/2)*block.len_x() +
                static_cast<std::size_t>(block.len_z()/2)*block.len_x()*block.len_y()];
            seed_c = mid.c;
            seed_lfs = mid.lfs;

            /* A cuboid block with repeating corners was cut open once and still closes up along
             * the cut: the corners repeat along the wrapped axis, so the next sheet has to be
             * perpendicular to it. Corner `lv` has bit 0/1/2 = x/y/z side. */
            const auto corners = block.corner_vertices();
            constexpr GEO::index_t corner_pairs[3][4][2] = {
                {{0, 1}, {2, 3}, {4, 5}, {6, 7}}, // repeat along x
                {{0, 2}, {1, 3}, {4, 6}, {5, 7}}, // repeat along y
                {{0, 4}, {1, 5}, {2, 6}, {3, 7}} // repeat along z
            };
            for (GEO::index_t axis = 0; axis < 3 && preferred == GEO::NO_INDEX; ++axis) {
                bool wrapped = true;
                for (const auto& pair : corner_pairs[axis]) {
                    if (corners[pair[0]] != corners[pair[1]]) {
                        wrapped = false;
                        break;
                    }
                }
                if (wrapped)
                    preferred = axis;
            }
        }

        /* Trace one sheet per block direction. Sides 2*d and 2*d+1 of the block cell frame are the
         * two facets of the cell that face along direction d. */
        std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>> ordered_c_le_lf;
        std::array<SheetTrace, 3> traces;
        std::array<bool, 3> found{false, false, false};
        for (GEO::index_t d = 0; d < 3; ++d) {
            for (GEO::index_t s = 0; s < 2 && !found[d]; ++s) {
                const auto f_lf = seed_lfs[2*d+s];
                if (!is_sheet_facet_usable(seed_c, f_lf, block_id))
                    continue;

                trace_sheet(seed_c, f_lf, block_id, ordered_c_le_lf, traces[d]);
                found[d] = true;
            }
        }

        /* Keep the sheet that cuts the block open best. How well a sheet cuts is decided by its
         * boundary: a sheet that wraps around has no boundary, and a sheet that only wraps in one
         * direction has two boundary loops instead of one. */
        const auto cuts_better = [](const SheetTrace& candidate, const SheetTrace& current) {
            if (candidate.closed != current.closed)
                return !candidate.closed;
            if ((candidate.boundary_loops == 1) != (current.boundary_loops == 1))
                return candidate.boundary_loops == 1;
            return candidate.border_edges > current.border_edges;
        };

        GEO::index_t best = GEO::NO_INDEX;
        for (GEO::index_t d = 0; d < 3; ++d) {
            if (!found[d])
                continue;
            if (best == GEO::NO_INDEX) {
                best = d;
                continue;
            }

            if (preferred != GEO::NO_INDEX) {
                /* A block that was already cut open once has to be cut again along the same axis. */
                if (d == preferred && best != preferred)
                    best = d;
                continue;
            }

            if (cuts_better(traces[d], traces[best]))
                best = d;
        }
        if (best == GEO::NO_INDEX)
            return false;

        GEO::index_t tagged_nb = 0;
        for (const auto& facet : traces[best].facets)
            tagged_nb += tag_cell_facet(facet.c, facet.lf, SEAM_TAG);

        return tagged_nb > 0;
    }

    bool HexMotorCycleComplex::block_is_valid(
        const HexMotorCycleBlock& block
        ) {
        if (!block.is_cuboid())
            return false;

        /* A cuboid whose corners repeat wraps around, so its coarse hex would be degenerate. */
        const auto corners = block.corner_vertices();
        for (GEO::index_t i = 0; i < 8; ++i) {
            for (GEO::index_t j = i+1; j < 8; ++j) {
                if (corners[i] == corners[j])
                    return false;
            }
        }

        return true;
    }

    bool HexMotorCycleComplex::find_invalid_block(
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