//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/5.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "quad_motorcycle_block.h"
#include <cassert>
#include <geogram/mesh/mesh_io.h>
#include <array>

namespace
{
    const std::array<GEO::vec2i, 4> QUAD_LV_EXT_COORD = {
        {
            GEO::vec2i(0, -1),
            GEO::vec2i(1, 0),
            GEO::vec2i(0, 1),
            GEO::vec2i(-1, 0)
        }
    };
}

namespace geolio
{
    QuadMotorCycleBlock::QuadMotorCycleBlock(
        const GEO::Mesh& mesh,
        const GEO::Attribute<GEO::index_t>& mesh_fc_tagged
        ) : mesh_(mesh),
            mesh_fc_tagged_(mesh_fc_tagged)
    {
        assert([&]() {
                for (const auto& f : mesh_.facets) {
                    if (mesh_.facets.nb_vertices(f) != 4)
                        return false;
                }
                return true;
            }()); // check all-quad mesh
        assert(mesh_fc_tagged_.is_bound());
    }

    void QuadMotorCycleBlock::flood_fill_facets(
        const GEO::index_t start_f
        ) {
        std::vector<bool> prcessed_facets(mesh_.facets.nb(), false);

        std::vector<BlockFacet> stack;

        /* Initialize */
        {
            BlockFacet BF;
            BF.f = start_f;
            BF.coord = GEO::vec2i(0, 0);
            BF.lv = 0; // use the lv order of facet start_f as the reference

            stack.push_back(BF);
            prcessed_facets[start_f] = true;
        }

        /* Start */
        while (!stack.empty()) {
            const auto BF = stack.back();
            stack.pop_back();
            block_facets_.push_back(BF);

            const auto f = BF.f;
            for (GEO::index_t lv = 0; lv < 4; ++lv) {
                const auto& f_lv = (BF.lv+lv)%4;
                assert(f_lv < 4);

                if (mesh_fc_tagged_[mesh_.facets.corner(f, f_lv)] != GEO::NO_INDEX) // a wall
                    continue;

                const auto nf = mesh_.facets.adjacent(f, f_lv);
                if (nf == GEO::NO_FACET) // on border
                    continue;

                if (prcessed_facets[nf])
                    continue;

                BlockFacet nBF;
                nBF.f = nf;
                nBF.coord = BF.coord + QUAD_LV_EXT_COORD[lv];

                /* Match lv order
                 * x-------x 0-------3  x-------x 1-------0  x-------x 2-------1  x-------x 3-------2
                 * |  nf   | |   f   |  |  nf   | |   f   |  |  nf   | |   f   |  |  nf   | |   f   |
                 * |       | |       |  |       | |       |  |       | |       |  |       | |       |
                 * x-----nlv 1-------2  x-----nlv 2-------3  x-----nlv 3-------0  x-----nlv 0-------1
                 */
                const auto nlv = mesh_.facets.find_vertex(nf, mesh_.facets.vertex(f, (f_lv+1)%4));
                assert(nlv != GEO::NO_INDEX);
                nBF.lv = (nlv+6-lv)%4;
                // switch (lv) {
                //     case 0: nBF.lv = (nlv+2)%4; break;
                //     case 1: nBF.lv = (nlv+1)%4; break;
                //     case 2: nBF.lv = nlv; break;
                //     case 3: nBF.lv = (nlv+3)%4; break;
                //     default: assert(0);
                // }

                stack.push_back(nBF);
                prcessed_facets[nf] = true;
            }
        }

        // DEBUG
        if constexpr (false) {
            GEO::Mesh mesh_out;
            mesh_out.copy(mesh_);
            GEO::Attribute<int> mesh_out_f_coord_x(mesh_out.facets.attributes(), "coord_x");
            GEO::Attribute<int> mesh_out_f_coord_y(mesh_out.facets.attributes(), "coord_y");
            GEO::vector<GEO::index_t> facets_to_delete(mesh_out.facets.nb(), 1);
            for (const auto& BF : block_facets_) {
                facets_to_delete[BF.f] = 0;
                mesh_out_f_coord_x[BF.f] = BF.coord.x;
                mesh_out_f_coord_y[BF.f] = BF.coord.y;
            }
            mesh_out.facets.delete_elements(facets_to_delete);
            GEO::mesh_save(mesh_out, "debug.geogram");
            throw std::logic_error("im here");
        }

        rebuild_ordered_facets();
    }

    GEO::index_t QuadMotorCycleBlock::facet_corner_vertex(
        const GEO::index_t lv
        ) const {
        assert(!block_facets_.empty());
        assert(lv < 4);

        switch (lv) {
            case 0: {
                const auto& BF = block_facets_[0];
                return mesh_.facets.vertex(BF.f, BF.lv);
            }
            case 1: {
                const auto& BF = block_facets_[len_x_-1];
                return mesh_.facets.vertex(BF.f, (BF.lv+1)%4);
            }
            case 2: {
                const auto& BF = block_facets_[len_x_*len_y_-1];
                return mesh_.facets.vertex(BF.f, (BF.lv+2)%4);
            }
            case 3: {
                const auto& BF = block_facets_[len_x_*(len_y_-1)];
                return mesh_.facets.vertex(BF.f, (BF.lv+3)%4);
            }
            default:
                assert(0);
                return GEO::NO_INDEX;
        }
    }

    void QuadMotorCycleBlock::rebuild_ordered_facets(
        ) {
        assert(!block_facets_.empty());

        int min_x = std::numeric_limits<int>::max();
        int max_x = -std::numeric_limits<int>::max();
        int min_y = std::numeric_limits<int>::max();
        int max_y = -std::numeric_limits<int>::max();
        for (const auto& BF : block_facets_) {
            min_x = std::min(min_x, BF.coord.x);
            max_x = std::max(max_x, BF.coord.x);
            min_y = std::min(min_y, BF.coord.y);
            max_y = std::max(max_y, BF.coord.y);
        }
        len_x_ = max_x - min_x + 1;
        len_y_ = max_y - min_y + 1;
        if (len_x_*len_y_ != block_facets_.size()) {
            throw std::logic_error("This block is likely toroidal rather than cuboidal! But the split function has not yet been implemented.");
            // TODO: need to split this block
            return;
        }

        /* Fill orderly */
        std::vector<BlockFacet> new_block_facets(len_x_*len_y_);
        for (auto& BF : block_facets_) {
            BF.coord.x -= min_x;
            BF.coord.y -= min_y;
            new_block_facets[BF.coord.x + BF.coord.y*len_x_] = BF;
        }
        block_facets_.swap(new_block_facets);
        assert(std::ranges::all_of(block_facets_, [&](const auto& BF){ return BF.f != GEO::NO_FACET; }));

        // DEBUG
        if constexpr (false) {
            GEO::Mesh mesh_out;
            mesh_out.copy(mesh_);
            GEO::Attribute<int> mesh_out_f_coord_x(mesh_out.facets.attributes(), "coord_x");
            GEO::Attribute<int> mesh_out_f_coord_y(mesh_out.facets.attributes(), "coord_y");
            GEO::vector<GEO::index_t> facets_to_delete(mesh_out.facets.nb(), 1);
            for (const auto& BF : block_facets_) {
                facets_to_delete[BF.f] = 0;
                mesh_out_f_coord_x[BF.f] = BF.coord.x;
                mesh_out_f_coord_y[BF.f] = BF.coord.y;
            }
            mesh_out.facets.delete_elements(facets_to_delete);
            GEO::mesh_save(mesh_out, "debug.geogram");
            throw std::logic_error("im here");
        }
    }
}