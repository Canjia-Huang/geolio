//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "base_operation.h"
#include <geogram/mesh/mesh_io.h>
#include <geolio/common/log.h>

namespace geolio
{
    template <GEO::index_t DIM>
    BaseOperation<DIM>::BaseOperation(
        MeshElementManager<DIM>& mesh_element_manager
        ) : manager_(mesh_element_manager),
            mesh_(mesh_element_manager.mesh),
            attribute_id_(generate_random_string(22))
    {
        /* Bind attributes */
        mesh_f_timestamping_.bind(mesh_.facets.attributes(), attribute_id_+":timestamping");
        mesh_f_timestamping_.fill(false);
    }

    template <GEO::index_t DIM>
    BaseOperation<DIM>::~BaseOperation(
        ) {
        /* Destroy attributes */
        if (mesh_f_timestamping_.is_bound())
            mesh_f_timestamping_.destroy();
    }

    template <GEO::index_t DIM>
    bool BaseOperation<DIM>::post_check(
        ) {
        if constexpr (false) { // Check whether all used facets use the used vertices.
            for (const auto& f : mesh_.facets) {
                if (!manager_.mesh_f_used[f])
                    continue;
                for (GEO::index_t lv = 0; lv < 3; ++lv) {
                    if (const auto& v = mesh_.facets.vertex(f, lv);
                        !manager_.mesh_v_used[v])
                        return false;
                }
            }
        }
        if constexpr (false) { // Adjacent facets of a used facet should also be detected as used.
            for (const auto& f : mesh_.facets) {
                if (!manager_.mesh_f_used[f])
                    continue;
                for (GEO::index_t lv = 0; lv < 3; ++lv) {
                    if (const auto& nf = mesh_.facets.adjacent(f, lv);
                        nf != GEO::NO_FACET)
                        assert(manager_.mesh_f_used[nf]);
                }
            }
        }
        if constexpr (false) { // Fixed mesh facet corner should appear in pairs.
            for (const auto& f : mesh_.facets) {
                if (!manager_.mesh_f_used[f])
                    continue;
                for (GEO::index_t lv = 0; lv < 3; ++lv) {
                    if (const auto& nf = mesh_.facets.adjacent(f, lv);
                        nf != GEO::NO_FACET) {
                        if (manager_.mesh_fc_fixed[mesh_.facets.corner(f, lv)]) {
                            const auto& v = mesh_.facets.vertex(f, lv);
                            const auto nlv = mesh_.facets.find_vertex(nf, v);
                            assert(nlv != GEO::NO_INDEX);
                            assert(manager_.mesh_fc_fixed[mesh_.facets.corner(nf, (nlv+2)%3)]);
                        }
                    }
                }
            }
        }

        return true;
    }

    template class BaseOperation<2>;
    template class BaseOperation<3>;
}
