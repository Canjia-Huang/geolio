//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_BASE_OPERATION_H
#define GEOLIO_BASE_OPERATION_H

#include "mesh_element_manager.h"
#include <string>

namespace geolio
{
    template <GEO::index_t DIM>
    class BaseOperation {
    public:
        /**
         * @brief Constructs a BaseOperation bound to the given mesh element manager.
         * @details Binds a per-facet "timestamping" attribute on the manager's mesh under a
         *          random, session-unique attribute name (so it cannot collide with
         *          externally-visible attributes) and pre-fills it with 0. Derived operations
         *          increment a facet's timestamp whenever its topology changes, allowing stale
         *          entries in their priority queues to be detected. The random name is generated
         *          by generate_random_string().
         * @param[in] mesh_element_manager The mesh element manager exposing the mesh and its
         *                                 usage/fixed element attributes.
         */
        explicit BaseOperation(MeshElementManager<DIM>& mesh_element_manager);

        /**
         * @brief Destroys the BaseOperation.
         * @details Destroys the bound "timestamping" facet attribute if it is still bound,
         *          releasing the underlying mesh attribute storage.
         */
        ~BaseOperation();

        /**
         * @brief Validates the consistency of the mesh element usage state.
         * @details Runs a set of consistency checks that are currently disabled at compile
         *          time (if constexpr(false)): used facets must only reference used vertices,
         *          the adjacent facets of a used facet must be used, and fixed facet corners
         *          must appear in matching pairs. In the current build it always returns true.
         * @return true if the mesh passes all checks (always true in the current build).
         */
        bool post_check();

    protected:
        /**
         * @brief Invokes @p func on every undirected edge of the mesh exactly once.
         * @details Iterates over all used facets and their three local edges, skipping edges
         *          whose corner has already been processed. Whenever an interior edge is
         *          visited, its counterpart corner on the adjacent facet is also marked as
         *          processed, so each edge is visited from a single side. The callback receives
         *          the facet index and the local vertex index identifying the oriented edge
         *          (lv -> lv+1).
         * @tparam Func Callable type accepting (GEO::index_t f, GEO::index_t lv).
         * @param[in] func Callback invoked once per undirected edge.
         */
        template <typename Func>
        void for_each_edge(Func func) {
            std::vector<bool> processed_edge(mesh_.facet_corners.nb(), false);
            for (const auto& f : mesh_.facets) {
                if (!manager_.mesh_f_used[f])
                    continue;

                for (GEO::index_t lv = 0; lv < 3; ++lv) {
                    if (const auto& fc = mesh_.facets.corner(f, lv);
                        processed_edge[fc])
                        continue;
                    else
                        processed_edge[fc] = true;

                    func(f, lv);

                    if (const auto& nf = mesh_.facets.adjacent(f, lv);
                        nf != GEO::NO_FACET
                        ) {
                        const auto& nlv = mesh_.facets.find_vertex(nf, mesh_.facets.vertex(f, lv));
                        assert(nlv != GEO::NO_INDEX);
                        processed_edge[mesh_.facets.corner(nf, (nlv+2)%3)] = true;
                    }
                }
            }
        }

        MeshElementManager<DIM>& manager_;
        const std::string attribute_id_; // Prevent anyone from using these attributes externally (unsafety).

        GEO::Mesh& mesh_;
        GEO::Attribute<GEO::index_t> mesh_f_timestamping_; // f -> timestamp counter to detect stale queue entries
    };

    extern template class BaseOperation<2>;
    extern template class BaseOperation<3>;
}

#endif //GEOLIO_BASE_OPERATION_H
