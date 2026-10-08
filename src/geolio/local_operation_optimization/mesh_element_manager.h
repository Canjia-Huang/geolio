//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_MESH_ELEMENT_MANAGER_H
#define GEOLIO_MESH_ELEMENT_MANAGER_H

#include <geogram/mesh/mesh.h>
#include <geolio/common/utils.h>
#include <cassert>
#include <string>
#include <unordered_set>
#include <vector>
#include "geolio/common/pair_hash.h"

namespace geolio
{
    template <GEO::index_t DIM>
    class MeshElementManager {
    public:
        /**
         * @brief Constructs a MeshElementManager over the given triangle mesh.
         * @details Records the mesh reference and generates a random attribute name prefix via
         *          generate_random_string(). Binds and initializes the per-vertex, per-facet and
         *          per-corner usage/fixed attributes under that prefix: vertices and facets start
         *          as used, boundary, fixed and non-manifold flags start false.
         * @param[in] _mesh The triangle mesh to manage; it must contain only simplex facets.
         */
        explicit MeshElementManager(GEO::Mesh& _mesh);

        /**
         * @brief Destroys the MeshElementManager.
         * @details Destroys all bound element attributes (boundary, fixed, non-manifold, used
         *          and fixed-corner) if they are still bound, releasing their mesh storage.
         */
        ~MeshElementManager();

        /**
         * @brief Returns a free vertex index that is available for reuse.
         * @details If the free-vertex pool is empty, first allocates a batch of new vertices by
         *          appending as many vertices as the mesh currently has. Pops the back of the
         *          pool, asserts the index is in range, and marks it as used in mesh_v_used.
         * @return The index of an available (unused) vertex.
         */
        [[nodiscard]] GEO::index_t require_new_vertex() {
            if (free_vertices_.empty())
                allocate_new_vertices();
            assert(!free_vertices_.empty());

            const GEO::index_t new_v = free_vertices_.back();
            free_vertices_.pop_back();
            assert(new_v < mesh.vertices.nb());
            mesh_v_used[new_v] = true;

            return new_v;
        }

        /**
         * @brief Marks a vertex as disused and recycles it for future reuse.
         * @details Pushes the vertex index onto the free-vertex pool so require_new_vertex()
         *          can hand it out again, and zeroes the vertex's per-vertex attribute values to
         *          restore a clean state.
         * @param[in] v Index of the vertex to disuse.
         */
        void disuse_vertex(const GEO::index_t v) {
            assert(v < mesh.vertices.nb());

            /* Recycle */
            free_vertices_.push_back(v);

            /* Restore attributes */
            mesh.vertices.attributes().zero_item(v);
        }

        /**
         * @brief Returns a free facet index that is available for reuse.
         * @details If the free-facet pool is empty, first allocates a batch of new facets by
         *          appending as many triangles as the mesh currently has. Pops the back of the
         *          pool, asserts the index is in range, and marks it as used in mesh_f_used.
         * @return The index of an available (unused) facet.
         */
        [[nodiscard]] GEO::index_t require_new_facet() {
            if (free_facets_.empty())
                allocate_new_facets();
            assert(!free_facets_.empty());

            const GEO::index_t new_f = free_facets_.back();
            free_facets_.pop_back();
            assert(new_f < mesh.facets.nb());
            mesh_f_used[new_f] = true;

            return new_f;
        }

        /**
         * @brief Marks a facet as disused and recycles it for future reuse.
         * @details Pushes the facet index onto the free-facet pool, zeroes the facet's attribute
         *          values, and zeroes the attribute values of the facet's three corners.
         * @param[in] f Index of the facet to disuse.
         */
        void disuse_facet(const GEO::index_t f) {
            assert(f < mesh.facets.nb());

            /* Recycle */
            free_facets_.push_back(f);

            /* Restore attributes */
            mesh.facets.attributes().zero_item(f);
            for (GEO::index_t lv = 0; lv < 3; ++lv)
                mesh.facet_corners.attributes().zero_item(mesh.facets.corner(f, lv));
        }

        /**
         * @brief Physically removes disused facets (and optionally isolated vertices) from the mesh.
         * @details If both free pools are empty, returns immediately. Otherwise builds a delete
         *          mask over the free-facet indices and calls mesh.facets.delete_elements(),
         *          forwarding remove_isolated_vertices, then clears both free pools. Asserts that
         *          all remaining used flags are consistent after deletion.
         * @param[in] remove_isolated_vertices When true, vertices that become isolated after the
         *                                     facet deletion are removed as well.
         */
        void clean_unused_elements(bool remove_isolated_vertices = true);

        /**
         * @brief Computes the Euclidean length of the local edge of facet @p f at local vertex @p lv.
         * @details Reads the two endpoints of the oriented edge (lv -> lv+1) of facet @p f and
         *          returns the GEO::distance between them, using the DIM-dimensional coordinates
         *          of the templated manager.
         * @param[in] f Index of the facet containing the edge.
         * @param[in] lv Local vertex index identifying the oriented edge (lv -> lv+1).
         * @return The length of the edge.
         */
        [[nodiscard]] double get_edge_length(const GEO::index_t f, const GEO::index_t lv) const {
            assert(f < mesh.facets.nb());
            assert(lv < 3);
            return GEO::distance(mesh.facets.point<DIM>(f, lv), mesh.facets.point<DIM>(f, (lv+1)%3));
        }

        /**
         * @brief Computes the average edge length over all facets of the mesh.
         * @details Iterates over every facet and accumulates the length of all three local edges
         *          via get_edge_length(), then divides the sum by the total number of edges.
         * @return The average mesh edge length.
         */
        [[nodiscard]] double compute_average_mesh_edge_length() const;

        GEO::Mesh& mesh;
        GEO::Attribute<bool> mesh_v_boundary; // v -> on boundary
        GEO::Attribute<bool> mesh_v_fixed; // v -> fixed
        GEO::Attribute<bool> mesh_v_non_manifold; // v -> non manifold
        GEO::Attribute<bool> mesh_v_used; // v -> used
        GEO::Attribute<bool> mesh_f_used; // f -> used
        std::unordered_set<std::pair<GEO::index_t, GEO::index_t>, PairHash> fixed_edges_;

    private:
        /**
         * @brief Allocates a batch of new mesh vertices for future reuse.
         * @details Appends as many new vertices as the mesh currently has via create_vertices()
         *          and pushes their indices onto the free-vertex pool so require_new_vertex()
         *          can hand them out later.
         */
        void allocate_new_vertices();

        /**
         * @brief Allocates a batch of new triangle facets for future reuse.
         * @details Appends as many new triangles as the mesh currently has via create_triangles()
         *          and pushes their indices onto the free-facet pool so require_new_facet()
         *          can hand them out later.
         */
        void allocate_new_facets();

        const std::string attribute_id_; // Prevent anyone from using these attributes externally (unsafety).

        std::vector<GEO::index_t> free_vertices_;
        std::vector<GEO::index_t> free_facets_;
    };

    extern template class MeshElementManager<2>;
    extern template class MeshElementManager<3>;
}

#endif //GEOLIO_MESH_ELEMENT_MANAGER_H
