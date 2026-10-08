//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "mesh_element_manager.h"
#include <cassert>
#include <ranges>

namespace geolio
{
    template <GEO::index_t DIM>
    MeshElementManager<DIM>::MeshElementManager(
        GEO::Mesh& _mesh
        ) : mesh(_mesh),
            attribute_id_(generate_random_string(22))
    {
        assert(mesh.facets.are_simplices());

        /* Bind attributes */
        mesh_v_boundary.bind(mesh.vertices.attributes(), attribute_id_+":boundary");
        mesh_v_boundary.fill(false);
        mesh_v_fixed.bind(mesh.vertices.attributes(), attribute_id_+":fixed");
        mesh_v_fixed.fill(false);
        mesh_v_non_manifold.bind(mesh.vertices.attributes(), attribute_id_+":non-manifold");
        mesh_v_non_manifold.fill(false);
        mesh_v_used.bind(mesh.vertices.attributes(), attribute_id_+":used");
        mesh_v_used.fill(true);
        mesh_f_used.bind(mesh.facets.attributes(), attribute_id_+":used");
        mesh_f_used.fill(true);
    }

    template <GEO::index_t DIM>
    MeshElementManager<DIM>::~MeshElementManager(
        ) {
        /* Destroy attributes */
        if (mesh_v_boundary.is_bound())
            mesh_v_boundary.destroy();
        if (mesh_v_fixed.is_bound())
            mesh_v_fixed.destroy();
        if (mesh_v_non_manifold.is_bound())
            mesh_v_non_manifold.destroy();
        if (mesh_v_used.is_bound())
            mesh_v_used.destroy();
        if (mesh_f_used.is_bound())
            mesh_f_used.destroy();
    }

    template <GEO::index_t DIM>
    void MeshElementManager<DIM>::clean_unused_elements(
        const bool remove_isolated_vertices
        ) {
        if (free_facets_.empty() && free_vertices_.empty())
            return;

        GEO::vector<GEO::index_t> facets_to_delete(mesh.facets.nb(), 0);
        for (const auto& f : free_facets_) {
            if (f < mesh.facets.nb())
                facets_to_delete[f] = 1;
        }
        mesh.facets.delete_elements(facets_to_delete, remove_isolated_vertices);

        free_vertices_.clear();
        free_facets_.clear();

        assert(!remove_isolated_vertices ||
            std::ranges::all_of(mesh_v_used.get_vector(), [](const auto& b){ return b; }));
        assert(std::ranges::all_of(mesh_f_used.get_vector(), [](const auto& b){ return b; }));
    }

    template <GEO::index_t DIM>
    double MeshElementManager<DIM>::compute_average_mesh_edge_length(
        ) const {
        double l = 0;
        GEO::index_t edges_nb = 0;
        for (const auto& f : mesh.facets) {
            for (GEO::index_t lv = 0; lv < 3; ++lv) {
                l += get_edge_length(f, lv);
                ++edges_nb;
            }
        }

        return l / edges_nb;
    }

    template <GEO::index_t DIM>
    void MeshElementManager<DIM>::allocate_new_vertices(
        ) {
        assert(mesh.vertices.nb() > 0);

        const GEO::index_t PREV_MESH_VERTICES_NB = mesh.vertices.nb();
        const GEO::index_t ALLOCATE_MESH_VERTICES_NB = PREV_MESH_VERTICES_NB;

        mesh.vertices.create_vertices(ALLOCATE_MESH_VERTICES_NB);
        free_vertices_.reserve(free_vertices_.size() + ALLOCATE_MESH_VERTICES_NB);
        for (GEO::index_t v = PREV_MESH_VERTICES_NB, v_end = mesh.vertices.nb(); v < v_end; ++v)
            free_vertices_.push_back(v);
    }

    template <GEO::index_t DIM>
    void MeshElementManager<DIM>::allocate_new_facets(
        ) {
        assert(mesh.facets.nb() > 0);

        const GEO::index_t PREV_MESH_FACETS_NB = mesh.facets.nb();
        const GEO::index_t ALLOCATE_MESH_FACETS_NB = PREV_MESH_FACETS_NB;

        mesh.facets.create_triangles(ALLOCATE_MESH_FACETS_NB);
        free_facets_.reserve(free_facets_.size() + ALLOCATE_MESH_FACETS_NB);
        for (GEO::index_t f = PREV_MESH_FACETS_NB, f_end = mesh.facets.nb(); f < f_end; ++f)
            free_facets_.push_back(f);
    }

    template class MeshElementManager<2>;
    template class MeshElementManager<3>;
}