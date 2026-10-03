//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/5/16.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_MESH_OPERATIONS_H
#define GEOLIO_MESH_OPERATIONS_H
#include <geogram/mesh/mesh.h>
#include <tuple>
#include <utility>
#include <vector>
#include <cassert>

namespace geolio
{
    /**
     * @brief Collect facets incident to a vertex in one-ring order.
     * @details Starting from the seed facet @p start_f and its local vertex slot @p start_lv,
     *          the function walks facet-to-facet adjacency links around the target vertex,
     *          recording ordered (facet, local vertex) pairs. For interior vertices the walk
     *          forms a closed ring; for border vertices it stops at the border and then walks
     *          in the opposite direction to order the list from one border side to the other.
     *          The traversal handles arbitrary polygonal (including hybrid) meshes. For
     *          non-manifold vertices, only the connected component reachable from the seed
     *          facet is explored.
     * @param[in] mesh Input mesh.
     * @param[in] start_f Seed facet index incident to the target vertex.
     * @param[in] start_lv Local vertex index of the target vertex in @p start_f.
     * @param[out] ordered_f_lv Output ordered one-ring list. Each element is (f, lv), where
     *                              @p f is an incident facet and @p lv is the local index of the target
     *                              vertex inside that facet. Existing contents are cleared.
     * @return true if the target vertex is on the mesh border; false if it is an interior vertex.
     */
    bool get_vertex_incident_facets(
        const GEO::Mesh& mesh,
        GEO::index_t start_f,
        GEO::index_t start_lv,
        std::vector<std::pair<GEO::index_t, GEO::index_t>>& ordered_f_lv);

    /**
     * @brief Collect cells incident to a vertex from a seed cell.
     * @details Starting from (@p start_c, @p start_lv), the function performs a
     *          depth-first search over cells sharing the target global vertex,
     *          using a stack and a processed-cell set. For each visited cell it
     *          outputs (cell index, local vertex index) and pushes the neighbors
     *          across every local facet that also contains the vertex. The traversal
     *          currently supports tetrahedra and hexahedra. For non-manifold
     *          configurations, only the component reachable from the seed cell is
     *          collected.
     * @param[in] mesh Input mesh.
     * @param[in] start_c Seed cell index incident to the target vertex.
     * @param[in] start_lv Local vertex index of the target vertex in @p start_c.
     * @param[out] c_and_lv Output incident list. Each element is (c, lv), where
     *                      @p c is an incident cell and @p lv is the local index
     *                      of the target vertex in that cell. Existing contents are cleared.
     * @return true if any incident side of the vertex reaches the border; false otherwise.
     */
    bool get_vertex_incident_cells(
        const GEO::Mesh& mesh,
        GEO::index_t start_c,
        GEO::index_t start_lv,
        std::vector<std::pair<GEO::index_t, GEO::index_t>>& c_and_lv);

    /**
     * @brief Collect edge-incident cells in ring/chain order from a seed local edge.
     * @details The edge is identified by local edge index @p start_le in cell @p start_c.
     *          The function walks cell-to-cell adjacency links across the local facets
     *          adjacent to the edge, recording ordered (c, le, lf) tuples, where @p lf is
     *          the facet used to move to the next cell. For interior edges the walk forms
     *          a closed loop; for border edges it additionally walks in the opposite
     *          direction to order the sequence from one border side to the other. Only
     *          tetrahedral and hexahedral cells are currently supported.
     * @param[in] mesh Input mesh.
     * @param[in] start_c Seed cell containing the target edge.
     * @param[in] start_le Local edge index in @p start_c.
     * @param[out] ordered_c_le_lf Output ordered incident list of (c, le, lf).
     *                              Existing contents are cleared.
     * @return true if the target edge is on the border; false if it is interior.
     */
    bool get_edge_incident_cells(
        const GEO::Mesh& mesh,
        GEO::index_t start_c,
        GEO::index_t start_le,
        std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>>& ordered_c_le_lf);

    /**
     * @brief Collect edge-incident cells in ring/chain order from a facet edge seed.
     * @details The target edge is derived from two consecutive facet vertices:
     *          `facet_vertex(start_c, start_lf, start_lv)` and
     *          `facet_vertex(start_c, start_lf, (start_lv+1)%N)`, where `N` is the
     *          number of vertices of the facet (3 for tetrahedra, 4 for hexahedra).
     *          The function then delegates to the local-edge overload.
     * @param[in] mesh Input mesh.
     * @param[in] start_c Seed cell index.
     * @param[in] start_lf Local facet index in @p start_c.
     * @param[in] start_lv Local vertex slot inside @p start_lf; together with the
     *                     next facet vertex defines the seed edge.
     * @param[out] ordered_c_le_lf Output ordered incident list of (c, le, lf).
     *                              Existing contents are cleared.
     * @return true if the derived edge is on the border; false if it is interior.
     */
    bool get_edge_incident_cells(
        const GEO::Mesh& mesh,
        GEO::index_t start_c,
        GEO::index_t start_lf,
        GEO::index_t start_lv,
        std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>>& ordered_c_le_lf);

    /**
     * @brief Normalize mesh vertices into a centered, unit-scale coordinate system.
     * @details Computes the axis-aligned bounding box of the mesh in the first
     *          @p DIM coordinates. The bounding-box center is written to @p center,
     *          the largest axis extent is written to @p scale, and every vertex is
     *          transformed in place by `(p - center) / scale`. Only 2D and 3D
     *          normalization are supported.
     * @param[in,out] mesh Mesh whose vertex coordinates are normalized in place.
     * @param[out] center Center of the mesh bounding box before normalization.
     * @param[out] scale Maximum extent of the mesh bounding box before normalization.
     */
    template <GEO::index_t DIM>
    void normalize(
        GEO::Mesh& mesh,
        GEO::vecng<DIM, double>& center,
        double& scale
        ) {
        static_assert(DIM == 2 || DIM == 3);
        assert(mesh.vertices.dimension() == DIM);

        std::array<double, DIM> xyz_min{};
        std::array<double, DIM> xyz_max{};
        for (GEO::index_t d = 0; d < DIM; ++d) {
            xyz_min[d] = std::numeric_limits<double>::max();
            xyz_max[d] = -std::numeric_limits<double>::max();
        }

        for (const auto& p : mesh.vertices.points<DIM>()) {
            for (GEO::index_t d = 0; d < DIM; ++d) {
                xyz_min[d] = std::min(xyz_min[d], p[d]);
                xyz_max[d] = std::max(xyz_max[d], p[d]);
            }
        }

        scale = 0;
        for (GEO::index_t d = 0; d < DIM; ++d) {
            center[d] = 0.5*(xyz_min[d]+xyz_max[d]);
            scale = std::max(scale, xyz_max[d]-xyz_min[d]);
        }

        for (auto& p : mesh.vertices.points<DIM>()) {
            p -= center;
            p /= scale;
        }
    }
}

#endif //GEOLIO_MESH_OPERATIONS_H
