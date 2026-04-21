//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/3/31.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOGRAM_MESH_UTILS_HEXAHEDRON_OPERATIONS_H
#define GEOGRAM_MESH_UTILS_HEXAHEDRON_OPERATIONS_H
#include <geogram/mesh/mesh.h>
#include <cassert>

namespace GEO::MeshUtils::Hex
{
    /**
     * Finds the local vertex index in a hexahedron from a global vertex index.
     *
     * @param[in] M  The hexahedral mesh to query
     * @param[in] c  Index of the hexahedral cell to search
     * @param[in] v  Global vertex index to locate in cell @p c
     * @return Local vertex index (0-7) in @p c if found; otherwise GEO::NO_INDEX
     */
    inline GEO::index_t find_hex_vertex(
        const GEO::Mesh& M,
        const GEO::index_t c,
        const GEO::index_t v
        ) {
        assert(c < M.cells.nb());
        for (GEO::index_t lv = 0; lv < 8; ++lv) {
            if (M.cells.vertex(c, lv) == v)
                return lv;
        }
        return GEO::NO_INDEX;
    }

    /**
     * Finds the local edge index in a hexahedron from two local endpoint vertices.
     *
     * Edge direction is ignored.
     *
     * @param[in] lv0 Local vertex index (0-7) of one endpoint
     * @param[in] lv1 Local vertex index (0-7) of the other endpoint
     * @return Local edge index (0-11) if @p lv0 and @p lv1 form a hexahedron edge; otherwise GEO::NO_INDEX
     */
    GEO::index_t find_hex_edge_from_local_vertices(
        GEO::index_t lv0,
        GEO::index_t lv1);

    /**
     * Finds the local edge index in a hexahedron from two endpoint vertex.
     *
     * The search is performed in cell @p c using global vertex indices @p ev0 and @p ev1.
     * Edge direction is ignored.
     *
     * @param[in] M    The hexahedral mesh to query
     * @param[in] c    Index of the hexahedral cell to search
     * @param[in] v0  Global vertex index of one endpoint
     * @param[in] v1  Global vertex index of the other endpoint
     * @return Local edge index (0-11) in @p c if found; otherwise GEO::NO_INDEX
     */
    GEO::index_t find_hex_edge(
        const GEO::Mesh& M,
        GEO::index_t c,
        GEO::index_t v0,
        GEO::index_t v1);

    /**
     * Finds a local facet index from three local vertices of a hexahedron.
     *
     * The three local vertices can be provided in any order.
     *
     * @param[in] lv0 Local vertex index (0-7)
     * @param[in] lv1 Local vertex index (0-7)
     * @param[in] lv2 Local vertex index (0-7)
     * @return Local facet index (0-5) if the three vertices are on the same facet; otherwise GEO::NO_INDEX
     */
    GEO::index_t find_hex_facet_from_local_vertices(
        GEO::index_t lv0,
        GEO::index_t lv1,
        GEO::index_t lv2);

    /**
     * Finds a local facet index from four local vertices of a hexahedron.
     *
     * The four local vertices can be provided in any order.
     *
     * @param[in] lv0 Local vertex index (0-7)
     * @param[in] lv1 Local vertex index (0-7)
     * @param[in] lv2 Local vertex index (0-7)
     * @param[in] lv3 Local vertex index (0-7)
     * @return Local facet index (0-5) if the four vertices exactly define one facet; otherwise GEO::NO_INDEX
     */
    GEO::index_t find_hex_facet_from_local_vertices(
        GEO::index_t lv0,
        GEO::index_t lv1,
        GEO::index_t lv2,
        GEO::index_t lv3);

    /**
     * Finds the local facet index in a hexahedron from three global vertices.
     *
     * The three vertices can be provided in any order.
     *
     * @param[in] M   The hexahedral mesh to query
     * @param[in] c   Index of the hexahedral cell to search
     * @param[in] v0  Global vertex index on the target facet
     * @param[in] v1  Global vertex index on the target facet
     * @param[in] v2  Global vertex index on the target facet
     * @return Local facet index (0-5) in @p c if found; otherwise GEO::NO_INDEX
     */
    GEO::index_t find_hex_facet(
        const GEO::Mesh& M,
        GEO::index_t c,
        GEO::index_t v0,
        GEO::index_t v1,
        GEO::index_t v2);

    /**
     * Collects hexahedra incident to a mesh edge in traversal order.
     *
     * Starting from a cell-local-edge seed, this function traces the edge
     * ring (or border chain) and outputs an ordered list of
     * (cell, local-edge, local-facet) tuples, where each local facet contains
     * the queried edge.
     *
     * @param[in]     M                The hexahedral mesh to query
     * @param[in]     start_c          Index of the seed cell
     * @param[in]     start_le         Local edge index (0-11) in @p start_c
     * @param[in,out] ordered_c_le_lf  Output ordered (cell, local-edge, local-facet) list; existing contents are cleared
     * @return true if the queried edge is on the mesh border; false if it is an interior edge
     */
    bool get_edge_incident_hexahedra(
        const GEO::Mesh& M,
        GEO::index_t start_c,
        GEO::index_t start_le,
        std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>>& ordered_c_le_lf);
}

#endif //GEOGRAM_MESH_UTILS_HEXAHEDRON_OPERATIONS_H