//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/3/31.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOGRAM_MESH_UTILS_HEXAHEDRON_OPERATORS_H
#define GEOGRAM_MESH_UTILS_HEXAHEDRON_OPERATORS_H

#include <geogram/mesh/mesh.h>

namespace GEO::MeshUtils
{
    /**
     * Finds the local edge index in a hexahedron from two endpoint vertex.
     *
     * The search is performed in cell @p c using global vertex indices @p ev0 and @p ev1.
     * Edge direction is ignored.
     *
     * @param[in] M    The hexahedral mesh to query
     * @param[in] c    Index of the hexahedral cell to search
     * @param[in] ev0  Global vertex index of one endpoint
     * @param[in] ev1  Global vertex index of the other endpoint
     * @return Local edge index (0-11) in @p c if found; otherwise GEO::NO_INDEX
     */
    GEO::index_t find_hex_edge(
        const GEO::Mesh& M,
        GEO::index_t c,
        GEO::index_t ev0,
        GEO::index_t ev1);

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

#endif //GEOGRAM_MESH_UTILS_HEXAHEDRON_OPERATORS_H