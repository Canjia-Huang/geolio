//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/21.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_QUAD_OPERATIONS_H
#define GEOLIO_QUAD_OPERATIONS_H
#include <geogram/mesh/mesh.h>

namespace geolio
{
    /**
     * @brief Trace a quadrilateral sheet starting from a facet and local vertex.
     * @details The function follows facet-to-facet adjacency across the two edges
     *          opposite the starting edge, propagating the edge orientation as it
     *          traverses the mesh. Each visited facet is recorded together with
     *          the directed edge cuts that belong to the traced sheet. Traversal
     *          stops at mesh boundaries or when all reachable directed cuts have
     *          already been visited.
     * @param[in] mesh The quadrilateral surface mesh to query.
     * @param[in] start_f Index of the starting facet
     *                    (0..mesh.facets.nb()-1).
     * @param[in] start_lv Local vertex index in the starting facet (0..3).
     *                      The initial directed edge is from local vertex
     *                      @p start_lv to local vertex (start_lv+1)%4.
     * @param[out] sheet_quads Output vector containing pairs
     *             (facet_index, facet_cut_type) for all visited facets.
     *             The vector is cleared before population. Each facet_cut_type
     *             is an 8-bit unsigned integer whose low four bits encode
     *             directed cuts in the facet:
     *             - Bit 0: cut from facet vertex v0 to v1.
     *             - Bit 1: cut from facet vertex v1 to v0.
     *             - Bit 2: cut from facet vertex v0 to v3.
     *             - Bit 3: cut from facet vertex v3 to v0.
     *             The remaining bits are not used.
     * @note The output order is not guaranteed. The function does not validate
     *       whether the resulting sheet is topologically valid.
     */
    void find_sheet_quads(
        const GEO::Mesh& mesh,
        GEO::index_t start_f,
        GEO::index_t start_lv,
        std::vector<std::pair<GEO::index_t, GEO::Numeric::uint8>>& sheet_quads);
}

#endif //GEOLIO_QUAD_OPERATIONS_H
