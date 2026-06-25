//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/5/16.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOGRAMMESHUTILS_SURF_OPERATIONS_H
#define GEOGRAMMESHUTILS_SURF_OPERATIONS_H

#include <geogram/mesh/mesh.h>

namespace geolio
{
    /**
     * @brief Collect facets incident to a vertex in one-ring order.
     *
     * Starting from facet @p start_f and its local vertex slot @p start_lv, this function traverses
     * all incident triangles around that vertex and outputs ordered pairs (facet index, local vertex index).
     * For interior vertices, the sequence forms a closed ring. For border vertices, the sequence is ordered
     * from one border side to the other.
     *
     * @param[in] M Input triangle mesh.
     * @param[in] _f Seed facet index incident to the target vertex.
     * @param[in] _lv Local vertex index (0, 1, or 2) of the target vertex in @p start_f.
     * @param[out] ordered_f_and_lv Output ordered one-ring list. Each element is (f, lv), where
     *                              @p f is an incident facet and @p lv is the local index of the target
     *                              vertex inside that facet. Existing contents are cleared.
     * @return true if the target vertex is on the mesh border; false if it is an interior vertex.
     */
    bool get_vertex_incident_facets(
        const GEO::Mesh& M,
        const GEO::index_t _f,
        const GEO::index_t _lv,
        std::vector<std::pair<GEO::index_t, GEO::index_t>>& ordered_f_and_lv);
}

#endif //GEOGRAMMESHUTILS_SURF_OPERATIONS_H
