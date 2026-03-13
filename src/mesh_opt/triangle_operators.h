//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/3/13.
// Copyright (c) 2026 Graphics@XMU. All rights reserved.
//
#ifndef MESHOPT_TRIANGLE_OPERATORS_H
#define MESHOPT_TRIANGLE_OPERATORS_H

#include <geogram/mesh/mesh.h>

namespace MeshOpt
{
    /**
     * @brief Split an edge of a triangle in a mesh and update the adjacency topology accordingly.
     *
     * Given triangle facet @p f and local vertex index @p lv, a new vertex @p new_v is inserted
     * on the edge (lv → lv+1) at interpolation ratio @p r, splitting @p f into two triangles
     * (@p f and @p new_f1). If the adjacent facet across that edge exists (af != NO_FACET),
     * it is simultaneously split into two triangles (@p af and @p new_f2) to keep the mesh
     * topology consistent.
     *
     * @param[in, out] M The target mesh. Vertex and facet storage must be pre-allocated.
     * @param[in] f Index of the triangle facet to split.
     * @param[in] lv Local vertex index (0, 1, or 2) that identifies the edge to split (lv → lv+1).
     * @param[in] new_v Index of the pre-allocated new vertex. Its position will be set to (1-r)*p(lv) + r*p(lv+1).
     * @param[in] r Interpolation ratio in [0, 1] controlling where the new vertex is placed along the edge.
     * @param[in] new_f1 Index of the pre-allocated new facet produced by splitting @p f.
     * @param[in] new_f2 Index of the pre-allocated new facet produced by splitting the adjacent facet @p af.
     * Ignored when @p af does not exist (NO_FACET).
     */
     void triangle_split_edge(
        GEO::Mesh& M,
        GEO::index_t f,
        GEO::index_t lv,
        GEO::index_t new_v,
        double r,
        GEO::index_t new_f1,
        GEO::index_t new_f2);
}

#endif //MESHOPT_TRIANGLE_OPERATORS_H