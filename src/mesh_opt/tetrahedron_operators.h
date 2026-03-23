//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/3/23.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef PROGRESSIVEMESHOPT_TETRAHEDRON_OPERATORS_H
#define PROGRESSIVEMESHOPT_TETRAHEDRON_OPERATORS_H

#include <geogram/mesh/mesh.h>

namespace ProgressiveMeshOpt::Tet
{
    /**
     * Performs a 2-3 edge swap operation on a tetrahedral mesh.
     *
     * This operation replaces 2 tetrahedra sharing a common facet with 3 tetrahedra
     * by flipping the shared facet. Given a cell @p c with a local facet @p lf,
     * this function identifies the adjacent cell across that facet and performs
     * the topological transformation.
     *
     * @param[in,out] M          The tetrahedral mesh to modify
     * @param[in]     c          Index of the first cell (tetrahedron) to swap
     * @param[in]     lf         Local facet index (0-3) of cell @p c defining the swap edge
     * @param[in,out] new_c      Index of the newly created cell for the 2-3 swap
     */
    void edge_swap_2_3(
        GEO::Mesh& M,
        GEO::index_t c,
        GEO::index_t lf,
        GEO::index_t new_c);
}

#endif //PROGRESSIVEMESHOPT_TETRAHEDRON_OPERATORS_H