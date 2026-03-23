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

    /**
     * Splits one tetrahedron into four tetrahedra by inserting an interior vertex.
     *
     * The vertex index @p new_v is expected to be pre-allocated. Its position will be set
     * to the barycenter of cell @p c. The original cell @p c is updated in-place and
     * three additional tetrahedra (@p new_c0, @p new_c1, @p new_c2) are filled.
     * Adjacency between the four resulting cells and neighboring cells is updated.
     *
     * @param[in,out] M      The tetrahedral mesh to modify
     * @param[in]     c      Index of the tetrahedron to split
     * @param[in]     new_v  Index of a pre-allocated vertex used as the split vertex
     * @param[in]     new_c0 Index of the first pre-allocated tetrahedron created by the split
     * @param[in]     new_c1 Index of the second pre-allocated tetrahedron created by the split
     * @param[in]     new_c2 Index of the third pre-allocated tetrahedron created by the split
     */
    void cell_split(
        GEO::Mesh& M,
        GEO::index_t c,
        GEO::index_t new_v,
        GEO::index_t new_c0,
        GEO::index_t new_c1,
        GEO::index_t new_c2);
}

#endif //PROGRESSIVEMESHOPT_TETRAHEDRON_OPERATORS_H