//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/3/21.
// Copyright (c) 2026 Graphics@XMU. All rights reserved.
//
#ifndef MESHOPT_TRIANGLE_METRICS_H
#define MESHOPT_TRIANGLE_METRICS_H

#include <geogram/mesh/mesh.h>

namespace MeshOpt
{
    /**
     * @brief Compute the minimum interior angle of a triangle facet.
     *
     * The function evaluates the three corner angles of facet @p f and returns the smallest one.
     *
     * @param[in] M Input mesh.
     * @param[in] f Facet index of the target triangle.
     * @return Minimum interior angle of facet @p f (in radians).
     */
    double get_triangle_minimum_angle(
        const GEO::Mesh& M,
        GEO::index_t f);
}

#endif //MESHOPT_TRIANGLE_METRICS_H