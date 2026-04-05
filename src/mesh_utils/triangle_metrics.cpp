//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/3/21.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "triangle_metrics.h"
#include <cassert>

namespace GEO::MeshUtils
{
    double get_triangle_minimum_angle(
        const GEO::Mesh& M,
        const GEO::index_t f
        ) {
        assert(f < M.facets.nb());
        const auto& p0 = M.facets.point(f, 0);
        const auto& p1 = M.facets.point(f, 1);
        const auto& p2 = M.facets.point(f, 2);
        return std::min({
            GEO::Geom::angle(p1-p0, p2-p0),
            GEO::Geom::angle(p0-p1, p2-p1),
            GEO::Geom::angle(p0-p2, p1-p2)});
    }
}
