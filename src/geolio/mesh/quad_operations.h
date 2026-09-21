//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/21.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_QUAD_OPERATIONS_H
#define GEOLIO_QUAD_OPERATIONS_H
#include <geogram/mesh/mesh.h>

namespace geolio
{
    void find_sheet_quads(
        const GEO::Mesh& mesh,
        GEO::index_t start_f,
        GEO::index_t start_lv,
        std::vector<std::pair<GEO::index_t, GEO::Numeric::uint8>>& sheet_quads);
}

#endif //GEOLIO_QUAD_OPERATIONS_H
