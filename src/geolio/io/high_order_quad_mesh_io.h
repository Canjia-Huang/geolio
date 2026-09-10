//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/9.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_HIGH_ORDER_QUAD_MESH_IO_H
#define GEOLIO_HIGH_ORDER_QUAD_MESH_IO_H
#include <geolio/high_order/quad_control_grid.h>

namespace geolio
{
    template<GEO::index_t DIM>
    void save_high_order_quad_mesh(
        const QuadControlGrid<DIM>& control_grid,
        const std::string& filepath,
        const std::string& version_number = "2.2"
        );
}

#endif //GEOLIO_HIGH_ORDER_QUAD_MESH_IO_H
