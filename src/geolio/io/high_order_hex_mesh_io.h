//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/10.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_HIGH_ORDER_HEX_MESH_IO_H
#define GEOLIO_HIGH_ORDER_HEX_MESH_IO_H
#include <geolio/high_order/hex_control_grid.h>

namespace geolio
{
    bool high_order_hex_mesh_save(
        const HexControlGrid& control_grid,
        const std::string& filepath,
        const std::string& version_number = "2.2");

    bool high_order_hex_mesh_load(
        const std::string& filepath,
        GEO::Mesh& mesh,
        std::unique_ptr<HexControlGrid>& control_grid_ptr);
}

#endif //GEOLIO_HIGH_ORDER_HEX_MESH_IO_H
