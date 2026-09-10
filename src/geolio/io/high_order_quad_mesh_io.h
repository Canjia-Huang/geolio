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
    bool high_order_quad_mesh_save(
        const QuadControlGrid<DIM>& control_grid,
        const std::string& filepath,
        const std::string& version_number = "2.2");

    extern template bool high_order_quad_mesh_save<2>(const QuadControlGrid<2>& control_grid, const std::string& filepath, const std::string& version_number);
    extern template bool high_order_quad_mesh_save<3>(const QuadControlGrid<3>& control_grid, const std::string& filepath, const std::string& version_number);

    template<GEO::index_t DIM>
    bool high_order_quad_mesh_load(
        const std::string& filepath,
        GEO::Mesh& mesh,
        std::unique_ptr<QuadControlGrid<DIM>>& control_grid_ptr);

    extern template bool high_order_quad_mesh_load<2>(const std::string& filepath, GEO::Mesh& mesh, std::unique_ptr<QuadControlGrid<2>>& control_grid_ptr);
    extern template bool high_order_quad_mesh_load<3>(const std::string& filepath, GEO::Mesh& mesh, std::unique_ptr<QuadControlGrid<3>>& control_grid_ptr);
}

#endif //GEOLIO_HIGH_ORDER_QUAD_MESH_IO_H
