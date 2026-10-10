//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/9.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_HIGH_ORDER_QUAD_MESH_IO_H
#define GEOLIO_HIGH_ORDER_QUAD_MESH_IO_H
#include <geolio/high_order/quad_control_grid.h>
#include <memory>

namespace geolio
{
    /**
     * @brief Loads a high-order quadrilateral mesh from a Gmsh file into a control grid.
     *
     * @param[in] filepath The input Gmsh file path.
     * @param[out] mesh The mesh object to populate with the loaded geometry.
     * @param[out] control_grid_ptr The pointer that receives the reconstructed control grid.
     * @return True if the file is loaded successfully; otherwise, false.
     */
    template <GEO::index_t MESH_DIM, GEO::index_t QUANTITIES_DIM>
    bool high_order_quad_mesh_load(
        const std::string& filepath,
        GEO::Mesh& mesh,
        std::unique_ptr<QuadControlGrid<MESH_DIM, QUANTITIES_DIM>>& control_grid_ptr);

    extern template bool high_order_quad_mesh_load<2, 0>(const std::string& filepath, GEO::Mesh& mesh, std::unique_ptr<QuadControlGrid<2, 0>>& control_grid_ptr);
    extern template bool high_order_quad_mesh_load<2, 1>(const std::string& filepath, GEO::Mesh& mesh, std::unique_ptr<QuadControlGrid<2, 1>>& control_grid_ptr);
    extern template bool high_order_quad_mesh_load<2, 2>(const std::string& filepath, GEO::Mesh& mesh, std::unique_ptr<QuadControlGrid<2, 2>>& control_grid_ptr);
    extern template bool high_order_quad_mesh_load<2, 3>(const std::string& filepath, GEO::Mesh& mesh, std::unique_ptr<QuadControlGrid<2, 3>>& control_grid_ptr);
    extern template bool high_order_quad_mesh_load<3, 0>(const std::string& filepath, GEO::Mesh& mesh, std::unique_ptr<QuadControlGrid<3, 0>>& control_grid_ptr);
    extern template bool high_order_quad_mesh_load<3, 1>(const std::string& filepath, GEO::Mesh& mesh, std::unique_ptr<QuadControlGrid<3, 1>>& control_grid_ptr);
    extern template bool high_order_quad_mesh_load<3, 2>(const std::string& filepath, GEO::Mesh& mesh, std::unique_ptr<QuadControlGrid<3, 2>>& control_grid_ptr);
    extern template bool high_order_quad_mesh_load<3, 3>(const std::string& filepath, GEO::Mesh& mesh, std::unique_ptr<QuadControlGrid<3, 3>>& control_grid_ptr);

    /**
     * @brief Saves a high-order quadrilateral control grid to a Gmsh mesh file.
     *
     * @param[in] control_grid The control grid to be exported.
     * @param[in] filepath The destination file path.
     * @param[in] version_number The Gmsh mesh version string, such as "2.2" or "4.1".
     * @return True if the mesh is written successfully; otherwise, false.
     */
    template <GEO::index_t MESH_DIM, GEO::index_t QUANTITIES_DIM>
    bool high_order_quad_mesh_save(
        const QuadControlGrid<MESH_DIM, QUANTITIES_DIM>& control_grid,
        const std::string& filepath,
        const std::string& version_number = "2.2");

    extern template bool high_order_quad_mesh_save<2, 0>(const QuadControlGrid<2, 0>& control_grid, const std::string& filepath, const std::string& version_number);
    extern template bool high_order_quad_mesh_save<2, 1>(const QuadControlGrid<2, 1>& control_grid, const std::string& filepath, const std::string& version_number);
    extern template bool high_order_quad_mesh_save<2, 2>(const QuadControlGrid<2, 2>& control_grid, const std::string& filepath, const std::string& version_number);
    extern template bool high_order_quad_mesh_save<2, 3>(const QuadControlGrid<2, 3>& control_grid, const std::string& filepath, const std::string& version_number);
    extern template bool high_order_quad_mesh_save<3, 0>(const QuadControlGrid<3, 0>& control_grid, const std::string& filepath, const std::string& version_number);
    extern template bool high_order_quad_mesh_save<3, 1>(const QuadControlGrid<3, 1>& control_grid, const std::string& filepath, const std::string& version_number);
    extern template bool high_order_quad_mesh_save<3, 2>(const QuadControlGrid<3, 2>& control_grid, const std::string& filepath, const std::string& version_number);
    extern template bool high_order_quad_mesh_save<3, 3>(const QuadControlGrid<3, 3>& control_grid, const std::string& filepath, const std::string& version_number);
}

#endif //GEOLIO_HIGH_ORDER_QUAD_MESH_IO_H
