//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/10.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_HIGH_ORDER_HEX_MESH_IO_H
#define GEOLIO_HIGH_ORDER_HEX_MESH_IO_H
#include <geolio/high_order/hex_control_grid.h>
#include <memory>

namespace geolio
{
    /**
     * @brief Saves a high-order hexahedral control grid to a Gmsh mesh file.
     *
     * @param[in] control_grid The hexahedral control grid to export.
     * @param[in] filepath The destination file path.
     * @param[in] version_number The Gmsh mesh version string, such as "2.2" or "4.1".
     * @return True if the mesh is written successfully; otherwise, false.
     */
    bool high_order_hex_mesh_save(
        const HexControlGrid& control_grid,
        const std::string& filepath,
        const std::string& version_number = "2.2");

    /**
     * @brief Loads a high-order hexahedral mesh from a Gmsh file and reconstructs the control grid.
     *
     * @param[in] filepath The input Gmsh file path.
     * @param[out] mesh The mesh object to populate with the loaded geometry.
     * @param[out] control_grid_ptr The pointer that receives the reconstructed control grid.
     * @return True if the file is loaded successfully; otherwise, false.
     */
    bool high_order_hex_mesh_load(
        const std::string& filepath,
        GEO::Mesh& mesh,
        std::unique_ptr<HexControlGrid>& control_grid_ptr);
}

#endif //GEOLIO_HIGH_ORDER_HEX_MESH_IO_H
