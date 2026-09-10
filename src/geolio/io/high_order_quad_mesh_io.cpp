//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/9.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "high_order_quad_mesh_io.h"

namespace
{
    const std::string MESH_FORMAT_BEGIN = "$MeshFormat";
    const std::string MESH_FORMAT_END = "$EndMeshFormat";
}

namespace geolio
{
    template<GEO::index_t DIM>
    void save_high_order_quad_mesh(
        const QuadControlGrid<DIM>& control_grid,
        const std::string& filepath,
        const std::string& version_number
        ) {
        std::ofstream out(filepath);
        if (!out.good())
            throw std::runtime_error("Could not open file `"+filepath+"` for writing");

        /* == Mesh format ========================================================================================== */
        out << MESH_FORMAT_BEGIN << "\n";
        if (version_number == "2.2")
            out << "2.2 0 8" << "\n";
        else if (version_number == "4.1")
            out << "4.1 0 8" << "\n";
        else
            throw std::logic_error("Unsupported version number `"+version_number+"`");
        out << MESH_FORMAT_END << "\n";

        /* == Nodes ================================================================================================ */

    }
}