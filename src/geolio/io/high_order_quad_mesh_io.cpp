//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/9.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "high_order_quad_mesh_io.h"

namespace
{
    const std::string MESH_FORMAT_BEGIN = "$MeshFormat";
    const std::string MESH_FORMAT_END = "$EndMeshFormat";
    const std::string NODES_BEGIN = "$Nodes";
    const std::string NODES_END = "$EndNodes";
    const std::string ELEMENTS_BEGIN = "$Elements";
    const std::string ELEMENTS_END = "$EndElements";
    const GEO::index_t NUMBERS_OF_TAGS = 2;
    const GEO::index_t PHYSICAL_GROUP_ID = 1;
    const GEO::index_t ELEMENTARY_ENTITY_ID = 1;

    GEO::index_t get_element_type_code(const GEO::index_t order) {
        assert(order > 0);
        switch (order) {
            case 1: return 3;
            case 2: return 10;
            case 3: return 36;
            case 4: return 37;
            case 5: return 38;
            case 6: return 47;
            case 7: return 48;
            case 8: return 49;
            case 9: return 50;
            case 10: return 51;
            default: return order+41; // right?
        }
    }

    void generate_msh_nodes_order(
        const GEO::index_t order,
        std::vector<GEO::index_t>& nodes_order
        ) {
        assert(order > 0);
        const auto n = order+1;

        std::vector<GEO::index_t> msh_nodes_order(n*n, GEO::NO_INDEX);

        GEO::index_t current_nd = 0;
        for (GEO::index_t k = 0, k_end = (n+1)/2; k < k_end; ++k) {
            if (const GEO::index_t M = n - 2 * k;
                M == 1
                ) {
                msh_nodes_order[k*n+k] = current_nd++;
                break;
            }

            msh_nodes_order[k*n+k]           = current_nd++;
            msh_nodes_order[k*n+n-1-k]       = current_nd++;
            msh_nodes_order[(n-1-k)*n+n-1-k] = current_nd++;
            msh_nodes_order[(n-1-k)*n+k]     = current_nd++;

            for (GEO::index_t x = k+1, x_end = n-1-k; x < x_end; ++x)
                msh_nodes_order[k*n+x] = current_nd++;

            for (GEO::index_t y = k+1, y_end = n-1-k; y < y_end; ++y)
                msh_nodes_order[y*n+n-1-k] = current_nd++;

            for (int x = static_cast<int>(n-2-k), x_end = static_cast<int>(k); x > x_end; --x)
                msh_nodes_order[(n-1-k)*n+x] = current_nd++;

            for (int y = static_cast<int>(n-2-k), y_end = static_cast<int>(k); y > y_end; --y)
                msh_nodes_order[y*n+k] = current_nd++;
        }
        assert(std::ranges::none_of(msh_nodes_order, [](const auto i){ return i == GEO::NO_INDEX; }));

        nodes_order.assign(n*n, GEO::NO_INDEX);
        for (GEO::index_t i = 0, i_end = nodes_order.size(); i < i_end; ++i)
            nodes_order[msh_nodes_order[i]] = i;
    }
}

namespace geolio
{
    template<GEO::index_t DIM>
    void high_order_quad_mesh_save_4_1(
        const QuadControlGrid<DIM>& control_grid,
        std::ofstream& out
        ) {
        /* == Mesh format ========================================================================================== */
        out << MESH_FORMAT_BEGIN << "\n";
        out << "4.1 0 8" << "\n";
        out << MESH_FORMAT_END << "\n";

        /* == Entities ============================================================================================= */

        /* == Nodes ================================================================================================ */
    }

    template<GEO::index_t DIM>
    static void high_order_quad_mesh_save_2_2(
        const QuadControlGrid<DIM>& control_grid,
        std::ofstream& out
        ) {
        /* == Mesh format ========================================================================================== */
        {
            out << MESH_FORMAT_BEGIN << "\n";
            out << "2.2 0 8" << "\n";
            out << MESH_FORMAT_END << "\n";
        }

        /* == Nodes ================================================================================================ */
        {
            out << NODES_BEGIN << "\n";
            out << control_grid.control_nodes_nb() << "\n";
            for (GEO::index_t nd = 0, nd_end = control_grid.control_nodes_nb(); nd < nd_end; ++nd) {
                const auto& p = control_grid.control_node(nd);
                out << nd+1 << " ";
                for (GEO::index_t d = 0; d < DIM; ++d)
                    out << p[d] << " ";
                if constexpr (DIM == 2) // The gmsh2.2 format does not support 2D nodes, so a z-coordinate of “0” is added.
                    out << "0";
                out << "\n";
            }
            out << NODES_END << "\n";
        }

        /* == Elements ============================================================================================= */
        {
            const auto& mesh = control_grid.mesh();
            const auto order = control_grid.order();
            const auto element_type_code = get_element_type_code(order);

            std::vector<GEO::index_t> nodes_order;
            generate_msh_nodes_order(order, nodes_order);
            assert(nodes_order.size() == (order+1)*(order+1));

            out << ELEMENTS_BEGIN << "\n";
            out << mesh.facets.nb() << "\n";
            for (const auto& f : mesh.facets) {
                out << f+1 << " "
                    << element_type_code << " "
                    << NUMBERS_OF_TAGS << " "
                    << PHYSICAL_GROUP_ID << " "
                    << ELEMENTARY_ENTITY_ID << " ";
                for (const auto i : nodes_order)
                    out << control_grid.facet_nd(f, i)+1 << " ";
                out << "\n";
            }
            out << ELEMENTS_END << "\n";
        }
    }

    template<GEO::index_t DIM>
    void high_order_quad_mesh_save(
        const QuadControlGrid<DIM>& control_grid,
        const std::string& filepath,
        const std::string& version_number
        ) {
        std::ofstream out(filepath);
        if (!out.good())
            throw std::runtime_error("Could not open file `"+filepath+"` for writing");

        if (version_number == "2.2")
            high_order_quad_mesh_save_2_2(control_grid, out);
        else if (version_number == "4.1")
            high_order_quad_mesh_save_4_1(control_grid, out);
        else
            throw std::logic_error("Unsupported version number `"+version_number+"`");
    }

    template void high_order_quad_mesh_save<2>(const QuadControlGrid<2>& control_grid, const std::string& filepath, const std::string& version_number);
    template void high_order_quad_mesh_save<3>(const QuadControlGrid<3>& control_grid, const std::string& filepath, const std::string& version_number);
}