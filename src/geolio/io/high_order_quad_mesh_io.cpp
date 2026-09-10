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
            default: return order+41;
        }
    }

    void generate_msh_nodes_order(
        const GEO::index_t order,
        std::vector<GEO::index_t>& nodes_order
        ) {
        assert(order > 0);
        const auto n = order+1;

        std::vector<std::vector<GEO::index_t>> grid(n, std::vector<GEO::index_t>(n, 0));

        GEO::index_t current_nd = 0;
        for (GEO::index_t k = 0, k_end = (n+1)/2; k < k_end; ++k) {
            if (const GEO::index_t M = n - 2 * k;
                M == 1) {
                grid[k][k] = current_nd++;
                break;
            }

            grid[k][k]         = current_nd++;
            grid[k][n-1-k]     = current_nd++;
            grid[n-1-k][n-1-k] = current_nd++;
            grid[n-1-k][k]     = current_nd++;

            for (GEO::index_t x = k+1; x <= n-2-k; ++x)
                grid[k][x] = current_nd++;

            for (GEO::index_t y = k+1; y <= n-2-k; ++y)
                grid[y][n-1-k] = current_nd++;

            for (int x = static_cast<int>(n-2-k); x >= static_cast<int>(k+1); --x)
                grid[n-1-k][x] = current_nd++;

            for (int y = static_cast<int>(n-2-k); y >= static_cast<int>(k+1); --y)
                grid[y][k] = current_nd++;
        }

        nodes_order.clear();
        nodes_order.reserve(n * n);
        for (GEO::index_t y = 0; y < n; ++y) {
            for (GEO::index_t x = 0; x < n; ++x)
                nodes_order.push_back(grid[y][x]);
        }
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

        /* == Nodes ================================================================================================ */
    }

    template<GEO::index_t DIM>
    void high_order_quad_mesh_save_2_2(
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
            std::vector<GEO::index_t> ordered_nd(nodes_order.size()); // pre-allocated

            out << ELEMENTS_BEGIN << "\n";
            out << mesh.facets.nb() << "\n";
            for (const auto& f : mesh.facets) {
                out << f+1 << " " << element_type_code << " " << "2" << " " << "1" << " " << "1" << " ";

                ordered_nd.assign(nodes_order.size(), GEO::NO_INDEX);
                for (GEO::index_t i = 0, i_end = nodes_order.size(); i < i_end; ++i)
                    ordered_nd[nodes_order[i]] = control_grid.facet_nd(f, i);
                assert(std::ranges::none_of(ordered_nd, [](const auto nd){ return nd == GEO::NO_INDEX; }));

                for (const GEO::index_t nd : ordered_nd)
                    out << nd+1 << " ";

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