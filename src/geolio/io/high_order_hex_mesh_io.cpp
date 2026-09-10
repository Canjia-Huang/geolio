//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/10.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "high_order_hex_mesh_io.h"
#include <geolio/common/log.h>

namespace
{
    const std::string MESH_FORMAT_BEGIN = "$MeshFormat";
    const std::string MESH_FORMAT_END = "$EndMeshFormat";
    const std::string ENTITIES_BEGIN = "$Entities";
    const std::string ENTITIES_END = "$EndEntities";
    constexpr GEO::index_t ENTITIES_0D_NUMBER = 0;
    constexpr GEO::index_t ENTITIES_1D_NUMBER = 0;
    constexpr GEO::index_t ENTITIES_2D_NUMBER = 0;
    constexpr GEO::index_t ENTITIES_3D_NUMBER = 1;
    constexpr GEO::index_t ENTITIES_TAG = 1;
    constexpr GEO::index_t PHYSICAL_GROUP_NUMBER = 1;
    constexpr GEO::index_t PHYSICAL_GROUP_TAD = 1;
    constexpr GEO::index_t BOUNDARY_CURVES_NUMBER = 0;
    const std::string NODES_BEGIN = "$Nodes";
    const std::string NODES_END = "$EndNodes";
    constexpr GEO::index_t NODES_ENTITY_BLOCK = 1;
    constexpr GEO::index_t NODES_ENTITY_DIMENSION = 3;
    constexpr GEO::index_t NODES_ENTITY_ID = 1;
    constexpr GEO::index_t NODES_CONTAIM_PARAMETRIC_COORDINATES = 0;
    const std::string ELEMENTS_BEGIN = "$Elements";
    const std::string ELEMENTS_END = "$EndElements";
    constexpr GEO::index_t ELEMENTS_ENTITY_BLOCK = 1;
    constexpr GEO::index_t ELEMENTS_ENTITY_DIMENSION = 3;
    constexpr GEO::index_t ELEMENTS_NUMBERS_OF_TAGS = 2;
    constexpr GEO::index_t ELEMENTS_PHYSICAL_GROUP_ID = 1;
    constexpr GEO::index_t ELEMENTS_ENTITY_ID = 1;

    /**
     * @brief Gets the Gmsh element type code for a hexahedral element of the given order.
     *
     * @param order The polynomial order of the hexahedral element.
     * @return The corresponding Gmsh element type code.
     */
    GEO::index_t get_element_type_code(const GEO::index_t order) {
        assert(order > 0);
        switch (order) {
            case 1: return 5;
            case 2: return 12;
            default: return order+89; // right?
        }
    }

    /**
     * @brief Generates the node permutation from the internal order to the Gmsh MSH order.
     *
     * @param order The polynomial order of the hexahedral element.
     * @param nodes_order The output node order mapping, indexed by internal node order and storing the corresponding MSH index.
     */
    void generate_msh_nodes_order(
        const GEO::index_t order,
        std::vector<GEO::index_t>& nodes_order
        ) {
        assert(order > 0);
        const auto n = order + 1;
        const auto total_nodes = n * n * n;

        std::vector<GEO::index_t> msh_nodes_order(total_nodes, GEO::NO_INDEX);

        auto idx = [n](const GEO::index_t x, const GEO::index_t y, const GEO::index_t z) -> GEO::index_t {
            return z * n * n + y * n + x;
        };

        auto order_quad_nodes = [](const GEO::index_t S, auto&& visit) {
            for (GEO::index_t p = 0, p_end = (S + 1) / 2; p < p_end; ++p) {
                if (const GEO::index_t M = S - 2 * p;
                    M == 1
                    ) {
                    visit(p, p);
                    break;
                }

                visit(p, p);
                visit(S - 1 - p, p);
                visit(S - 1 - p, S - 1 - p);
                visit(p, S - 1 - p);

                for (GEO::index_t i = p + 1, i_end = S - 1 - p; i < i_end; ++i)
                    visit(i, p);
                for (GEO::index_t j = p + 1, j_end = S - 1 - p; j < j_end; ++j)
                    visit(S - 1 - p, j);
                for (int i = static_cast<int>(S - 2 - p), i_end = static_cast<int>(p); i > i_end; --i)
                    visit(static_cast<GEO::index_t>(i), S - 1 - p);
                for (int j = static_cast<int>(S - 2 - p), j_end = static_cast<int>(p); j > j_end; --j)
                    visit(p, static_cast<GEO::index_t>(j));
            }
        };

        GEO::index_t current_nd = 0;

        for (GEO::index_t k = 0, k_end = (n + 1) / 2; k < k_end; ++k) {
            const GEO::index_t M = n - 2 * k;

            if (M == 1) {
                msh_nodes_order[idx(k, k, k)] = current_nd++;
                break;
            }

            const GEO::index_t x0 = k, x1 = n - 1 - k;
            const GEO::index_t y0 = k, y1 = n - 1 - k;
            const GEO::index_t z0 = k, z1 = n - 1 - k;

            msh_nodes_order[idx(x0, y0, z0)] = current_nd++; // V0
            msh_nodes_order[idx(x1, y0, z0)] = current_nd++; // V1
            msh_nodes_order[idx(x1, y1, z0)] = current_nd++; // V2
            msh_nodes_order[idx(x0, y1, z0)] = current_nd++; // V3
            msh_nodes_order[idx(x0, y0, z1)] = current_nd++; // V4
            msh_nodes_order[idx(x1, y0, z1)] = current_nd++; // V5
            msh_nodes_order[idx(x1, y1, z1)] = current_nd++; // V6
            msh_nodes_order[idx(x0, y1, z1)] = current_nd++; // V7

            if (M > 2) {
                // Edge 0:  V0(0) -> V1(1)
                for (GEO::index_t x = x0 + 1; x < x1; ++x)
                    msh_nodes_order[idx(x, y0, z0)] = current_nd++;

                // Edge 1:  V0(0) -> V3(3)
                for (GEO::index_t y = y0 + 1; y < y1; ++y)
                    msh_nodes_order[idx(x0, y, z0)] = current_nd++;

                // Edge 2:  V0(0) -> V4(4)
                for (GEO::index_t z = z0 + 1; z < z1; ++z)
                    msh_nodes_order[idx(x0, y0, z)] = current_nd++;

                // Edge 3:  V1(1) -> V2(2)
                for (GEO::index_t y = y0 + 1; y < y1; ++y)
                    msh_nodes_order[idx(x1, y, z0)] = current_nd++;

                // Edge 4:  V1(1) -> V5(5)
                for (GEO::index_t z = z0 + 1; z < z1; ++z)
                    msh_nodes_order[idx(x1, y0, z)] = current_nd++;

                // Edge 5:  V2(2) -> V3(3)
                for (int x = static_cast<int>(x1 - 1), x_end = static_cast<int>(x0); x > x_end; --x)
                    msh_nodes_order[idx(static_cast<GEO::index_t>(x), y1, z0)] = current_nd++;

                // Edge 6:  V2(2) -> V6(6)
                for (GEO::index_t z = z0 + 1; z < z1; ++z)
                    msh_nodes_order[idx(x1, y1, z)] = current_nd++;

                // Edge 7:  V3(3) -> V7(7)
                for (GEO::index_t z = z0 + 1; z < z1; ++z)
                    msh_nodes_order[idx(x0, y1, z)] = current_nd++;

                // Edge 8:  V4(4) -> V5(5)
                for (GEO::index_t x = x0 + 1; x < x1; ++x)
                    msh_nodes_order[idx(x, y0, z1)] = current_nd++;

                // Edge 9:  V4(4) -> V7(7)
                for (GEO::index_t y = y0 + 1; y < y1; ++y)
                    msh_nodes_order[idx(x0, y, z1)] = current_nd++;

                // Edge 10: V5(5) -> V6(6)
                for (GEO::index_t y = y0 + 1; y < y1; ++y)
                    msh_nodes_order[idx(x1, y, z1)] = current_nd++;

                // Edge 11: V6(6) -> V7(7)
                for (int x = static_cast<int>(x1 - 1), x_end = static_cast<int>(x0); x > x_end; --x)
                    msh_nodes_order[idx(static_cast<GEO::index_t>(x), y1, z1)] = current_nd++;
            }

            if (M > 2) {
                const GEO::index_t S = M - 2;

                // Face 0
                order_quad_nodes(S, [&](const GEO::index_t i, const GEO::index_t j) {
                    msh_nodes_order[idx(x0 + 1 + j, y0 + 1 + i, z0)] = current_nd++;
                });

                // Face 1
                order_quad_nodes(S, [&](const GEO::index_t i, const GEO::index_t j) {
                    msh_nodes_order[idx(x0 + 1 + i, y0, z0 + 1 + j)] = current_nd++;
                });

                // Face 2
                order_quad_nodes(S, [&](const GEO::index_t i, const GEO::index_t j) {
                    msh_nodes_order[idx(x0, y0 + 1 + j, z0 + 1 + i)] = current_nd++;
                });

                // Face 3
                order_quad_nodes(S, [&](const GEO::index_t i, const GEO::index_t j) {
                    msh_nodes_order[idx(x1, y0 + 1 + i, z0 + 1 + j)] = current_nd++;
                });

                // Face 4
                order_quad_nodes(S, [&](const GEO::index_t i, const GEO::index_t j) {
                    msh_nodes_order[idx(x1 - 1 - i, y1, z0 + 1 + j)] = current_nd++;
                });

                // Face 5
                order_quad_nodes(S, [&](const GEO::index_t i, const GEO::index_t j) {
                    msh_nodes_order[idx(x0 + 1 + i, y0 + 1 + j, z1)] = current_nd++;
                });
            }
        }

        assert(std::ranges::none_of(msh_nodes_order, [](const auto i) { return i == GEO::NO_INDEX; }));

        nodes_order.assign(total_nodes, GEO::NO_INDEX);
        for (GEO::index_t i = 0, i_end = nodes_order.size(); i < i_end; ++i)
            nodes_order[msh_nodes_order[i]] = i;
    }
}

namespace geolio
{
    static bool high_order_hex_mesh_save_2_2(
        const HexControlGrid& control_grid,
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
                for (GEO::index_t d = 0; d < 3; ++d)
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
            assert(nodes_order.size() == (order+1)*(order+1)*(order+1));

            out << ELEMENTS_BEGIN << "\n";
            out << mesh.cells.nb() << "\n";
            for (const auto& c : mesh.cells) {
                out << c+1 << " "
                    << element_type_code << " "
                    << ELEMENTS_NUMBERS_OF_TAGS << " "
                    << ELEMENTS_PHYSICAL_GROUP_ID << " "
                    << ELEMENTS_ENTITY_ID << " ";
                for (const auto i : nodes_order)
                    out << control_grid.cell_nd(c, i)+1 << " ";
                out << "\n";
            }
            out << ELEMENTS_END << "\n";
        }

        return true;
    }

    static bool high_order_hex_mesh_save_4_1(
        const HexControlGrid& control_grid,
        std::ofstream& out
        ) {
        /* == Mesh format ========================================================================================== */
        {
            out << MESH_FORMAT_BEGIN << "\n";
            out << "4.1 0 8" << "\n";
            out << MESH_FORMAT_END << "\n";
        }

        /* == Entities ============================================================================================= */
        {
            out << ENTITIES_BEGIN << "\n";
            out << ENTITIES_0D_NUMBER << " "
                << ENTITIES_1D_NUMBER << " "
                << ENTITIES_2D_NUMBER << " "
                << ENTITIES_3D_NUMBER << "\n";

            std::vector<double> xyz_min{std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
            std::vector<double> xyz_max{std::numeric_limits<double>::min(), std::numeric_limits<double>::min(), std::numeric_limits<double>::min()};
            for (GEO::index_t nd = 0, nd_end = control_grid.control_nodes_nb(); nd < nd_end; ++nd) {
                const auto& p = control_grid.control_node(nd);
                for (GEO::index_t d = 0; d < 3; ++d) {
                    xyz_min[d] = std::min(xyz_min[d], p[d]);
                    xyz_max[d] = std::max(xyz_max[d], p[d]);
                }
            }
            out << ENTITIES_TAG << " ";
            out << xyz_min[0] << " " << xyz_min[1] << " " << xyz_min[2] << " " << xyz_max[0] << " " << xyz_max[1] << " " << xyz_max[2] << " ";
            out << PHYSICAL_GROUP_NUMBER << " "
                << PHYSICAL_GROUP_TAD << " "
                << BOUNDARY_CURVES_NUMBER << "\n";

            out << ENTITIES_END << "\n";
        }

        /* == Nodes ================================================================================================ */
        {
            out << NODES_BEGIN << "\n";

            out << NODES_ENTITY_BLOCK << " "
                << control_grid.control_nodes_nb() << " "
                << "1" << " " // min node idx
                << control_grid.control_nodes_nb() << "\n"; // max node idx

            out << NODES_ENTITY_DIMENSION << " "
                << NODES_ENTITY_ID << " "
                << NODES_CONTAIM_PARAMETRIC_COORDINATES << " "
                << control_grid.control_nodes_nb() << "\n";

            for (GEO::index_t nd = 0, nd_end = control_grid.control_nodes_nb(); nd < nd_end; ++nd)
                out << nd+1 << "\n";
            for (GEO::index_t nd = 0, nd_end = control_grid.control_nodes_nb(); nd < nd_end; ++nd) {
                const auto& p = control_grid.control_node(nd);
                for (GEO::index_t d = 0; d < 3; ++d)
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
            assert(nodes_order.size() == (order+1)*(order+1)*(order+1));

            out << ELEMENTS_BEGIN << "\n";

            out << ELEMENTS_ENTITY_BLOCK << " "
                << mesh.cells.nb() << " "
                << "1" << " "
                << mesh.cells.nb() << "\n";

            out << ELEMENTS_ENTITY_DIMENSION << " "
                << ELEMENTS_ENTITY_ID << " "
                << element_type_code << " "
                << mesh.cells.nb() << "\n";

            for (const auto& c : mesh.cells) {
                out << c+1 << " ";
                for (const auto i : nodes_order)
                    out << control_grid.cell_nd(c, i)+1 << " ";
                out << "\n";
            }

            out << ELEMENTS_END << "\n";
        }

        return true;
    }

    bool high_order_hex_mesh_save(
        const HexControlGrid& control_grid,
        const std::string& filepath,
        const std::string& version_number
        ) {
        std::ofstream out(filepath);
        if (!out.good()) {
            LOG::ERROR("Could not open file `{}` for writing!", filepath);
            return false;
        }

        if (version_number == "2.2")
            return high_order_hex_mesh_save_2_2(control_grid, out);
        if (version_number == "4.1")
            return high_order_hex_mesh_save_4_1(control_grid, out);
        LOG::ERROR("Unsupported version number `{}`", version_number);
        return false;
    }
}