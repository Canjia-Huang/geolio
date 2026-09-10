//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/9.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "high_order_quad_mesh_io.h"
#include <geolio/common/log.h>
#include "line_stream.h"

namespace
{
    const std::string MESH_FORMAT_BEGIN = "$MeshFormat";
    const std::string MESH_FORMAT_END = "$EndMeshFormat";
    const std::string ENTITIES_BEGIN = "$Entities";
    const std::string ENTITIES_END = "$EndEntities";
    constexpr GEO::index_t ENTITIES_0D_NUMBER = 0;
    constexpr GEO::index_t ENTITIES_1D_NUMBER = 0;
    constexpr GEO::index_t ENTITIES_2D_NUMBER = 1;
    constexpr GEO::index_t ENTITIES_3D_NUMBER = 0;
    constexpr GEO::index_t ENTITIES_TAG = 1;
    constexpr GEO::index_t PHYSICAL_GROUP_NUMBER = 1;
    constexpr GEO::index_t PHYSICAL_GROUP_TAD = 1;
    constexpr GEO::index_t BOUNDARY_CURVES_NUMBER = 0;
    const std::string NODES_BEGIN = "$Nodes";
    const std::string NODES_END = "$EndNodes";
    constexpr GEO::index_t NODES_ENTITY_BLOCK = 1;
    constexpr GEO::index_t NODES_ENTITY_DIMENSION = 2;
    constexpr GEO::index_t NODES_ENTITY_ID = 1;
    constexpr GEO::index_t NODES_CONTAIM_PARAMETRIC_COORDINATES = 0;
    const std::string ELEMENTS_BEGIN = "$Elements";
    const std::string ELEMENTS_END = "$EndElements";
    constexpr GEO::index_t ELEMENTS_ENTITY_BLOCK = 1;
    constexpr GEO::index_t ELEMENTS_ENTITY_DIMENSION = 2;
    constexpr GEO::index_t ELEMENTS_NUMBERS_OF_TAGS = 2;
    constexpr GEO::index_t ELEMENTS_PHYSICAL_GROUP_ID = 1;
    constexpr GEO::index_t ELEMENTS_ENTITY_ID = 1;

    /**
     * @brief Gets the Gmsh element type code for a quadrilateral element of the given order.
     *
     * @param order The polynomial order of the quadrilateral element.
     * @return The corresponding Gmsh element type code.
     */
    GEO::index_t get_element_type_code(const GEO::index_t order) {
        assert(order > 0);
        switch (order) {
            case 1: return 3;
            case 2: return 10;
            case 3: return 36;
            case 4: return 37;
            case 5: return 38;
            default: return order+41; // right?
        }
    }

    /**
     * @brief Recovers the polynomial order of a quadrilateral element from its Gmsh element type code.
     *
     * @param element_type_code The Gmsh element type code for the high-order quadrilateral.
     * @return The corresponding polynomial order, or GEO::NO_INDEX if the type code is unsupported.
     */
    /**
     * @brief Recovers the polynomial order of a quadrilateral element from its Gmsh element type code.
     *
     * @param element_type_code The Gmsh element type code for the high-order quadrilateral.
     * @return The corresponding polynomial order, or GEO::NO_INDEX if the type code is unsupported.
     */
    GEO::index_t get_order(const GEO::index_t element_type_code) {
        switch (element_type_code) {
            case 3: return 1;
            case 10: return 2;
            case 36: return 3;
            case 37: return 4;
            case 38: return 5;
            default:
                if (element_type_code < 47)
                    return GEO::NO_INDEX;
                return element_type_code-41;
        }
    }

    /**
     * @brief Generates the node permutation from the internal order to the Gmsh MSH order.
     *
     * @param order The polynomial order of the quadrilateral element.
     * @param msh_nodes_order The output node-order mapping, where each entry stores the corresponding Gmsh node index.
     */
    void generate_msh_nodes_order(
        const GEO::index_t order,
        std::vector<GEO::index_t>& msh_nodes_order
        ) {
        assert(order > 0);
        const auto n = order+1;

        msh_nodes_order.assign(n*n, GEO::NO_INDEX);

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
    }
}

namespace geolio
{
    /**
     * @brief Writes a high-order quadrilateral mesh in Gmsh 2.2 format.
     *
     * @tparam DIM The spatial dimension of the control grid.
     * @param control_grid The control grid to export.
     * @param out The output stream for the mesh file.
     * @return True if the mesh is written successfully; otherwise, false.
     */
    template<GEO::index_t DIM>
    static bool high_order_quad_mesh_save_2_2(
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
                if constexpr (DIM == 2) // The gmsh format does not support 2D nodes, so a z-coordinate of “0” is added.
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

            std::vector<GEO::index_t> msh_nodes_order;
            generate_msh_nodes_order(order, msh_nodes_order);
            assert(msh_nodes_order.size() == (order+1)*(order+1));

            std::vector<GEO::index_t> nodes_order(msh_nodes_order.size(), GEO::NO_INDEX);
            for (GEO::index_t i = 0, i_end = nodes_order.size(); i < i_end; ++i)
                nodes_order[msh_nodes_order[i]] = i;
            assert(std::ranges::none_of(nodes_order, [](const auto i){ return i == GEO::NO_INDEX; }));

            out << ELEMENTS_BEGIN << "\n";
            out << mesh.facets.nb() << "\n";
            for (const auto& f : mesh.facets) {
                out << f+1 << " "
                    << element_type_code << " "
                    << ELEMENTS_NUMBERS_OF_TAGS << " "
                    << ELEMENTS_PHYSICAL_GROUP_ID << " "
                    << ELEMENTS_ENTITY_ID << " ";
                for (const auto i : nodes_order)
                    out << control_grid.facet_nd(f, i)+1 << " ";
                out << "\n";
            }
            out << ELEMENTS_END << "\n";
        }

        return true;
    }

    template<GEO::index_t DIM>
    static bool high_order_quad_mesh_save_4_1(
        const QuadControlGrid<DIM>& control_grid,
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
                for (GEO::index_t d = 0; d < DIM; ++d) {
                    xyz_min[d] = std::min(xyz_min[d], p[d]);
                    xyz_max[d] = std::max(xyz_max[d], p[d]);
                }
            }
            out << ENTITIES_TAG << " ";
            if constexpr (DIM == 2)
                out << xyz_min[0] << " " << xyz_min[1] << " " << 0 << " " << xyz_max[0] << " " << xyz_max[1] << " " << 0 << " ";
            else if constexpr (DIM == 3)
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
                for (GEO::index_t d = 0; d < DIM; ++d)
                    out << p[d] << " ";
                if constexpr (DIM == 2) // The gmsh format does not support 2D nodes, so a z-coordinate of “0” is added.
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

            std::vector<GEO::index_t> msh_nodes_order;
            generate_msh_nodes_order(order, msh_nodes_order);
            assert(msh_nodes_order.size() == (order+1)*(order+1));

            std::vector<GEO::index_t> nodes_order(msh_nodes_order.size(), GEO::NO_INDEX);
            for (GEO::index_t i = 0, i_end = nodes_order.size(); i < i_end; ++i)
                nodes_order[msh_nodes_order[i]] = i;
            assert(std::ranges::none_of(nodes_order, [](const auto i){ return i == GEO::NO_INDEX; }));

            out << ELEMENTS_BEGIN << "\n";

            out << ELEMENTS_ENTITY_BLOCK << " "
                << mesh.facets.nb() << " "
                << "1" << " "
                << mesh.facets.nb() << "\n";

            out << ELEMENTS_ENTITY_DIMENSION << " "
                << ELEMENTS_ENTITY_ID << " "
                << element_type_code << " "
                << mesh.facets.nb() << "\n";

            for (const auto& f : mesh.facets) {
                out << f+1 << " ";
                for (const auto i : nodes_order)
                    out << control_grid.facet_nd(f, i)+1 << " ";
                out << "\n";
            }

            out << ELEMENTS_END << "\n";
        }

        return true;
    }

    template<GEO::index_t DIM>
    bool high_order_quad_mesh_save(
        const QuadControlGrid<DIM>& control_grid,
        const std::string& filepath,
        const std::string& version_number
        ) {
        std::ofstream out(filepath);
        if (!out.good()) {
            LOG::ERROR("Could not open file `{}` for writing!", filepath);
            return false;
        }

        if (version_number == "2.2")
            return high_order_quad_mesh_save_2_2(control_grid, out);
        if (version_number == "4.1")
            return high_order_quad_mesh_save_4_1(control_grid, out);
        LOG::ERROR("Unsupported version number `{}`", version_number);
        return false;
    }

    template bool high_order_quad_mesh_save<2>(const QuadControlGrid<2>& control_grid, const std::string& filepath, const std::string& version_number);
    template bool high_order_quad_mesh_save<3>(const QuadControlGrid<3>& control_grid, const std::string& filepath, const std::string& version_number);

    template<GEO::index_t DIM>
    static bool high_order_quad_mesh_load_2_2(
        LineInput& in,
        GEO::index_t& order,
        std::vector<double>& nodes,
        std::vector<GEO::index_t>& elements
        ) {
        while (!in.eof()) {
            if (!in.get_line())
                break;
            in.get_fields();
            if (in.nb_fields() != 1) {
                LOG::ERROR("Line {} :Expect keyword!", in.line_number());
                return false;
            }

            if (const std::string kw = in.field(0);
                kw == NODES_BEGIN
                ) {
                in.get_line();
                in.get_fields();
                if (in.nb_fields() != 1) {
                    LOG::ERROR("Line {} :Invalid nodes nb, expected 1 number!", in.line_number());
                    return false;
                }

                const GEO::index_t nodes_nb = in.field_as_uint(0);
                nodes.assign(3*nodes_nb, 0.0);
                for (GEO::index_t i = 0; i < nodes_nb; ++i) {
                    in.get_line();
                    in.get_fields();
                    if (in.nb_fields() != 4) {
                        LOG::ERROR("Line {} :Invalid node, expected 4 number!", in.line_number());
                        return false;
                    }

                    const GEO::index_t nd = in.field_as_uint(0)-1;
                    if (nd > nodes_nb-1) {
                        LOG::ERROR("Line {} :Invalid node idx `{}`!", in.line_number(), nd);
                        return false;
                    }

                    nodes[3*nd]   = in.field_as_double(1);
                    nodes[3*nd+1] = in.field_as_double(2);
                    nodes[3*nd+2] = in.field_as_double(3);
                }

                in.get_line();
                in.get_fields();
                if (in.nb_fields() != 1) {
                    LOG::ERROR("Line {} :Expect end keyword!", in.line_number());
                    return false;
                }
                if (const std::string end_kw = in.field(0);
                    end_kw != NODES_END) {
                    LOG::ERROR("Line {} :Invalid end mesh format `{}`!", in.line_number(), end_kw);
                    return false;
                }
            }
            else if (kw == ELEMENTS_BEGIN) {
                in.get_line();
                in.get_fields();
                if (in.nb_fields() != 1) {
                    LOG::ERROR("Line {} :Invalid elements nb, expected 1 number!", in.line_number());
                    return false;
                }

                const GEO::index_t elements_nb = in.field_as_uint(0);
                GEO::index_t element_type_code = GEO::NO_INDEX;
                GEO::index_t element_nodes_nb = GEO::NO_INDEX;
                for (GEO::index_t i = 0; i < elements_nb; ++i) {
                    in.get_line();
                    in.get_fields();
                    if (in.nb_fields() <= 5) {
                        LOG::ERROR("Line {} :Invalid element, expected more than 5 uint!", in.line_number());
                        return false;
                    }

                    const GEO::index_t ele = in.field_as_uint(0)-1;
                    if (ele > elements_nb-1) {
                        LOG::ERROR("Line {} :Invalid element idx `{}`!", in.line_number(), ele);
                        return false;
                    }

                    if (element_type_code == GEO::NO_INDEX) {
                        element_type_code = in.field_as_uint(1);
                        order = get_order(element_type_code);
                        if (order == 0 || order == GEO::NO_INDEX) {
                            LOG::ERROR("Line {} :Invalid element type code `{}`!", in.line_number(), element_type_code);
                            return false;
                        }

                        element_nodes_nb = (order+1)*(order+1);
                        elements.assign(elements_nb*element_nodes_nb, GEO::NO_INDEX);
                    }
                    else if (element_type_code != in.field_as_uint(1)) {
                        LOG::ERROR("Line {} :Only supports elements that are all of the same type `{}!={}`!`.", in.line_number(), in.field_as_uint(1), element_type_code);
                        return false;
                    }

                    if (in.nb_fields() != 5+element_nodes_nb) {
                        LOG::ERROR("Line {} :Invalid element nodes nb, expected `{}` uint!", in.line_number(), 5+element_nodes_nb);
                        return false;
                    }
                    assert(element_nodes_nb != GEO::NO_INDEX);
                    for (GEO::index_t j = 0; j < element_nodes_nb; ++j)
                        elements[ele*element_nodes_nb+j] = in.field_as_uint(5+j)-1;
                }

                in.get_line();
                in.get_fields();
                if (in.nb_fields() != 1) {
                    LOG::ERROR("Line {} :Expect end keyword!", in.line_number());
                    return false;
                }
                if (const std::string end_kw = in.field(0);
                    end_kw != ELEMENTS_END) {
                    LOG::ERROR("Line {} :Invalid end mesh format `{}`!", in.line_number(), end_kw);
                    return false;
                }
            }
            else {
                LOG::ERROR("Line {} :Invalid kw `{}`!", in.line_number(), kw);
                return false;
            }
        }

        return true;
    }

    template<GEO::index_t DIM>
    static bool high_order_quad_mesh_load_4_1(
        LineInput& in,
        GEO::index_t& order,
        std::vector<double>& nodes,
        std::vector<GEO::index_t>& elements
        ) {
        while (!in.eof()) {
            if (!in.get_line())
                break;
            in.get_fields();
            if (in.nb_fields() != 1) {
                LOG::ERROR("Line {} :Expect keyword!", in.line_number());
                return false;
            }

            if (const std::string kw = in.field(0);
                kw == NODES_BEGIN
                ) {
                in.get_line();
                in.get_fields();
                if (in.nb_fields() != 4) {
                    LOG::ERROR("Line {} :Invalid line, expected 4!", in.line_number());
                    return false;
                }

                const GEO::index_t nodes_nb = in.field_as_uint(1);
                nodes.assign(3*nodes_nb, 0.0);

                in.get_line(); // skip a line

                std::vector<GEO::index_t> nodes_indices(nodes_nb, GEO::NO_INDEX);
                for (GEO::index_t i = 0; i < nodes_nb; ++i) {
                    in.get_line();
                    in.get_fields();
                    if (in.nb_fields() != 1) {
                        LOG::ERROR("Line {} :Invalid node idx, expected 1!", in.line_number());
                        return false;
                    }
                    const auto nd = in.field_as_uint(0)-1;
                    if (nd > nodes_nb-1) {
                        LOG::ERROR("Line {} :Invalid node idx `{}`!", in.line_number(), nd);
                        return false;
                    }

                    nodes_indices[i] = nd;
                }
                assert(std::ranges::none_of(nodes_indices, [](const auto idx){ return idx == GEO::NO_INDEX; }));

                for (GEO::index_t i = 0; i < nodes_nb; ++i) {
                    in.get_line();
                    in.get_fields();
                    if (in.nb_fields() != 3) {
                        LOG::ERROR("Line {} :Invalid node, expected 3 number!", in.line_number());
                        return false;
                    }

                    const GEO::index_t nd = nodes_indices[i];
                    nodes[3*nd]   = in.field_as_double(0);
                    nodes[3*nd+1] = in.field_as_double(1);
                    nodes[3*nd+2] = in.field_as_double(2);
                }

                in.get_line();
                in.get_fields();
                if (in.nb_fields() != 1) {
                    LOG::ERROR("Line {} :Expect end keyword!", in.line_number());
                    return false;
                }
                if (const std::string end_kw = in.field(0);
                    end_kw != NODES_END) {
                    LOG::ERROR("Line {} :Invalid end mesh format `{}`!", in.line_number(), end_kw);
                    return false;
                }
            }
            else if (kw == ELEMENTS_BEGIN) {
                in.get_line();
                in.get_fields();
                if (in.nb_fields() != 4) {
                    LOG::ERROR("Line {} :Invalid line, expected 4 number!", in.line_number());
                    return false;
                }
                const GEO::index_t elements_nb = in.field_as_uint(1);

                in.get_line();
                in.get_fields();
                if (in.nb_fields() != 4) {
                    LOG::ERROR("Line {} :Invalid line, expected 4 number!", in.line_number());
                    return false;
                }
                const GEO::index_t element_type_code = in.field_as_uint(2);
                order = get_order(element_type_code);
                if (order == 0 || order == GEO::NO_INDEX) {
                    LOG::ERROR("Line {} :Invalid element type code `{}`!", in.line_number(), element_type_code);
                    return false;
                }
                const GEO::index_t element_nodes_nb = (order+1)*(order+1);

                elements.assign(elements_nb*element_nodes_nb, GEO::NO_INDEX);

                for (GEO::index_t ele = 0; ele < elements_nb; ++ele) {
                    in.get_line();
                    in.get_fields();
                    if (in.nb_fields() <= 2) {
                        LOG::ERROR("Line {} :Invalid element, expected more than 2 uint!", in.line_number());
                        return false;
                    }

                    if (in.nb_fields() != 1+element_nodes_nb) {
                        LOG::ERROR("Line {} :Invalid element nodes nb, expected `{}` uint!", in.line_number(), 5+element_nodes_nb);
                        return false;
                    }
                    assert(element_nodes_nb != GEO::NO_INDEX);
                    for (GEO::index_t j = 0; j < element_nodes_nb; ++j)
                        elements[ele*element_nodes_nb+j] = in.field_as_uint(1+j)-1;
                }

                in.get_line();
                in.get_fields();
                if (in.nb_fields() != 1) {
                    LOG::ERROR("Line {} :Expect end keyword!", in.line_number());
                    return false;
                }
                if (const std::string end_kw = in.field(0);
                    end_kw != ELEMENTS_END) {
                    LOG::ERROR("Line {} :Invalid end mesh format `{}`!", in.line_number(), end_kw);
                    return false;
                }
            }
            else if (kw == ENTITIES_BEGIN) {
                in.get_line();
                in.get_line();

                in.get_line();
                in.get_fields();
                if (in.nb_fields() != 1) {
                    LOG::ERROR("Line {} :Expect end keyword!", in.line_number());
                    return false;
                }
                if (const std::string end_kw = in.field(0);
                    end_kw != ENTITIES_END) {
                    LOG::ERROR("Line {} :Invalid end mesh format `{}`!", in.line_number(), end_kw);
                    return false;
                }
            }
            else {
                LOG::ERROR("Line {} :Invalid kw `{}`!", in.line_number(), kw);
                return false;
            }
        }

        return true;
    }

    template<GEO::index_t DIM>
    bool high_order_quad_mesh_load(
        const std::string& filepath,
        GEO::Mesh& mesh,
        std::unique_ptr<QuadControlGrid<DIM>>& control_grid_ptr
        ) {
        mesh.clear();
        mesh.vertices.set_dimension(DIM);

        LineInput in(filepath);
        if (!in.OK()) {
            LOG::ERROR("Could not open file `{}` for reading!", filepath);
            return false;
        }

        while (!in.eof()) {
            if (!in.get_line())
                break;
            in.get_fields();
            if (in.nb_fields() != 1) {
                LOG::ERROR("Line {} :Expect keyword!", in.line_number());
                return false;
            }

            if (const std::string kw = in.field(0);
                kw == MESH_FORMAT_BEGIN
                ) {
                in.get_line();
                in.get_fields();
                if (in.nb_fields() != 3) {
                    LOG::ERROR("Line {} :Invalid mesh format, expected 3 numbers!", in.line_number());
                    return false;
                }

                const std::string version_number = in.field(0);

                in.get_line();
                in.get_fields();
                if (in.nb_fields() != 1) {
                    LOG::ERROR("Line {} :Expect end keyword!", in.line_number());
                    return false;
                }
                if (const std::string end_kw = in.field(0);
                    end_kw != MESH_FORMAT_END) {
                    LOG::ERROR("Line {} :Invalid end mesh format `{}`!", in.line_number(), end_kw);
                    return false;
                }

                /* Load nodes and elements */
                GEO::index_t order = GEO::NO_INDEX;
                std::vector<double> nodes;
                std::vector<GEO::index_t> elements;
                if (version_number == "2.2") {
                    if (!high_order_quad_mesh_load_2_2<DIM>(in, order, nodes, elements)) {
                        LOG::ERROR("Load msh 2.2 failed!");
                        return false;
                    }
                }
                else if (version_number == "4.1") {
                    if (!high_order_quad_mesh_load_4_1<DIM>(in, order, nodes, elements)) {
                        LOG::ERROR("Load msh 4.1 failed!");
                        return false;
                    }
                }
                else {
                    LOG::ERROR("Line {} :Unsupported version number `{}`!", in.line_number(), version_number);
                    return false;
                }

                const GEO::index_t element_nodes_nb = (order+1)*(order+1);

                /* Build mesh */
                {
                    const GEO::index_t nodes_nb = nodes.size()/3;
                    const GEO::index_t elements_nb = elements.size()/element_nodes_nb;

                    std::vector<GEO::index_t> node_to_vertex(nodes.size()/3, GEO::NO_VERTEX);
                    std::vector<GEO::index_t> quad_vertices;
                    quad_vertices.reserve(4*elements_nb);

                    GEO::index_t nb_vertices = 0;
                    for (GEO::index_t ele = 0; ele < elements_nb; ++ele) {
                        for (GEO::index_t lv = 0; lv < 4; ++lv) {
                            const auto& nd = elements[ele*element_nodes_nb+lv];
                            if (nd > nodes_nb-1) {
                                LOG::ERROR("Invalid node idx `{}`!", nd);
                                return false;
                            }
                            if (node_to_vertex[nd] == GEO::NO_VERTEX)
                                node_to_vertex[nd] = nb_vertices++;

                            quad_vertices.push_back(node_to_vertex[nd]);
                        }
                    }

                    /* Create vertices */
                    mesh.vertices.create_vertices(nb_vertices);
                    for (GEO::index_t nd = 0; nd < nodes_nb; ++nd) {
                        if (const auto& v = node_to_vertex[nd];
                            v != GEO::NO_VERTEX
                            ) {
                            auto& p = mesh.vertices.point<DIM>(v);
                            for (GEO::index_t d = 0; d < DIM; ++d)
                                p[d] = nodes[nd*3+d];
                        }
                    }

                    /* Create quads */
                    mesh.facets.create_quads(quad_vertices.size()/4);
                    for (const auto& f : mesh.facets) {
                        for (GEO::index_t lv = 0; lv < 4; ++lv)
                            mesh.facets.set_vertex(f, lv, quad_vertices[4*f+lv]);
                    }
                    mesh.facets.connect();
                }

                /* Build control grid */
                {
                    std::vector<GEO::index_t> gmsh_nodes_order;
                    generate_msh_nodes_order(order, gmsh_nodes_order);

                    control_grid_ptr = std::make_unique<QuadControlGrid<DIM>>(mesh, order);
                    for (const auto& f : mesh.facets) {
                        for (GEO::index_t i = 0; i < element_nodes_nb; ++i) {
                            const auto nd = control_grid_ptr->facet_nd(f, i);
                            const auto gmsh_nd = elements[f*element_nodes_nb+gmsh_nodes_order[i]];
                            auto& p = control_grid_ptr->control_node(nd);
                            for (GEO::index_t d = 0; d < DIM; ++d)
                                p[d] = nodes[3*gmsh_nd+d];
                        }
                    }
                }
            }
            else {
                LOG::ERROR("Line {} :Invalid kw `{}`!", in.line_number(), kw);
                return false;
            }
        }

        return true;
    }

    template bool high_order_quad_mesh_load<2>(const std::string& filepath, GEO::Mesh& mesh, std::unique_ptr<QuadControlGrid<2>>& control_grid_ptr);
    template bool high_order_quad_mesh_load<3>(const std::string& filepath, GEO::Mesh& mesh, std::unique_ptr<QuadControlGrid<3>>& control_grid_ptr);
}