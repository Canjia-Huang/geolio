//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/3/23.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//

#include <geogram/basic/command_line_args.h>
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_io.h>
#include <gtest/gtest.h>
#include "utils.h"
#include "common/log.h"
#include "mesh_utils/tet_operators.h"

namespace
{
    void build_tet_mesh(
        GEO::Mesh& M
        ) {
        M.clear();

        GEO::index_t new_v = M.vertices.create_vertices(4*4*4);
        for (GEO::index_t i = 0; i < 4; ++i) {
            for (GEO::index_t j = 0; j < 4; ++j) {
                for (GEO::index_t k = 0; k < 4; ++k)
                    M.vertices.point(new_v++) = GEO::vec3(k, j, i);
            }
        }

        GEO::index_t new_c = M.cells.create_tets(3*3*3*6);
        for (GEO::index_t i = 0; i < 3; ++i) {
            for (GEO::index_t j = 0; j < 3; ++j) {
                for (GEO::index_t k = 0; k < 3; ++k) {
                    const GEO::index_t v0 = 16*i+4*j+k;
                    const GEO::index_t v1 = v0+1;
                    const GEO::index_t v2 = v0+4;
                    const GEO::index_t v3 = v1+4;
                    const GEO::index_t v4 = v0+16;
                    const GEO::index_t v5 = v4+1;
                    const GEO::index_t v6 = v4+4;
                    const GEO::index_t v7 = v5+4;
                    M.cells.set_vertex(new_c+0, 0, v4); M.cells.set_vertex(new_c+0, 1, v6); M.cells.set_vertex(new_c+0, 2, v5); M.cells.set_vertex(new_c+0, 3, v1);
                    M.cells.set_vertex(new_c+1, 0, v0); M.cells.set_vertex(new_c+1, 1, v4); M.cells.set_vertex(new_c+1, 2, v6); M.cells.set_vertex(new_c+1, 3, v1);
                    M.cells.set_vertex(new_c+2, 0, v0); M.cells.set_vertex(new_c+2, 1, v1); M.cells.set_vertex(new_c+2, 2, v2); M.cells.set_vertex(new_c+2, 3, v6);
                    M.cells.set_vertex(new_c+3, 0, v1); M.cells.set_vertex(new_c+3, 1, v3); M.cells.set_vertex(new_c+3, 2, v2); M.cells.set_vertex(new_c+3, 3, v6);
                    M.cells.set_vertex(new_c+4, 0, v1); M.cells.set_vertex(new_c+4, 1, v7); M.cells.set_vertex(new_c+4, 2, v3); M.cells.set_vertex(new_c+4, 3, v6);
                    M.cells.set_vertex(new_c+5, 0, v1); M.cells.set_vertex(new_c+5, 1, v6); M.cells.set_vertex(new_c+5, 2, v5); M.cells.set_vertex(new_c+5, 3, v7);
                    new_c += 6;
                }
            }
        }

        M.cells.connect();
    }

    void get_mesh_c_lf(
        const GEO::Mesh& M,
        std::vector<std::pair<GEO::index_t, GEO::index_t>>& interior_facet_c_lf,
        std::vector<std::pair<GEO::index_t, GEO::index_t>>& border_facet_c_lf
        ) {
        for (const auto& c : M.cells) {
            for (GEO::index_t lf = 0; lf < 4; ++lf) {
                if (M.cells.adjacent(c, lf) == GEO::NO_CELL)
                    border_facet_c_lf.emplace_back(c, lf);
                else
                    interior_facet_c_lf.emplace_back(c, lf);
            }
        }
    }
}

namespace GEO::MeshUtils::Test
{
    using namespace GEO::MeshUtils;

    class TetrahedronOperatorsTest : public ::testing::TestWithParam<std::pair<GEO::index_t, GEO::index_t>> {
        void SetUp() override {
            build_tet_mesh(M);
            M_c_processed.bind(M.cells.attributes(), "processed");
        }

    public:
        void check_connections() {
            std::vector<GEO::index_t> current_connections(4*M.cells.nb(), GEO::NO_CELL);
            for (const auto& c : M.cells) {
                for (GEO::index_t lf = 0; lf < 4; ++lf)
                    current_connections[4*c+lf] = M.cells.adjacent(c, lf);
            }

            M.cells.connect();
            for (const auto& c : M.cells) {
                for (GEO::index_t lf = 0; lf < 4; ++lf)
                    EXPECT_EQ(current_connections[4*c+lf], M.cells.adjacent(c, lf));
            }

            /* Rollback adjacency */
            for (const auto& c : M.cells) {
                for (GEO::index_t lf = 0; lf < 4; ++lf)
                    M.cells.set_adjacent(c, lf, current_connections[4*c+lf]);
            }
        }

        void save_results(
            const GEO::index_t c,
            const GEO::index_t lf
            ) const {
            EXPECT_TRUE(GEO::mesh_save(M, get_current_test_name() +
                                            "_c" + std::to_string(c) +
                                            "_lf" + std::to_string(lf) +
                                            ".geogram"));
        }

        GEO::Mesh M;
        GEO::Attribute<GEO::index_t> M_c_processed;
    };

    /* == For INSTANTIATE_TEST_SUITE =============================================================================== */

    enum TETRAHEDRAL_MESH_TEST_TYPE {
        INTERIOR_FACET_C_LF,
        BORDER_FACET_C_LF
    };

    const auto TETRAHEDRON_MESH_GET_TEST_PARAMS = [](
        const TETRAHEDRAL_MESH_TEST_TYPE type
        ) {
        GEO::initialize(GEO::GEOGRAM_INSTALL_ALL);

        GEO::Mesh M;
        build_tet_mesh(M);

        std::vector<std::pair<GEO::index_t, GEO::index_t>> interior_facet_c_lf;
        std::vector<std::pair<GEO::index_t, GEO::index_t>> border_facet_c_lf;
        get_mesh_c_lf(
            M,
            interior_facet_c_lf,
            border_facet_c_lf);

        if (type == INTERIOR_FACET_C_LF)
            return interior_facet_c_lf;
        if (type == BORDER_FACET_C_LF)
            return border_facet_c_lf;
        assert(0);
    };

    /* == EdgeSwap23Test =========================================================================================== */

    class EdgeSwap23Test : public TetrahedronOperatorsTest {
    public:
        virtual void compute(
            GEO::index_t c,
            GEO::index_t lf) = 0;
    };

    class InteriorEdgeSwap23Test : public EdgeSwap23Test {
    public:
        void compute(
            const GEO::index_t c,
            const GEO::index_t lf
            ) override {
            ASSERT_NE(M.cells.adjacent(c, lf), GEO::NO_CELL);
            const GEO::index_t new_c = M.cells.create_tets(1);

            M_c_processed[c] = 1;
            M_c_processed[M.cells.adjacent(c, lf)] = 1;
            M_c_processed[new_c] = 1;

            edge_swap_2_3(
                M,
                c, lf,
                new_c);
        }
    };

    class BorderEdgeSwap23Test : public EdgeSwap23Test {
    public:
        void compute(
            const GEO::index_t c,
            const GEO::index_t lf
            ) override {
            ASSERT_EQ(M.cells.adjacent(c, lf), GEO::NO_CELL);

            M_c_processed[c] = 1;

            edge_swap_2_3(
                M,
                c, lf,
                GEO::NO_CELL);
        }
    };

    TEST_P(InteriorEdgeSwap23Test, each_facet) {
        auto [c, lf] = GetParam();

        compute(c, lf);
        check_connections();
        save_results(c, lf);
    }

    TEST_P(BorderEdgeSwap23Test, each_facet) {
        auto [c, lf] = GetParam();

        compute(c, lf);
        check_connections();
        save_results(c, lf);
    }

    INSTANTIATE_TEST_SUITE_P(
        TetrahedronOperatorsTest,
        InteriorEdgeSwap23Test,
        ::testing::ValuesIn(TETRAHEDRON_MESH_GET_TEST_PARAMS(INTERIOR_FACET_C_LF))
    );

    INSTANTIATE_TEST_SUITE_P(
        TetrahedronOperatorsTest,
        BorderEdgeSwap23Test,
        ::testing::ValuesIn(TETRAHEDRON_MESH_GET_TEST_PARAMS(BORDER_FACET_C_LF))
    );

    /* == CellSplit ================================================================================================ */

    class CellSplitTest : public TetrahedronOperatorsTest {
    public:
        void compute(
            const GEO::index_t c
            ) {
            const GEO::index_t new_v = M.vertices.create_vertices(1);
            const GEO::index_t new_c = M.cells.create_tets(3);

            M_c_processed[c] = 1;
            M_c_processed[new_c] = 1;
            M_c_processed[new_c+1] = 1;
            M_c_processed[new_c+2] = 1;

            cell_split(
                M,
                c,
                new_v,
                new_c, new_c+1, new_c+2);
        }
    };

    class InteriorCellSplitTest : public CellSplitTest {};

    class BorderCellSplitTest : public CellSplitTest {};

    TEST_P(InteriorCellSplitTest, each_cell) {
        auto [c, _] = GetParam();

        compute(c);
        check_connections();
        save_results(c, 0);
    }

    TEST_P(BorderCellSplitTest, each_cell) {
        auto [c, _] = GetParam();

        compute(c);
        check_connections();
        save_results(c, 0);
    }

    INSTANTIATE_TEST_SUITE_P(
        TetrahedronOperatorsTest,
        InteriorCellSplitTest,
        ::testing::ValuesIn(TETRAHEDRON_MESH_GET_TEST_PARAMS(INTERIOR_FACET_C_LF))
    );

    INSTANTIATE_TEST_SUITE_P(
        TetrahedronOperatorsTest,
        BorderCellSplitTest,
        ::testing::ValuesIn(TETRAHEDRON_MESH_GET_TEST_PARAMS(BORDER_FACET_C_LF))
    );
}