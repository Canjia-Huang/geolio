//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/3/23.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//

#include <ranges>
#include <unordered_set>
#include <geogram/basic/command_line_args.h>
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_io.h>
#include <gtest/gtest.h>
#include "utils.h"
#include "common/log.h"
#include "mesh_utils/pair_hash.h"
#include "mesh_utils/tet_operations.h"

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
                    M.vertices.point(new_v++) = GEO::vec3(i, j, k);
            }
        }

        GEO::index_t new_c = M.cells.create_tets(3*3*3*6);
        for (GEO::index_t i = 0; i < 3; ++i) {
            for (GEO::index_t j = 0; j < 3; ++j) {
                for (GEO::index_t k = 0; k < 3; ++k) {
                    const GEO::index_t v0 = 16*i+4*j+k;
                    const GEO::index_t v1 = v0+16;
                    const GEO::index_t v2 = v0+4;
                    const GEO::index_t v3 = v2+16;
                    const GEO::index_t v4 = v0+1;
                    const GEO::index_t v5 = v4+16;
                    const GEO::index_t v6 = v4+4;
                    const GEO::index_t v7 = v6+16;
                    M.cells.set_vertex(new_c+0, 0, v4); M.cells.set_vertex(new_c+0, 1, v6); M.cells.set_vertex(new_c+0, 2, v5); M.cells.set_vertex(new_c+0, 3, v1);
                    M.cells.set_vertex(new_c+1, 0, v0); M.cells.set_vertex(new_c+1, 1, v6); M.cells.set_vertex(new_c+1, 2, v4); M.cells.set_vertex(new_c+1, 3, v1);
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
        std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>>& interior_vertex_c_lv,
        std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>>& border_vertex_c_lv,
        std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>>& interior_edge_c_lf_lv,
        std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>>& border_edge_c_lf_lv,
        std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>>& interior_edge_c_le,
        std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>>& border_edge_c_le,
        std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>>& interior_facet_c_lf,
        std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>>& border_facet_c_lf
        ) {
        /* Find all border vertices and edges */
        std::unordered_set<GEO::index_t> border_vertices;
        std::unordered_set<std::pair<GEO::index_t, GEO::index_t>, GEO::MeshUtils::PairHash> border_edges;
        for (const auto& c : M.cells) {
            for (GEO::index_t lf = 0; lf < 4; ++lf) {
                if (M.cells.adjacent(c, lf) != GEO::NO_CELL)
                    continue;
                for (GEO::index_t lv = 0; lv < 3; ++lv) {
                    border_vertices.insert(M.cells.facet_vertex(c, lf, lv));

                    const std::pair<GEO::index_t, GEO::index_t> edge = std::minmax(
                        M.cells.facet_vertex(c, lf, lv), M.cells.facet_vertex(c, lf, (lv+1)%3));
                    border_edges.insert(edge);
                }
            }
        }

        /* Get interior/border vertex (c, lv) */
        for (const auto& c : M.cells) {
            for (GEO::index_t lv = 0; lv < 4; ++lv) {
                if (border_vertices.contains(M.cells.vertex(c, lv)))
                    border_vertex_c_lv.emplace_back(c, lv, GEO::NO_INDEX);
                else
                    interior_vertex_c_lv.emplace_back(c, lv, GEO::NO_INDEX);
            }
        }

        /* Get interior/border edge (c, lf, lv) */
        for (const auto& c : M.cells) {
            for (GEO::index_t lf = 0; lf < 4; ++lf) {
                for (GEO::index_t lv = 0; lv < 3; ++lv) {
                    const std::pair<GEO::index_t, GEO::index_t> edge = std::minmax(
                            M.cells.facet_vertex(c, lf, lv), M.cells.facet_vertex(c, lf, (lv+1)%3));
                    if (border_edges.contains(edge))
                        border_edge_c_lf_lv.emplace_back(c, lf, lv);
                    else
                        interior_edge_c_lf_lv.emplace_back(c, lf, lv);
                }
            }
        }

        /* Get interior/border edge (c, le) */
        for (const auto& c : M.cells) {
            for (GEO::index_t le = 0; le < 6; ++le) {
                const std::pair<GEO::index_t, GEO::index_t> edge = std::minmax(
                    M.cells.edge_vertex(c, le, 0), M.cells.edge_vertex(c, le, 1));
                if (border_edges.contains(edge))
                    border_edge_c_le.emplace_back(c, le, GEO::NO_INDEX);
                else
                    interior_edge_c_le.emplace_back(c, le, GEO::NO_INDEX);
            }
        }

        /* Get interior/border facet (c, lf) */
        for (const auto& c : M.cells) {
            for (GEO::index_t lf = 0; lf < 4; ++lf) {
                if (M.cells.adjacent(c, lf) == GEO::NO_CELL)
                    border_facet_c_lf.emplace_back(c, lf, GEO::NO_INDEX);
                else
                    interior_facet_c_lf.emplace_back(c, lf, GEO::NO_INDEX);
            }
        }
    }
}

namespace GEO::MeshUtils::Test
{
    using namespace GEO::MeshUtils::Tet;

    class TetrahedronOperationsTest : public ::testing::TestWithParam<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>> {
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

        void save_results_c(
            const GEO::index_t c
            ) const {
            EXPECT_TRUE(GEO::mesh_save(M, get_current_test_name() +
                                            "_c" + std::to_string(c) +
                                            ".geogram"));
        }

        void save_results_c_lf_lv(
            const GEO::index_t c,
            const GEO::index_t lf,
            const GEO::index_t lv
            ) const {
            EXPECT_TRUE(GEO::mesh_save(M, get_current_test_name() +
                                            "_c" + std::to_string(c) +
                                            "_lf" + std::to_string(lf) +
                                            "_lv" + std::to_string(lv) +
                                            ".geogram"));
        }

        void save_results_c_le(
            const GEO::index_t c,
            const GEO::index_t le
            ) const {
            EXPECT_TRUE(GEO::mesh_save(M, get_current_test_name() +
                                            "_c" + std::to_string(c) +
                                            "_le" + std::to_string(le) +
                                            ".geogram"));
        }

        void save_results_c_lf(
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
        INTERIOR_VERTEX_C_LV,
        BORDER_VERTEX_C_LV,
        INTERIOR_EDGE_C_LF_LV,
        BORDER_EDGE_C_LF_LV,
        INTERIOR_EDGE_C_LE,
        BORDER_EDGE_C_LE,
        INTERIOR_FACET_C_LF,
        BORDER_FACET_C_LF
    };

    const auto TETRAHEDRON_MESH_GET_TEST_PARAMS = [](
        const TETRAHEDRAL_MESH_TEST_TYPE type
        ) {
        GEO::initialize(GEO::GEOGRAM_INSTALL_ALL);

        GEO::Mesh M;
        build_tet_mesh(M);

        std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>> interior_vertex_c_lv;
        std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>> border_vertex_c_lv;
        std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>> interior_edge_c_lf_lv;
        std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>> border_edge_c_lf_lv;
        std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>> interior_edge_c_le;
        std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>> border_edge_c_le;
        std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>> interior_facet_c_lf;
        std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>> border_facet_c_lf;
        get_mesh_c_lf(
            M,
            interior_vertex_c_lv,
            border_vertex_c_lv,
            interior_edge_c_lf_lv,
            border_edge_c_lf_lv,
            interior_edge_c_le,
            border_edge_c_le,
            interior_facet_c_lf,
            border_facet_c_lf);

        switch (type) {
            case INTERIOR_VERTEX_C_LV: return interior_vertex_c_lv;
            case BORDER_VERTEX_C_LV: return border_vertex_c_lv;
            case INTERIOR_EDGE_C_LE: return interior_edge_c_le;
            case BORDER_EDGE_C_LE: return border_edge_c_le;
            case INTERIOR_EDGE_C_LF_LV: return interior_edge_c_lf_lv;
            case BORDER_EDGE_C_LF_LV: return border_edge_c_lf_lv;
            case INTERIOR_FACET_C_LF: return interior_facet_c_lf;
            case BORDER_FACET_C_LF: return border_facet_c_lf;
        }
        assert(0);
    };

    /* == GetVertexIncidentTetrahedraTest ========================================================================== */

    class GetVertexIncidentTetrahedraTest : public TetrahedronOperationsTest {
    public:
        bool compute(
            const GEO::index_t _c,
            const GEO::index_t _lv
            ) {
            return get_vertex_incident_tetrahedra(M, _c, _lv, c_and_lv);
        }

        void check_incident(
            const GEO::index_t v
            ) {
            for (const auto& [c, lv] : c_and_lv)
                EXPECT_EQ(M.cells.vertex(c, lv), v);
        }

        void check_complete(
            const GEO::index_t v
            ) const {
            std::unordered_map<std::pair<GEO::index_t, GEO::index_t>, bool, PairHash> incident_cells; // (c, lv) -> found
            for (const auto& c : M.cells) {
                for (GEO::index_t lv = 0; lv < 4; ++lv) {
                    if (M.cells.vertex(c, lv) == v) {
                        incident_cells.emplace(std::pair(c, lv), false);
                        break;
                    }
                }
            }

            for (const auto& c_lv : c_and_lv) {
                auto it = incident_cells.find(c_lv);
                ASSERT_FALSE(it == incident_cells.end());
                EXPECT_FALSE(it->second);
                it->second = true;
            }

            for (const auto &found: incident_cells | std::views::values)
                EXPECT_TRUE(found);
        }

        std::vector<std::pair<GEO::index_t, GEO::index_t>> c_and_lv;
    };

    class GetInteriorVertexIncidentTetrahedraTest : public GetVertexIncidentTetrahedraTest {};

    TEST_P(GetInteriorVertexIncidentTetrahedraTest, each_vertex) {
        auto [c, lv, _] = GetParam();

        EXPECT_FALSE(compute(c, lv));
        check_incident(M.cells.vertex(c, lv));
        check_complete(M.cells.vertex(c, lv));
    }

    INSTANTIATE_TEST_SUITE_P(
        TetrahedronOperationsTest,
        GetInteriorVertexIncidentTetrahedraTest,
        ::testing::ValuesIn(TETRAHEDRON_MESH_GET_TEST_PARAMS(INTERIOR_VERTEX_C_LV)));

    class GetBorderVertexIncidentTetrahedraTest : public GetVertexIncidentTetrahedraTest {};

    TEST_P(GetBorderVertexIncidentTetrahedraTest, each_vertex) {
        auto [c, lv, _] = GetParam();

        EXPECT_TRUE(compute(c, lv));
        check_incident(M.cells.vertex(c, lv));
        check_complete(M.cells.vertex(c, lv));
    }

    INSTANTIATE_TEST_SUITE_P(
        TetrahedronOperationsTest,
        GetBorderVertexIncidentTetrahedraTest,
        ::testing::ValuesIn(TETRAHEDRON_MESH_GET_TEST_PARAMS(BORDER_VERTEX_C_LV)));

    /* == GetEdgeIncidentTetrahedraTest ============================================================================ */

    class GetEdgeIncidentTetrahedraTest : public TetrahedronOperationsTest {
    public:
        bool compute(
            const GEO::index_t _c,
            const GEO::index_t _lf,
            const GEO::index_t _lv
            ) {
            return get_edge_incident_tetrahedra(M, _c, _lf, _lv, ordered_c_and_lf);
        }

        bool compute(
            const GEO::index_t _c,
            const GEO::index_t _le
            ) {
            return get_edge_incident_tetrahedra(M, _c, _le, ordered_c_and_lf);
        }

        void check_incident(
            const GEO::index_t ev0,
            const GEO::index_t ev1
            ) {
            for (const auto& [c, lf] : ordered_c_and_lf) {
                bool incident = false;
                for (GEO::index_t lv = 0; lv < 3; ++lv) {
                    const GEO::index_t cev0 = M.cells.facet_vertex(c, lf, lv);
                    const GEO::index_t cev1 = M.cells.facet_vertex(c, lf, (lv+1)%3);
                    if ((cev0 == ev0 && cev1 == ev1) ||
                        (cev0 == ev1 && cev1 == ev0)
                        ) {
                        incident = true;
                        break;
                    }
                }
                EXPECT_TRUE(incident);
            }
        }

        void check_complete(
            const GEO::index_t ev0,
            const GEO::index_t ev1
            ) {
            std::unordered_map<GEO::index_t, bool> incident_cells; // (cell, found)
            for (const auto& c : M.cells) {
                for (GEO::index_t le = 0; le < 6; ++le) {
                    const GEO::index_t cev0 = M.cells.edge_vertex(c, le, 0);
                    const GEO::index_t cev1 = M.cells.edge_vertex(c, le, 1);
                    if ((cev0 == ev0 && cev1 == ev1) || (cev0 == ev1 && cev1 == ev0)) {
                        incident_cells.emplace(c, false);
                        break;
                    }
                }
            }

            for (const auto &c: ordered_c_and_lf | std::views::keys) {
                auto it = incident_cells.find(c);
                EXPECT_FALSE(it == incident_cells.end());
                it->second = true;
            }

            for (const auto &found: incident_cells | std::views::values)
                EXPECT_TRUE(found);
        }

        virtual void check_loop() = 0;

        std::vector<std::pair<GEO::index_t, GEO::index_t>> ordered_c_and_lf;
    };


    class GetInteriorEdgeIncidentTetrahedraTest : public GetEdgeIncidentTetrahedraTest {
    public:
        void check_loop() override {
            for (GEO::index_t i = 0, i_end = ordered_c_and_lf.size(); i < i_end; ++i) {
                const auto& [c, lf] = ordered_c_and_lf[i];
                EXPECT_EQ(M.cells.adjacent(c, lf), ordered_c_and_lf[(i+1)%i_end].first);
            }
        }
    };

    class GetInteriorEdgeIncidentTetrahedraTest_c_lf_lv : public GetInteriorEdgeIncidentTetrahedraTest {};

    TEST_P(GetInteriorEdgeIncidentTetrahedraTest_c_lf_lv, each_edge) {
        auto [c, lf, lv] = GetParam();

        EXPECT_FALSE(compute(c, lf, lv));
        check_incident(M.cells.facet_vertex(c, lf, lv), M.cells.facet_vertex(c, lf, (lv+1)%3));
        check_complete(M.cells.facet_vertex(c, lf, lv), M.cells.facet_vertex(c, lf, (lv+1)%3));
        check_loop();
    }

    INSTANTIATE_TEST_SUITE_P(
        TetrahedronOperationsTest,
        GetInteriorEdgeIncidentTetrahedraTest_c_lf_lv,
        ::testing::ValuesIn(TETRAHEDRON_MESH_GET_TEST_PARAMS(INTERIOR_EDGE_C_LF_LV)));

    class GetInteriorEdgeIncidentTetrahedraTest_c_le : public GetInteriorEdgeIncidentTetrahedraTest {};

    TEST_P(GetInteriorEdgeIncidentTetrahedraTest_c_le, each_edge) {
        auto [c, le, _] = GetParam();

        EXPECT_FALSE(compute(c, le));
        check_incident(M.cells.edge_vertex(c, le, 0), M.cells.edge_vertex(c, le, 1));
        check_complete(M.cells.edge_vertex(c, le, 0), M.cells.edge_vertex(c, le, 1));
        check_loop();
    }

    INSTANTIATE_TEST_SUITE_P(
        TetrahedronOperationsTest,
        GetInteriorEdgeIncidentTetrahedraTest_c_le,
        ::testing::ValuesIn(TETRAHEDRON_MESH_GET_TEST_PARAMS(INTERIOR_EDGE_C_LE)));


    class GetBorderEdgeIncidentTetrahedraTest : public GetEdgeIncidentTetrahedraTest {
    public:
        void check_loop() override {
            for (GEO::index_t i = 0, i_end = ordered_c_and_lf.size(); i < i_end; ++i) {
                const auto& [c, lf] = ordered_c_and_lf[i];
                if (i == i_end-1)
                    EXPECT_EQ(M.cells.adjacent(c, lf), GEO::NO_CELL);
                else
                    EXPECT_EQ(M.cells.adjacent(c, lf), ordered_c_and_lf[i+1].first);
            }
        }
    };

    class GetBorderEdgeIncidentTetrahedraTest_c_lf_lv : public GetBorderEdgeIncidentTetrahedraTest {};

    TEST_P(GetBorderEdgeIncidentTetrahedraTest_c_lf_lv, each_c_lf_lv) {
        auto [c, lf, lv] = GetParam();

        EXPECT_TRUE(compute(c, lf, lv));
        check_incident(M.cells.facet_vertex(c, lf, lv), M.cells.facet_vertex(c, lf, (lv+1)%3));
        check_complete(M.cells.facet_vertex(c, lf, lv), M.cells.facet_vertex(c, lf, (lv+1)%3));
        check_loop();
    }

    INSTANTIATE_TEST_SUITE_P(
        TetrahedronOperationsTest,
        GetBorderEdgeIncidentTetrahedraTest_c_lf_lv,
        ::testing::ValuesIn(TETRAHEDRON_MESH_GET_TEST_PARAMS(BORDER_EDGE_C_LF_LV)));

    class GetBorderEdgeIncidentTetrahedraTest_c_le : public GetBorderEdgeIncidentTetrahedraTest {};

    TEST_P(GetBorderEdgeIncidentTetrahedraTest_c_le, each_c_le) {
        auto [c, le, _] = GetParam();

        EXPECT_TRUE(compute(c, le));
        check_incident(M.cells.edge_vertex(c, le, 0), M.cells.edge_vertex(c, le, 1));
        check_complete(M.cells.edge_vertex(c, le, 0), M.cells.edge_vertex(c, le, 1));
        check_loop();
    }

    INSTANTIATE_TEST_SUITE_P(
        TetrahedronOperationsTest,
        GetBorderEdgeIncidentTetrahedraTest_c_le,
        ::testing::ValuesIn(TETRAHEDRON_MESH_GET_TEST_PARAMS(BORDER_EDGE_C_LE)));

    /* == CellSplit14Test ========================================================================================== */

    class CellSplit14Test : public TetrahedronOperationsTest {
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

            cell_split_1_4(
                M,
                c,
                new_v,
                new_c, new_c+1, new_c+2);
        }
    };

    class InteriorCellSplit14Test : public CellSplit14Test {};

    class BorderCellSplit14Test : public CellSplit14Test {};

    TEST_P(InteriorCellSplit14Test, each_cell) {
        auto [c, _, __] = GetParam();

        compute(c);
        check_connections();
        save_results_c(c);
    }

    INSTANTIATE_TEST_SUITE_P(
        TetrahedronOperationsTest,
        InteriorCellSplit14Test,
        ::testing::ValuesIn(TETRAHEDRON_MESH_GET_TEST_PARAMS(INTERIOR_FACET_C_LF)));

    TEST_P(BorderCellSplit14Test, each_cell) {
        auto [c, _, __] = GetParam();

        compute(c);
        check_connections();
        save_results_c(c);
    }

    INSTANTIATE_TEST_SUITE_P(
        TetrahedronOperationsTest,
        BorderCellSplit14Test,
        ::testing::ValuesIn(TETRAHEDRON_MESH_GET_TEST_PARAMS(BORDER_FACET_C_LF)));

    /* == EdgeCollapseTest ========================================================================================= */

    class EdgeCollapseTest : public TetrahedronOperationsTest {
    public:
        void compute(
            const GEO::index_t c,
            const GEO::index_t le,
            const double r
            ) {
            GEO::index_t disuse_v;
            std::vector<GEO::index_t> disuse_cs;

            const GEO::index_t ev0 = M.cells.edge_vertex(c, le, 0);

            edge_collapse(
                M,
                c,
                le,
                r,
                &disuse_v,
                &disuse_cs);

            EXPECT_NE(disuse_v, ev0);
            for (const auto& cc : M.cells) {
                for (GEO::index_t lv = 0; lv < 4; ++lv) {
                    if (M.cells.vertex(cc, lv) == ev0) {
                        M_c_processed[cc] = 1;
                        break;
                    }
                }
            }

            /* Clean disuse vertices and cells */
            GEO::vector<GEO::index_t> cells_to_delete(M.cells.nb(), 0);
            for (const auto& cc : disuse_cs)
                cells_to_delete[cc] = 1;
            M.cells.delete_elements(cells_to_delete);
        }
    };

    class InteriorEdgeCollapseTest : public EdgeCollapseTest {};

    TEST_P(InteriorEdgeCollapseTest, each_edge) {
        auto [c, le, _] = GetParam();

        compute(c, le, GEO::Numeric::random_float32());
        check_connections();
        save_results_c_le(c, le);
    }

    INSTANTIATE_TEST_SUITE_P(
        TetrahedronOperationsTest,
        InteriorEdgeCollapseTest,
        ::testing::ValuesIn(TETRAHEDRON_MESH_GET_TEST_PARAMS(INTERIOR_EDGE_C_LE)));

    class BorderEdgeCollapseTest : public EdgeCollapseTest {};

    TEST_P(BorderEdgeCollapseTest, each_edge) {
        auto [c, le, _] = GetParam();

        compute(c, le, GEO::Numeric::random_float32());
        check_connections();
        save_results_c_le(c, le);
    }

    INSTANTIATE_TEST_SUITE_P(
        TetrahedronOperationsTest,
        BorderEdgeCollapseTest,
        ::testing::ValuesIn(TETRAHEDRON_MESH_GET_TEST_PARAMS(BORDER_EDGE_C_LE)));

    /* == EdgeSwap23Test =========================================================================================== */

    class EdgeSwap23Test : public TetrahedronOperationsTest {
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
        auto [c, lf, _] = GetParam();

        compute(c, lf);
        check_connections();
        save_results_c_lf(c, lf);
    }

    INSTANTIATE_TEST_SUITE_P(
        TetrahedronOperationsTest,
        InteriorEdgeSwap23Test,
        ::testing::ValuesIn(TETRAHEDRON_MESH_GET_TEST_PARAMS(INTERIOR_FACET_C_LF)));

    TEST_P(BorderEdgeSwap23Test, each_facet) {
        auto [c, lf, _] = GetParam();

        compute(c, lf);
        check_connections();
        save_results_c_lf(c, lf);
    }

    INSTANTIATE_TEST_SUITE_P(
        TetrahedronOperationsTest,
        BorderEdgeSwap23Test,
        ::testing::ValuesIn(TETRAHEDRON_MESH_GET_TEST_PARAMS(BORDER_FACET_C_LF)));
}