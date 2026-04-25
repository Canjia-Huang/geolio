//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/3/23.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//

#include <ranges>
#include <unordered_set>
#include "test_tet_opeartions.h"
#include "mesh_utils/tet_operations.h"

using namespace GEO::MeshUtils::Tet;

/* == GetVertexIncidentTetrahedraTest ============================================================================== */

namespace GEO::MeshUtils::Test
{
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
}

/* == GetEdgeIncidentTetrahedraTest ================================================================================ */

namespace GEO::MeshUtils::Test
{
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
}

/* == CellSplitTest ================================================================================================ */

namespace GEO::MeshUtils::Test
{
    class CellSplitTest : public TetrahedronOperationsTest {
    public:
        void compute(
            const GEO::index_t c
            ) {
            const GEO::index_t new_v = M.vertices.create_vertices(1);
            const GEO::index_t new_c = M.cells.create_tets(3);

            M_c_affected[c] = 1;
            M_c_affected[new_c] = 1;
            M_c_affected[new_c+1] = 1;
            M_c_affected[new_c+2] = 1;

            cell_split(
                M,
                c,
                new_v,
                new_c, new_c+1, new_c+2);
        }
    };

    class InteriorCellSplitTest : public CellSplitTest {};

    TEST_P(InteriorCellSplitTest, each_cell) {
        auto [c, _, __] = GetParam();

        compute(c);
        check_connections();
        save_results_c(c);
    }

    INSTANTIATE_TEST_SUITE_P(
        TetrahedronOperationsTest,
        InteriorCellSplitTest,
        ::testing::ValuesIn(TETRAHEDRON_MESH_GET_TEST_PARAMS(INTERIOR_FACET_C_LF)));

    class BorderCellSplitTest : public CellSplitTest {};

    TEST_P(BorderCellSplitTest, each_cell) {
        auto [c, _, __] = GetParam();

        compute(c);
        check_connections();
        save_results_c(c);
    }

    INSTANTIATE_TEST_SUITE_P(
        TetrahedronOperationsTest,
        BorderCellSplitTest,
        ::testing::ValuesIn(TETRAHEDRON_MESH_GET_TEST_PARAMS(BORDER_FACET_C_LF)));
}

/* == CellFacetSplit =============================================================================================== */

namespace GEO::MeshUtils::Test
{
    class CellFacetSplitTest : public TetrahedronOperationsTest {};

    class InteriorCellFacetSplitTest : public CellFacetSplitTest {
    public:
        void compute(
            const GEO::index_t c,
            const GEO::index_t lf
            ) {
            ASSERT_FALSE(M.cells.adjacent(c, lf) == GEO::NO_CELL);
            const GEO::index_t new_v = M.vertices.create_vertices(1);
            const GEO::index_t new_c = M.cells.create_tets(4);

            M_c_affected[c] = 1;
            M_c_affected[M.cells.adjacent(c, lf)] = 1;
            M_c_affected[new_c] = 1;
            M_c_affected[new_c+1] = 1;
            M_c_affected[new_c+2] = 1;
            M_c_affected[new_c+3] = 1;

            cell_facet_split(M, c, lf, new_v, new_c, new_c+1, new_c+2, new_c+3);
        }
    };

    TEST_P(InteriorCellFacetSplitTest, each_facet) {
        auto [c, lf, __] = GetParam();

        compute(c, lf);
        check_connections();
        save_results_c_lf(c, lf);
    }

    INSTANTIATE_TEST_SUITE_P(
        TetrahedronOperationsTest,
        InteriorCellFacetSplitTest,
        ::testing::ValuesIn(TETRAHEDRON_MESH_GET_TEST_PARAMS(INTERIOR_FACET_C_LF)));

    class BorderCellFacetSplitTest : public CellFacetSplitTest {
    public:
        void compute(
            const GEO::index_t c,
            const GEO::index_t lf
            ) {
            ASSERT_TRUE(M.cells.adjacent(c, lf) == GEO::NO_CELL);
            const GEO::index_t new_v = M.vertices.create_vertices(1);
            const GEO::index_t new_c = M.cells.create_tets(2);

            M_c_affected[c] = 1;
            M_c_affected[new_c] = 1;
            M_c_affected[new_c+1] = 1;

            cell_facet_split(M, c, lf, new_v, new_c, new_c+1);
        }
    };

    TEST_P(BorderCellFacetSplitTest, each_facet) {
        auto [c, lf, __] = GetParam();

        compute(c, lf);
        check_connections();
        save_results_c_lf(c, lf);
    }

    INSTANTIATE_TEST_SUITE_P(
        TetrahedronOperationsTest,
        BorderCellFacetSplitTest,
        ::testing::ValuesIn(TETRAHEDRON_MESH_GET_TEST_PARAMS(BORDER_FACET_C_LF)));
}

/* == CellEdgeSplitTest ============================================================================================ */

namespace GEO::MeshUtils::Test
{
    class CellEdgeSplitTest : public TetrahedronOperationsTest {
    public:
        void compute(
            const GEO::index_t c,
            const GEO::index_t le
            ) {
            const GEO::index_t ev0 = M.cells.edge_vertex(c, le, 0);
            const GEO::index_t ev1 = M.cells.edge_vertex(c, le, 1);
            for (const auto& cc : M.cells) {
                for (GEO::index_t lle = 0; lle < 6; ++lle) {
                    const GEO::index_t eev0 = M.cells.edge_vertex(cc, lle, 0);
                    const GEO::index_t eev1 = M.cells.edge_vertex(cc, lle, 1);
                    if ((eev0 == ev0 && eev1 == ev1) || (eev0 == ev1 && eev1 == ev0)) {
                        M_c_affected[cc] = 1;
                        break;
                    }
                }
            }

            const GEO::index_t new_v = M.vertices.create_vertices(1);
            const GEO::index_t new_cells_nb = 10.0*GEO::Numeric::random_float32();
            const GEO::index_t new_c = M.cells.create_tets(new_cells_nb);

            std::vector<GEO::index_t> new_cs;
            for (GEO::index_t cc = new_c; cc < M.cells.nb(); ++cc) {
                new_cs.push_back(cc);
                M_c_affected[cc] = 1;
            }

            cell_edge_split(M, c, le, new_v, new_cs, GEO::Numeric::random_float32());

            /* Delete unuse cells */
            GEO::vector<GEO::index_t> cells_to_delete(M.cells.nb(), 0);
            for (const auto& cc : new_cs) {
                if (cc != GEO::NO_CELL) {
                    for (GEO::index_t lv = 0; lv < 4; ++lv)
                        EXPECT_TRUE(M.cells.vertex(cc, lv) == GEO::NO_VERTEX);
                    cells_to_delete[cc] = 1;
                }
            }
            M.cells.delete_elements(cells_to_delete);
        }
    };

    class InteriorCellEdgeSplitTest : public CellEdgeSplitTest {};

    TEST_P(InteriorCellEdgeSplitTest, each_edge) {
        auto [c, le, __] = GetParam();

        compute(c, le);
        check_connections();
        save_results_c_le(c, le);
    }

    INSTANTIATE_TEST_SUITE_P(
        TetrahedronOperationsTest,
        InteriorCellEdgeSplitTest,
        ::testing::ValuesIn(TETRAHEDRON_MESH_GET_TEST_PARAMS(INTERIOR_EDGE_C_LE)));

    class BorderCellEdgeSplitTest : public CellEdgeSplitTest {};

    TEST_P(BorderCellEdgeSplitTest, each_edge) {
        auto [c, le, __] = GetParam();

        compute(c, le);
        check_connections();
        save_results_c_le(c, le);
    }

    INSTANTIATE_TEST_SUITE_P(
        TetrahedronOperationsTest,
        BorderCellEdgeSplitTest,
        ::testing::ValuesIn(TETRAHEDRON_MESH_GET_TEST_PARAMS(BORDER_EDGE_C_LE)));
}

/* == CellEdgeCollapseTest ========================================================================================= */

namespace GEO::MeshUtils::Test
{
    class CellEdgeCollapseTest : public TetrahedronOperationsTest {
    public:
        void compute(
            const GEO::index_t c,
            const GEO::index_t le,
            const double r
            ) {
            GEO::index_t disuse_v;
            std::vector<GEO::index_t> disuse_cs;

            const GEO::index_t ev0 = M.cells.edge_vertex(c, le, 0);

            cell_edge_collapse(
                M,
                c,
                le,
                disuse_v,
                disuse_cs,
                r);

            EXPECT_NE(disuse_v, ev0);
            for (const auto& cc : M.cells) {
                for (GEO::index_t lv = 0; lv < 4; ++lv) {
                    if (M.cells.vertex(cc, lv) == ev0) {
                        M_c_affected[cc] = 1;
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

    class InteriorCellEdgeCollapseTest : public CellEdgeCollapseTest {};

    TEST_P(InteriorCellEdgeCollapseTest, each_edge) {
        auto [c, le, _] = GetParam();

        compute(c, le, GEO::Numeric::random_float32());
        check_connections();
        save_results_c_le(c, le);
    }

    INSTANTIATE_TEST_SUITE_P(
        TetrahedronOperationsTest,
        InteriorCellEdgeCollapseTest,
        ::testing::ValuesIn(TETRAHEDRON_MESH_GET_TEST_PARAMS(INTERIOR_EDGE_C_LE)));

    class BorderCellEdgeCollapseTest : public CellEdgeCollapseTest {};

    TEST_P(BorderCellEdgeCollapseTest, each_edge) {
        auto [c, le, _] = GetParam();

        compute(c, le, GEO::Numeric::random_float32());
        check_connections();
        save_results_c_le(c, le);
    }

    INSTANTIATE_TEST_SUITE_P(
        TetrahedronOperationsTest,
        BorderCellEdgeCollapseTest,
        ::testing::ValuesIn(TETRAHEDRON_MESH_GET_TEST_PARAMS(BORDER_EDGE_C_LE)));
}

/* == CellEdgeSwap23Test =========================================================================================== */

namespace GEO::MeshUtils::Test
{
    class CellEdgeSwap23Test : public TetrahedronOperationsTest {};

    class InteriorCellEdgeSwap23Test : public CellEdgeSwap23Test {
    public:
        bool compute(
            const GEO::index_t c,
            const GEO::index_t lf
            ) {
            const GEO::index_t new_c = M.cells.create_tets(1);

            M_c_affected[c] = 1;
            M_c_affected[M.cells.adjacent(c, lf)] = 1;
            M_c_affected[new_c] = 1;

            return cell_edge_swap_2_3(
                M,
                c, lf,
                new_c);
        }
    };

    TEST_F(InteriorCellEdgeSwap23Test, test) {
        const GEO::index_t c = 1;
        const GEO::index_t lf = 1;
        EXPECT_TRUE(compute(c, lf));
        check_connections();
        save_results_c_lf(c, lf);
    }

    TEST_P(InteriorCellEdgeSwap23Test, each_facet) {
        auto [c, lf, _] = GetParam();

        EXPECT_TRUE(compute(c, lf));
        check_connections();
        save_results_c_lf(c, lf);
    }

    INSTANTIATE_TEST_SUITE_P(
        TetrahedronOperationsTest,
        InteriorCellEdgeSwap23Test,
        ::testing::ValuesIn(TETRAHEDRON_MESH_GET_TEST_PARAMS(INTERIOR_FACET_C_LF)));

    class BorderCellEdgeSwap23Test : public CellEdgeSwap23Test {
    public:
        bool compute(
            const GEO::index_t c,
            const GEO::index_t lf
            ) {
            M_c_affected[c] = 1;

            return cell_edge_swap_2_3(
                M,
                c, lf,
                GEO::NO_CELL);
        }
    };

    TEST_P(BorderCellEdgeSwap23Test, each_facet) {
        auto [c, lf, _] = GetParam();

        EXPECT_FALSE(compute(c, lf));
        check_connections();
    }

    INSTANTIATE_TEST_SUITE_P(
        TetrahedronOperationsTest,
        BorderCellEdgeSwap23Test,
        ::testing::ValuesIn(TETRAHEDRON_MESH_GET_TEST_PARAMS(BORDER_FACET_C_LF)));
}

/* == CellEdgeSwap32Test =========================================================================================== */

namespace GEO::MeshUtils::Test
{
    class CellEdgeSwap32Test : public TetrahedronOperationsTest {
    public:
        void compute(
            const GEO::index_t c,
            const GEO::index_t le,
            bool& processed
            ) {
            std::vector<std::pair<GEO::index_t, GEO::index_t>> ordered_c_and_lf;
            get_edge_incident_tetrahedra(M, c, le, ordered_c_and_lf);
            for (const auto &cc: ordered_c_and_lf | std::views::keys)
                M_c_affected[cc] = 1;

            GEO::index_t disuse_c = GEO::NO_CELL;

            processed = cell_edge_swap_3_2(
                M,
                c,
                le,
                disuse_c);

            if (processed) {
                /* Clean disuse vertices and cells */
                ASSERT_LT(disuse_c, M.cells.nb());
                GEO::vector<GEO::index_t> cells_to_delete(M.cells.nb(), 0);
                cells_to_delete[disuse_c] = 1;
                M.cells.delete_elements(cells_to_delete);
            }
        }
    };

    TEST_P(CellEdgeSwap32Test, each_edge) {
        auto [c, le, _] = GetParam();

        bool processed;
        compute(c, le, processed);

        if (processed) {
            check_connections();
            save_results_c_le(c, le);
        }
    }

    INSTANTIATE_TEST_SUITE_P(
        TetrahedronOperationsTest,
        CellEdgeSwap32Test,
        ::testing::ValuesIn(TETRAHEDRON_MESH_GET_TEST_PARAMS(INTERIOR_EDGE_C_LE)));
}