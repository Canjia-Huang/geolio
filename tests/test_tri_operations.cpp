//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/3/13.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//

#include "test_tri_operations.h"
#include <random>
#include <ranges>
#include "mesh_utils/tri_operations.h"

using namespace GEO::MeshUtils::Tri;

/* == GetIncidentTrianglesTest ===================================================================================== */

namespace GEO::MeshUtils::Test
{
    class GetIncidentTrianglesTest : public TriangleOperationsTest {
    public:
        bool compute(
            const GEO::index_t f,
            const GEO::index_t lv
            ) {
            return get_vertex_incident_triangles(M, f, lv, ordered_f_and_lv);
        }

        void check_incident(
            const GEO::index_t _f,
            const GEO::index_t _lv
            ) {
            const GEO::index_t v = M.facets.vertex(_f, _lv);
            for (const auto& [f, lv] : ordered_f_and_lv)
                EXPECT_EQ(M.facets.vertex(f, lv), v);
        }

        void check_complete(
            const GEO::index_t _f,
            const GEO::index_t _lv
            ) {
            const GEO::index_t v = M.facets.vertex(_f, _lv);

            std::unordered_map<GEO::index_t, bool> incident_facets; // (facet, found)
            for (const auto& f : M.facets) {
                for (GEO::index_t lv = 0; lv < 3; ++lv) {
                    if (M.facets.vertex(f, lv) == v) {
                        incident_facets.emplace(f, false);
                        break;
                    }
                }
            }

            for (const auto &f: ordered_f_and_lv | std::views::keys) {
                auto it = incident_facets.find(f);
                EXPECT_FALSE(it == incident_facets.end());
                it->second = true;
            }

            for (const auto &found: incident_facets | std::views::values)
                EXPECT_TRUE(found);
        }

        std::vector<std::pair<GEO::index_t, GEO::index_t>> ordered_f_and_lv;
    };

    class GetInteriorIncidentTrianglesTest : public GetIncidentTrianglesTest {
    public:
        void check_loop() {
            for (GEO::index_t i = 0, i_end = ordered_f_and_lv.size(); i < i_end; ++i) {
                const auto& [f, lv] =  ordered_f_and_lv[i];
                EXPECT_EQ(M.facets.adjacent(f, lv), ordered_f_and_lv[(i+1)%i_end].first);
            }
        }
    };

    TEST_P(GetInteriorIncidentTrianglesTest, each_vertex) {
        auto [f, lv] = GetParam();

        const bool is_on_border = compute(f, lv);
        EXPECT_FALSE(is_on_border);
        check_incident(f, lv);
        check_complete(f, lv);
        check_loop();
    }

    INSTANTIATE_TEST_SUITE_P(
        TriangleOperationsTest,
        GetInteriorIncidentTrianglesTest,
        ::testing::ValuesIn(TRIANGLE_MESH_GET_TEST_PARAMS(INTERIOR_VERTICES_F_LV)));

    class GetBorderIncidentTrianglesTest : public GetIncidentTrianglesTest {
    public:
        void check_loop() {
            for (GEO::index_t i = 0, i_end = ordered_f_and_lv.size(); i < i_end; ++i) {
                const auto& [f, lv] =  ordered_f_and_lv[i];
                if (i == 0)
                    EXPECT_EQ(M.facets.adjacent(f, (lv+2)%3), GEO::NO_FACET);
                else if (i == i_end-1)
                    EXPECT_EQ(M.facets.adjacent(f, lv), GEO::NO_FACET);
                else
                    EXPECT_EQ(M.facets.adjacent(f, lv), ordered_f_and_lv[i+1].first);
            }
        }
    };

    TEST_P(GetBorderIncidentTrianglesTest, each_vertex) {
        auto [f, lv] = GetParam();

        const bool is_on_border = compute(f, lv);
        EXPECT_TRUE(is_on_border);
        check_incident(f, lv);
        check_complete(f, lv);
        check_loop();
    }

    INSTANTIATE_TEST_SUITE_P(
         TriangleOperationsTest,
         GetBorderIncidentTrianglesTest,
         ::testing::ValuesIn(TRIANGLE_MESH_GET_TEST_PARAMS(BORDER_VERTICES_F_LV)));
}

/* == SplitEdgeTest ================================================================================================ */

namespace GEO::MeshUtils::Test
{
    class SplitEdgeTest : public TriangleOperationsTest {};

    class SplitInteriorEdgeTest : public SplitEdgeTest {
    public:
        void compute(
            const GEO::index_t f,
            const GEO::index_t lv,
            const double r
            ) {
            ASSERT_NE(M.facets.adjacent(f, lv), GEO::NO_FACET);
            const GEO::index_t new_v = M.vertices.create_vertices(1);
            const GEO::index_t new_f = M.facets.create_triangles(2);

            edge_split(
                M,
                f, lv,
                new_v,
                new_f, new_f+1,
                r);
        }
    };

    TEST_P(SplitInteriorEdgeTest, each_edge) {
        auto [f, lv] = GetParam();

        compute(f, lv, GEO::Numeric::random_float32());
        check_connections();
        save_results(f, lv);
    }

    INSTANTIATE_TEST_SUITE_P(
        TriangleOperationsTest,
        SplitInteriorEdgeTest,
        ::testing::ValuesIn(TRIANGLE_MESH_GET_TEST_PARAMS(INTERIOR_EDGES_F_LV)));

    class SplitBorderEdgeTest : public SplitEdgeTest {
    public:
        void compute(
            const GEO::index_t f,
            const GEO::index_t lv,
            const double r
            ) {
            ASSERT_EQ(M.facets.adjacent(f, lv), GEO::NO_FACET);
            const GEO::index_t new_v = M.vertices.create_vertices(1);
            const GEO::index_t new_f = M.facets.create_triangles(1);

            edge_split(
                M,
                f, lv,
                new_v,
                new_f, GEO::NO_FACET,
                r);
        }
    };

    TEST_P(SplitBorderEdgeTest, each_edge) {
        auto [f, lv] = GetParam();

        compute(f, lv, GEO::Numeric::random_float32());
        check_connections();
        save_results(f, lv);
    }

    INSTANTIATE_TEST_SUITE_P(
        TriangleOperationsTest,
        SplitBorderEdgeTest,
        ::testing::ValuesIn(TRIANGLE_MESH_GET_TEST_PARAMS(BORDER_EDGES_F_LV)));
}

/* == CollapseEdgeTest ============================================================================================= */

namespace GEO::MeshUtils::Test
{
    class CollapseEdgeTest : public TriangleOperationsTest {};

    class CollapseInteriorEdgeTest : public CollapseEdgeTest {
    public:
        void compute(
            const GEO::index_t f,
            const GEO::index_t lv,
            const double r
            ) {
            GEO::index_t disuse_v, disuse_f0, disuse_f1;

            edge_collapse(
                M,
                f, lv,
                r,
                &disuse_v,
                &disuse_f0, &disuse_f1);

            /* Clean disuse vertices and facets */
            GEO::vector<GEO::index_t> facets_to_delete(M.facets.nb(), 0);
            facets_to_delete[disuse_f0] = 1;
            EXPECT_NE(disuse_f1, GEO::NO_FACET);
            facets_to_delete[disuse_f1] = 1;
            M.facets.delete_elements(facets_to_delete);
        }
    };

    TEST_P(CollapseInteriorEdgeTest, each_edge) {
        auto [f, lv] = GetParam();

        compute(f, lv, GEO::Numeric::random_float32());
        check_connections();
        save_results(f, lv);
    }

    INSTANTIATE_TEST_SUITE_P(
        TriangleOperationsTest,
        CollapseInteriorEdgeTest,
        ::testing::ValuesIn(TRIANGLE_MESH_GET_TEST_PARAMS(INTERIOR_EDGES_F_LV)));

    class CollapseBorderEdgeTest : public CollapseEdgeTest {
    public:
        void compute(
            const GEO::index_t f,
            const GEO::index_t lv,
            const double r
            ) {
            GEO::index_t disuse_v, disuse_f0, disuse_f1;

            edge_collapse(
                M,
                f, lv,
                r,
                &disuse_v,
                &disuse_f0, &disuse_f1);

            /* Clean disuse vertices and facets */
            GEO::vector<GEO::index_t> facets_to_delete(M.facets.nb(), 0);
            facets_to_delete[disuse_f0] = 1;
            EXPECT_EQ(disuse_f1, GEO::NO_FACET);
            M.facets.delete_elements(facets_to_delete);
        }
    };

    TEST_P(CollapseBorderEdgeTest, each_edge) {
        auto [f, lv] = GetParam();

        compute(f, lv, GEO::Numeric::random_float32());
        check_connections();
        save_results(f, lv);
    }

    INSTANTIATE_TEST_SUITE_P(
         TriangleOperationsTest,
         CollapseBorderEdgeTest,
         ::testing::ValuesIn(TRIANGLE_MESH_GET_TEST_PARAMS(BORDER_EDGES_F_LV)));
}

/* == FlipEdgeTest ================================================================================================= */

namespace GEO::MeshUtils::Test
{
    class FlipEdgeTest : public TriangleOperationsTest {};

    class FlipInteriorEdgeTest : public FlipEdgeTest {
     public:
         void compute(
             const GEO::index_t f,
             const GEO::index_t lv
             ) {
             edge_swap(M, f, lv);
         }
     };

    TEST_P(FlipInteriorEdgeTest, each_edge) {
         auto [f, lv] = GetParam();

         compute(f, lv);
         check_connections();
         save_results(f, lv);
     }

    INSTANTIATE_TEST_SUITE_P(
         TriangleOperationsTest,
         FlipInteriorEdgeTest,
         ::testing::ValuesIn(TRIANGLE_MESH_GET_TEST_PARAMS(INTERIOR_EDGES_F_LV)));
}
