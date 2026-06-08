//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/3/13.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//

#include "test_tri_operations.h"
#include <random>
#include <ranges>
#include "geolio/tri_operations.h"

/* == GetIncidentTrianglesTest ===================================================================================== */

namespace geolio::test
{
    class GetIncidentTrianglesTest : public TriangleOperationsTest {
    public:
        bool compute(
            const GEO::index_t f,
            const GEO::index_t lv
            ) {
            return get_vertex_incident_facets(M, f, lv, ordered_f_and_lv);
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

/* == SplitFacetEdgeTest =========================================================================================== */

namespace geolio::test
{
    class SplitFacetEdgeTest : public TriangleOperationsTest {};

    class SplitInteriorFacetEdgeTest : public SplitFacetEdgeTest {
    public:
        void compute(
            const GEO::index_t f,
            const GEO::index_t lv,
            const double r
            ) {
            ASSERT_NE(M.facets.adjacent(f, lv), GEO::NO_FACET);
            const GEO::index_t new_v = M.vertices.create_vertices(1);
            const GEO::index_t new_f = M.facets.create_triangles(2);

            facet_edge_split(
                M,
                f, lv,
                new_v,
                new_f, new_f+1,
                r);
        }
    };

    TEST_P(SplitInteriorFacetEdgeTest, each_edge) {
        auto [f, lv] = GetParam();

        compute(f, lv, GEO::Numeric::random_float32());
        check_connections();
        save_results(f, lv);
    }

    INSTANTIATE_TEST_SUITE_P(
        TriangleOperationsTest,
        SplitInteriorFacetEdgeTest,
        ::testing::ValuesIn(TRIANGLE_MESH_GET_TEST_PARAMS(INTERIOR_EDGES_F_LV)));

    class SplitBorderFacetEdgeTest : public SplitFacetEdgeTest {
    public:
        void compute(
            const GEO::index_t f,
            const GEO::index_t lv,
            const double r
            ) {
            ASSERT_EQ(M.facets.adjacent(f, lv), GEO::NO_FACET);
            const GEO::index_t new_v = M.vertices.create_vertices(1);
            const GEO::index_t new_f = M.facets.create_triangles(1);

            facet_edge_split(
                M,
                f, lv,
                new_v,
                new_f, GEO::NO_FACET,
                r);
        }
    };

    TEST_P(SplitBorderFacetEdgeTest, each_edge) {
        auto [f, lv] = GetParam();

        compute(f, lv, GEO::Numeric::random_float32());
        check_connections();
        save_results(f, lv);
    }

    INSTANTIATE_TEST_SUITE_P(
        TriangleOperationsTest,
        SplitBorderFacetEdgeTest,
        ::testing::ValuesIn(TRIANGLE_MESH_GET_TEST_PARAMS(BORDER_EDGES_F_LV)));
}

/* == CollapseFacetEdgeTest ======================================================================================== */

namespace geolio::test
{
    class CollapseFacetEdgeTest : public TriangleOperationsTest {};

    class CollapseInteriorFacetEdgeTest : public CollapseFacetEdgeTest {
    public:
        void compute(
            const GEO::index_t f,
            const GEO::index_t lv,
            const double r
            ) {
            GEO::index_t disuse_v, disuse_f0, disuse_f1;

            facet_edge_collapse(
                M,
                f, lv,
                disuse_v,
                disuse_f0, disuse_f1,
                r);

            /* Clean disuse vertices and facets */
            GEO::vector<GEO::index_t> facets_to_delete(M.facets.nb(), 0);
            facets_to_delete[disuse_f0] = 1;
            EXPECT_NE(disuse_f1, GEO::NO_FACET);
            facets_to_delete[disuse_f1] = 1;
            M.facets.delete_elements(facets_to_delete);
        }
    };

    TEST_P(CollapseInteriorFacetEdgeTest, each_edge) {
        auto [f, lv] = GetParam();

        compute(f, lv, GEO::Numeric::random_float32());
        check_connections();
        save_results(f, lv);
    }

    INSTANTIATE_TEST_SUITE_P(
        TriangleOperationsTest,
        CollapseInteriorFacetEdgeTest,
        ::testing::ValuesIn(TRIANGLE_MESH_GET_TEST_PARAMS(INTERIOR_EDGES_F_LV)));

    class CollapseBorderFacetEdgeTest : public CollapseFacetEdgeTest {
    public:
        void compute(
            const GEO::index_t f,
            const GEO::index_t lv,
            const double r
            ) {
            GEO::index_t disuse_v, disuse_f0, disuse_f1;

            facet_edge_collapse(
                M,
                f, lv,
                disuse_v,
                disuse_f0, disuse_f1,
                r);

            /* Clean disuse vertices and facets */
            GEO::vector<GEO::index_t> facets_to_delete(M.facets.nb(), 0);
            facets_to_delete[disuse_f0] = 1;
            EXPECT_EQ(disuse_f1, GEO::NO_FACET);
            M.facets.delete_elements(facets_to_delete);
        }
    };

    TEST_P(CollapseBorderFacetEdgeTest, each_edge) {
        auto [f, lv] = GetParam();

        compute(f, lv, GEO::Numeric::random_float32());
        check_connections();
        save_results(f, lv);
    }

    INSTANTIATE_TEST_SUITE_P(
         TriangleOperationsTest,
         CollapseBorderFacetEdgeTest,
         ::testing::ValuesIn(TRIANGLE_MESH_GET_TEST_PARAMS(BORDER_EDGES_F_LV)));
}

/* == FlipFacetEdgeTest ============================================================================================ */

namespace geolio::test
{
    class FlipFacetdgeTest : public TriangleOperationsTest {
    public:
        bool compute(
            const GEO::index_t f,
            const GEO::index_t lv
            ) {
            return facet_edge_swap(M, f, lv);
        }
    };

    class FlipInteriorFacetEdgeTest : public FlipFacetdgeTest {};

    TEST_P(FlipInteriorFacetEdgeTest, each_edge) {
         auto [f, lv] = GetParam();

         EXPECT_TRUE(compute(f, lv));
         check_connections();
         save_results(f, lv);
     }

    INSTANTIATE_TEST_SUITE_P(
         TriangleOperationsTest,
         FlipInteriorFacetEdgeTest,
         ::testing::ValuesIn(TRIANGLE_MESH_GET_TEST_PARAMS(INTERIOR_EDGES_F_LV)));

    class FlipBorderFacetEdgeTest : public FlipFacetdgeTest {};

    TEST_P(FlipBorderFacetEdgeTest, each_edge) {
        auto [f, lv] = GetParam();

        EXPECT_FALSE(compute(f, lv));
        check_connections();
    }

    INSTANTIATE_TEST_SUITE_P(
         TriangleOperationsTest,
         FlipBorderFacetEdgeTest,
         ::testing::ValuesIn(TRIANGLE_MESH_GET_TEST_PARAMS(BORDER_EDGES_F_LV)));
}
