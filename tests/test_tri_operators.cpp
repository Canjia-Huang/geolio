//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/3/13.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//

#include <random>
#include <gtest/gtest.h>
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_io.h>
#include "mesh_utils/tri_operators.h"
#include "utils.h"
#include "common/log.h"

namespace
{
    void build_tri_mesh(
        GEO::Mesh& M
        ) {
        M.clear();

        GEO::index_t new_v = M.vertices.create_vertices(16);
        for (GEO::index_t i = 0; i < 4; ++i) {
            for (GEO::index_t j = 0; j < 4; ++j)
                M.vertices.point(new_v++) = GEO::vec3(j, i, 0);
        }
        M.vertices.point(5).z += 0.5;
        M.vertices.point(10).z += 0.5;

        GEO::index_t new_f = M.facets.create_triangles(18);
        for (GEO::index_t i = 0; i < 3; ++i) {
            for (GEO::index_t j = 0; j < 3; ++j) {
                const GEO::index_t v = 4 * i + j;
                M.facets.set_vertex(new_f, 0, v);
                M.facets.set_vertex(new_f, 1, v + 1);
                M.facets.set_vertex(new_f, 2, v + 4);
                M.facets.set_vertex(new_f + 1, 0, v + 5);
                M.facets.set_vertex(new_f + 1, 1, v + 4);
                M.facets.set_vertex(new_f + 1, 2, v + 1);
                new_f += 2;
            }
        }
        M.facets.connect();
    }

    void get_mesh_f_lv(
        const GEO::Mesh& M,
        std::vector<std::pair<GEO::index_t, GEO::index_t>>& interior_vertices_f_lv,
        std::vector<std::pair<GEO::index_t, GEO::index_t>>& border_vertices_f_lv,
        std::vector<std::pair<GEO::index_t, GEO::index_t>>& interior_edges_f_lv,
        std::vector<std::pair<GEO::index_t, GEO::index_t>>& border_edges_f_lv
        ) {
        /* Find all interior and border f_lv pair */
        std::vector<bool> M_v_border(M.vertices.nb(), false);
        for (const auto& f : M.facets) {
            for (GEO::index_t lv = 0; lv < 3; ++lv) {
                if (M.facets.adjacent(f, lv) == GEO::NO_FACET) {
                    border_edges_f_lv.emplace_back(f, lv);
                    M_v_border[M.facets.vertex(f, lv)] = true;
                    M_v_border[M.facets.vertex(f, (lv+1)%3)] = true;
                }
                else
                    interior_edges_f_lv.emplace_back(f, lv);
            }
        }
        for (const auto& f : M.facets) {
                for (GEO::index_t lv = 0; lv < 3; ++lv) {
                    if (M_v_border[M.facets.vertex(f, lv)])
                        border_vertices_f_lv.emplace_back(f, lv);
                    else
                        interior_vertices_f_lv.emplace_back(f, lv);
                }
            }
    }
}

namespace GEO::MeshUtils::Test
{
    using namespace GEO::MeshUtils;

    class TriangleOperatorsTest : public ::testing::TestWithParam<std::pair<GEO::index_t, GEO::index_t>> {
        void SetUp() override {
            build_tri_mesh(M);
        }

    public:
        void check_connections() {
            std::vector<GEO::index_t> current_connections(3*M.facets.nb(), GEO::NO_FACET);
            for (const auto& f : M.facets) {
                for (GEO::index_t lv = 0; lv < 3; ++lv)
                    current_connections[3*f+lv] = M.facets.adjacent(f, lv);
            }

            M.facets.connect();
            for (const auto& f : M.facets) {
                for (GEO::index_t lv = 0; lv < 3; ++lv)
                    EXPECT_EQ(current_connections[3*f+lv], M.facets.adjacent(f, lv));
            }

            /* Rollback adjacency */
            for (const auto& f : M.facets) {
                for (GEO::index_t lv = 0; lv < 3; ++lv)
                    M.facets.set_adjacent(f, lv, current_connections[3*f+lv]);
            }
        }

        void save_results(
            const GEO::index_t f,
            const GEO::index_t lv
            ) const {
            EXPECT_TRUE(GEO::mesh_save(M, get_current_test_name() +
                                            "_f" + std::to_string(f) +
                                            "_lv" + std::to_string(lv) +
                                            ".geogram"));
        }

        GEO::Mesh M;
    };

    /* == For INSTANTIATE_TEST_SUITE =============================================================================== */

    enum TEST_F_LV_TYPE {
        INTERIOR_VERTICES_F_LV,
        BORDER_VERTICES_F_LV,
        INTERIOR_EDGES_F_LV,
        BORDER_EDGES_F_LV
    };

    const auto GET_TEST_PARAMS = [](
        const TEST_F_LV_TYPE type
        ) {
        GEO::Mesh M;
        build_tri_mesh(M);

        std::vector<std::pair<GEO::index_t, GEO::index_t>> interior_vertices_f_lv;
        std::vector<std::pair<GEO::index_t, GEO::index_t>> border_vertices_f_lv;
        std::vector<std::pair<GEO::index_t, GEO::index_t>> interior_edges_f_lv;
        std::vector<std::pair<GEO::index_t, GEO::index_t>> border_edges_f_lv;

        get_mesh_f_lv(
            M,
            interior_vertices_f_lv,
            border_vertices_f_lv,
            interior_edges_f_lv,
            border_edges_f_lv
        );

        if (type == INTERIOR_VERTICES_F_LV)
            return interior_vertices_f_lv;
        if (type == BORDER_VERTICES_F_LV)
            return border_vertices_f_lv;
        if (type == INTERIOR_EDGES_F_LV)
            return interior_edges_f_lv;
        if (type == BORDER_EDGES_F_LV)
            return border_edges_f_lv;
        assert(0);
    };

    /* == GetIncidentTrianglesTest ================================================================================= */

    class GetIncidentTrianglesTest : public TriangleOperatorsTest {
    public:
        bool compute(
            const GEO::index_t start_f,
            const GEO::index_t start_lv
            ) {
            return get_vertex_incident_triangles(M, start_f, start_lv, ordered_f_and_lv);
        }

        void check_incident(
            const GEO::index_t start_f,
            const GEO::index_t start_lv
            ) {
            const GEO::index_t v = M.facets.vertex(start_f, start_lv);
            for (const auto& [f, lv] : ordered_f_and_lv)
                EXPECT_EQ(M.facets.vertex(f, lv), v);
        }

        virtual void check_loop() = 0;

        std::vector<std::pair<GEO::index_t, GEO::index_t>> ordered_f_and_lv;
    };

    class GetInteriorIncidentTrianglesTest : public GetIncidentTrianglesTest {
    public:
        void check_loop() override {
            for (GEO::index_t i = 0, i_end = ordered_f_and_lv.size(); i < i_end; ++i) {
                const auto& [f, lv] =  ordered_f_and_lv[i];
                EXPECT_EQ(M.facets.adjacent(f, lv), ordered_f_and_lv[(i+1)%i_end].first);
            }
        }
    };

    class GetBorderIncidentTrianglesTest : public GetIncidentTrianglesTest {
    public:
        void check_loop() override {
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

    TEST_P(GetInteriorIncidentTrianglesTest, each_vertex) {
        auto [f, lv] = GetParam();

        EXPECT_FALSE(compute(f, lv));
        check_incident(f, lv);
        check_loop();
    }

    TEST_P(GetBorderIncidentTrianglesTest, each_vertex) {
        auto [f, lv] = GetParam();

        EXPECT_TRUE(compute(f, lv));
        check_incident(f, lv);
        check_loop();
    }

    INSTANTIATE_TEST_SUITE_P(
        TriangleOperatorsTest,
        GetInteriorIncidentTrianglesTest,
        ::testing::ValuesIn(GET_TEST_PARAMS(INTERIOR_VERTICES_F_LV))
    );

    INSTANTIATE_TEST_SUITE_P(
        TriangleOperatorsTest,
        GetBorderIncidentTrianglesTest,
        ::testing::ValuesIn(GET_TEST_PARAMS(BORDER_VERTICES_F_LV))
    );

    /* == SplitEdgeTest ============================================================================================ */

    class SplitEdgeTest : public TriangleOperatorsTest {
    public:
        virtual void compute(
            GEO::index_t f,
            GEO::index_t lv,
            double r) = 0;
    };

    class SplitInteriorEdgeTest : public SplitEdgeTest {
    public:
        void compute(
            const GEO::index_t f,
            const GEO::index_t lv,
            const double r
            ) override {
            ASSERT_NE(M.facets.adjacent(f, lv), GEO::NO_FACET);
            const GEO::index_t new_v = M.vertices.create_vertices(1);
            const GEO::index_t new_f = M.facets.create_triangles(2);

            edge_split(
                M,
                f, lv,
                r,
                new_v,
                new_f, new_f+1);
        }
    };

    class SplitBorderEdgeTest : public SplitEdgeTest {
    public:
        void compute(
            const GEO::index_t f,
            const GEO::index_t lv,
            const double r
            ) override {
            ASSERT_EQ(M.facets.adjacent(f, lv), GEO::NO_FACET);
            const GEO::index_t new_v = M.vertices.create_vertices(1);
            const GEO::index_t new_f = M.facets.create_triangles(1);

            edge_split(
                M,
                f, lv,
                r,
                new_v,
                new_f, GEO::NO_FACET);
        }
    };

    TEST_P(SplitInteriorEdgeTest, each_edge) {
        auto [f, lv] = GetParam();

        compute(f, lv, GEO::Numeric::random_float32());
        check_connections();
        save_results(f, lv);
    }

    TEST_P(SplitBorderEdgeTest, each_edge) {
        auto [f, lv] = GetParam();

        compute(f, lv, GEO::Numeric::random_float32());
        check_connections();
        save_results(f, lv);
    }

    INSTANTIATE_TEST_SUITE_P(
        TriangleOperatorsTest,
        SplitInteriorEdgeTest,
        ::testing::ValuesIn(GET_TEST_PARAMS(INTERIOR_EDGES_F_LV))
    );

    INSTANTIATE_TEST_SUITE_P(
        TriangleOperatorsTest,
        SplitBorderEdgeTest,
        ::testing::ValuesIn(GET_TEST_PARAMS(BORDER_EDGES_F_LV))
    );

    /* == CollapseEdgeTest ========================================================================================= */

    class CollapseEdgeTest : public TriangleOperatorsTest {
    public:
        virtual void compute(
            GEO::index_t f,
            GEO::index_t lv,
            double r) = 0;
    };

    class CollapseInteriorEdgeTest : public CollapseEdgeTest {
    public:
        void compute(
            const GEO::index_t f,
            const GEO::index_t lv,
            const double r
            ) override {
            GEO::index_t disuse_v, disuse_f0, disuse_f1;

            edge_collapse(
                M,
                f, lv,
                r,
                disuse_v,
                disuse_f0, disuse_f1);

            /* Clean disuse vertices and facets */
            GEO::vector<GEO::index_t> facets_to_delete(M.facets.nb(), 0);
            facets_to_delete[disuse_f0] = 1;
            EXPECT_NE(disuse_f1, GEO::NO_FACET);
            facets_to_delete[disuse_f1] = 1;
            M.facets.delete_elements(facets_to_delete);
        }
    };

    class CollapseBorderEdgeTest : public CollapseEdgeTest {
    public:
        void compute(
            const GEO::index_t f,
            const GEO::index_t lv,
            const double r
            ) override {
            GEO::index_t disuse_v, disuse_f0, disuse_f1;

            edge_collapse(
                M,
                f, lv,
                r,
                disuse_v,
                disuse_f0, disuse_f1);

            /* Clean disuse vertices and facets */
            GEO::vector<GEO::index_t> facets_to_delete(M.facets.nb(), 0);
            facets_to_delete[disuse_f0] = 1;
            EXPECT_EQ(disuse_f1, GEO::NO_FACET);
            M.facets.delete_elements(facets_to_delete);
        }
    };

    TEST_P(CollapseInteriorEdgeTest, each_edge) {
        auto [f, lv] = GetParam();

        compute(f, lv, GEO::Numeric::random_float32());
        check_connections();
        save_results(f, lv);
    }

    TEST_P(CollapseBorderEdgeTest, each_edge) {
        auto [f, lv] = GetParam();

        compute(f, lv, GEO::Numeric::random_float32());
        check_connections();
        save_results(f, lv);
    }

    INSTANTIATE_TEST_SUITE_P(
        TriangleOperatorsTest,
        CollapseInteriorEdgeTest,
        ::testing::ValuesIn(GET_TEST_PARAMS(INTERIOR_EDGES_F_LV))
    );

    INSTANTIATE_TEST_SUITE_P(
        TriangleOperatorsTest,
        CollapseBorderEdgeTest,
        ::testing::ValuesIn(GET_TEST_PARAMS(BORDER_EDGES_F_LV))
    );

    /* == FlipEdgeTest ============================================================================================= */

    class FlipEdgeTest : public TriangleOperatorsTest {
    public:
        virtual void compute(
            GEO::index_t f,
            GEO::index_t lv) = 0;
    };

    class FlipInteriorEdgeTest : public FlipEdgeTest {
    public:
        void compute(
            const GEO::index_t f,
            const GEO::index_t lv
            ) override {
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
        TriangleOperatorsTest,
        FlipInteriorEdgeTest,
        ::testing::ValuesIn(GET_TEST_PARAMS(INTERIOR_EDGES_F_LV))
    );
}
