//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/3/13.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//

#include <gtest/gtest.h>
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_io.h>
#include "mesh_utils/tri_operators.h"
#include "utils.h"
#include "common/log.h"

namespace GEO::MeshUtils::Test
{
    using namespace GEO::MeshUtils;

    class TriangleOperatorsTest : public ::testing::Test {
        void SetUp() override {
            ASSERT_TRUE(GEO::mesh_load(std::string(TEST_DATA_PATH) + "simple_tri_mesh.obj", M_));
            LOG::DEBUG("M #V:{}, #F: {}", M_.vertices.nb(), M_.facets.nb());

            M_v_to_delete_.bind(M_.vertices.attributes(), "delete");
            M_f_to_delete_.bind(M_.facets.attributes(), "delete");
        }
    public:
        void clean_delete_elements() {
            auto facets_to_delete = M_f_to_delete_.get_vector();
            M_.facets.delete_elements(facets_to_delete);
        }

        void check_connections() {
            std::vector<GEO::index_t> current_connections(3*M_.facets.nb(), GEO::NO_FACET);
            for (const auto& f : M_.facets) {
                for (GEO::index_t lv = 0; lv < 3; ++lv)
                    current_connections[3*f+lv] = M_.facets.adjacent(f, lv);
            }

            M_.facets.connect();
            for (const auto& f : M_.facets) {
                for (GEO::index_t lv = 0; lv < 3; ++lv)
                    EXPECT_EQ(current_connections[3*f+lv], M_.facets.adjacent(f, lv));
            }

            /* Rollback adjacency */
            for (const auto& f : M_.facets) {
                for (GEO::index_t lv = 0; lv < 3; ++lv)
                    M_.facets.set_adjacent(f, lv, current_connections[3*f+lv]);
            }
        }

        void save_results() const {
            EXPECT_TRUE(GEO::mesh_save(M_, get_current_test_name()+".geogram"));
        }

        GEO::Mesh M_;
        GEO::Attribute<GEO::index_t> M_v_to_delete_;
        GEO::Attribute<GEO::index_t> M_f_to_delete_;
    };

    /* == GetIncidentTrianglesTest ================================================================================= */
    class GetIncidentTrianglesTest : public TriangleOperatorsTest {
    public:
        void compute(
            const GEO::index_t start_f,
            const GEO::index_t start_lv
            ) {
            start_f_ = start_f;
            start_lv_ = start_lv;
            get_vertex_incident_triangles(M_, start_f_, start_lv_, ordered_f_and_lv);
            check_incident();
        }

        void check_incident() {
            const GEO::index_t v = M_.facets.vertex(start_f_, start_lv_);
            for (const auto& [f, lv] : ordered_f_and_lv)
                EXPECT_EQ(M_.facets.vertex(f, lv), v);
        }

        void check_loop() {
            for (GEO::index_t i = 0, i_end = ordered_f_and_lv.size(); i < i_end; ++i) {
                const auto& [f, lv] =  ordered_f_and_lv[i];
                EXPECT_EQ(M_.facets.adjacent(f, lv), ordered_f_and_lv[(i+1)%i_end].first);
            }
        }

        void check_loop_on_border() {
            for (GEO::index_t i = 0, i_end = ordered_f_and_lv.size(); i < i_end; ++i) {
                const auto& [f, lv] =  ordered_f_and_lv[i];
                if (i == 0)
                    EXPECT_EQ(M_.facets.adjacent(f, (lv+2)%3), GEO::NO_FACET);
                else if (i == i_end-1)
                    EXPECT_EQ(M_.facets.adjacent(f, lv), GEO::NO_FACET);
                else
                    EXPECT_EQ(M_.facets.adjacent(f, lv), ordered_f_and_lv[i+1].first);
            }
        }

        std::vector<std::pair<GEO::index_t, GEO::index_t>> ordered_f_and_lv;
        GEO::index_t start_f_{};
        GEO::index_t start_lv_{};
    };

    TEST_F(GetIncidentTrianglesTest, interior_vertex) {
        compute(4, 2);
        check_incident();
        check_loop();
    }

    TEST_F(GetIncidentTrianglesTest, get_incident_triangles_border) {
        compute(0, 1);
        check_incident();
        check_loop_on_border();
    }

    /* == SplitEdgeTest ============================================================================================ */
    class SplitEdgeTest : public TriangleOperatorsTest {
    public:
        void compute(
            const GEO::index_t f,
            const GEO::index_t lv,
            const double r
            ) {
            ASSERT_NE(M_.facets.adjacent(f, lv), GEO::NO_FACET);
            const GEO::index_t new_v = M_.vertices.create_vertices(1);
            const GEO::index_t new_f = M_.facets.create_triangles(2);

            edge_split(
                M_,
                f, lv,
                r,
                new_v,
                new_f, new_f+1);
        }

        void compute_on_border(
            const GEO::index_t f,
            const GEO::index_t lv,
            const double r
            ) {
            ASSERT_EQ(M_.facets.adjacent(f, lv), GEO::NO_FACET);
            const GEO::index_t new_v = M_.vertices.create_vertices(1);
            const GEO::index_t new_f = M_.facets.create_triangles(1);

            edge_split(
                M_,
                f, lv,
                r,
                new_v,
                new_f, GEO::NO_FACET);
        }
    };

    TEST_F(SplitEdgeTest, split_edge_05) {
        compute(4, 2, 0.5);
        check_connections();
        save_results();
    }

    TEST_F(SplitEdgeTest, split_edge_02) {
        compute(4, 2, 0.2);
        check_connections();
        save_results();
    }

    TEST_F(SplitEdgeTest, split_edge_07_border) {
        compute_on_border(0, 1, 0.7);
        check_connections();
        save_results();
    }

    /* == CollapseEdgeTest ========================================================================================= */
    class CollapseEdgeTest : public TriangleOperatorsTest {
    public:
        void compute(
            const GEO::index_t f,
            const GEO::index_t lv,
            const double r
            ) {
            GEO::index_t disuse_v, disuse_f0, disuse_f1;

            edge_collapse(
                M_,
                f, lv,
                r,
                disuse_v,
                disuse_f0, disuse_f1);

            M_v_to_delete_[disuse_v] = 1;
            M_f_to_delete_[disuse_f0] = 1;
            EXPECT_NE(disuse_f1, GEO::NO_FACET);
            M_f_to_delete_[disuse_f1] = 1;
            clean_delete_elements();
        }

        void compute_on_border(
            const GEO::index_t f,
            const GEO::index_t lv,
            const double r
            ) {
            GEO::index_t disuse_v, disuse_f0, disuse_f1;

            edge_collapse(
                M_,
                f, lv,
                r,
                disuse_v,
                disuse_f0, disuse_f1);

            M_v_to_delete_[disuse_v] = 1;
            M_f_to_delete_[disuse_f0] = 1;
            EXPECT_EQ(disuse_f1, GEO::NO_FACET);
            clean_delete_elements();
        }
    };

    TEST_F(CollapseEdgeTest, collapse_edge_05) {
        compute(4, 2, 0.5);
        check_connections();
        save_results();
    }

    TEST_F(CollapseEdgeTest, collapse_edge_01) {
        compute(4, 2, 0.1);
        check_connections();
        save_results();
    }

    TEST_F(CollapseEdgeTest, collapse_edge_06_border) {
        compute_on_border(0, 1, 0.6);
        check_connections();
        save_results();
    }

    /* == FlipEdgeTest ============================================================================================= */
    class FlipEdgeTest : public TriangleOperatorsTest {
    public:
        void compute(
            const GEO::index_t f,
            const GEO::index_t lv
            ) {
            edge_swap(M_, f, lv);
        }
    };

    TEST_F(FlipEdgeTest, interior_edge) {
        compute(4, 2);
        check_connections();
        save_results();
    }
}
