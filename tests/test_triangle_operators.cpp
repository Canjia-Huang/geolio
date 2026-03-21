//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/3/13.
// Copyright (c) 2026 Graphics@XMU. All rights reserved.
//

#include <gtest/gtest.h>
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_io.h>
#include "mesh_opt/triangle_operators.h"
#include "utils.h"
#include "common/log.h"

namespace
{
    class TriangleOperatorsTest : public ::testing::Test {
        void SetUp() override {
            ASSERT_TRUE(GEO::mesh_load(std::string(TEST_DATA_PATH) + "case0.obj", M_));

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

        GEO::Mesh M_;
        GEO::Attribute<GEO::index_t> M_v_to_delete_;
        GEO::Attribute<GEO::index_t> M_f_to_delete_;
    };

    TEST_F(TriangleOperatorsTest, get_incident_triangles) {
        constexpr GEO::index_t start_f = 4;
        constexpr GEO::index_t start_lv = 2;

        std::vector<std::pair<GEO::index_t, GEO::index_t>> ordered_f_and_lv;
        MeshOpt::get_vertex_one_ring_triangles(M_, start_f, start_lv, ordered_f_and_lv);

        /* Check incident */
        const GEO::index_t v = M_.facets.vertex(start_f, start_lv);
        for (const auto& [f, lv] : ordered_f_and_lv)
            EXPECT_EQ(M_.facets.vertex(f, lv), v);

        /* Form a loop */
        for (GEO::index_t i = 0, i_end = ordered_f_and_lv.size(); i < i_end; ++i) {
            const auto& [f, lv] =  ordered_f_and_lv[i];
            EXPECT_EQ(M_.facets.adjacent(f, lv), ordered_f_and_lv[(i+1)%i_end].first);
        }
    }

    TEST_F(TriangleOperatorsTest, get_incident_triangles_border) {
        constexpr GEO::index_t start_f = 0;
        constexpr GEO::index_t start_lv = 1;

        std::vector<std::pair<GEO::index_t, GEO::index_t>> ordered_f_and_lv;
        MeshOpt::get_vertex_one_ring_triangles(M_, start_f, start_lv, ordered_f_and_lv);

        /* Check incident */
        const GEO::index_t v = M_.facets.vertex(start_f, start_lv);
        for (const auto& [f, lv] : ordered_f_and_lv)
            EXPECT_EQ(M_.facets.vertex(f, lv), v);

        /* Form a loop */
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

    TEST_F(TriangleOperatorsTest, split_edge_05) {
        const GEO::index_t new_v = M_.vertices.create_vertices(1);
        const GEO::index_t new_f = M_.facets.create_triangles(2);

        MeshOpt::split_triangle_edge(
            M_,
            4, 2,
            0.5,
            new_v,
            new_f, new_f+1);

        check_connections();
        EXPECT_TRUE(GEO::mesh_save(M_, get_current_test_name()+".geogram"));
    }

    TEST_F(TriangleOperatorsTest, split_edge_02) {
        const GEO::index_t new_v = M_.vertices.create_vertices(1);
        const GEO::index_t new_f = M_.facets.create_triangles(2);

        MeshOpt::split_triangle_edge(
            M_,
            4, 2,
            0.2,
            new_v,
            new_f, new_f+1);

        check_connections();
        EXPECT_TRUE(GEO::mesh_save(M_, get_current_test_name()+".geogram"));
    }

    TEST_F(TriangleOperatorsTest, split_edge_07_border) {
        const GEO::index_t new_v = M_.vertices.create_vertices(1);
        const GEO::index_t new_f = M_.facets.create_triangles(2);

        MeshOpt::split_triangle_edge(
            M_,
            0, 1,
            0.7,
            new_v,
            new_f, new_f+1);

        check_connections();
        EXPECT_TRUE(GEO::mesh_save(M_, get_current_test_name()+".geogram"));
    }

    TEST_F(TriangleOperatorsTest, collapse_edge_05) {
        GEO::index_t disuse_v, disuse_f0, disuse_f1;

        MeshOpt::collapse_triangle_edge(
            M_,
            4, 2,
            0.5,
            disuse_v,
            disuse_f0, disuse_f1);

        M_v_to_delete_[disuse_v] = 1;
        M_f_to_delete_[disuse_f0] = 1;
        EXPECT_NE(disuse_f1, GEO::NO_FACET);
        M_f_to_delete_[disuse_f1] = 1;
        clean_delete_elements();
        check_connections();
        EXPECT_TRUE(GEO::mesh_save(M_, get_current_test_name()+".geogram"));
    }

    TEST_F(TriangleOperatorsTest, collapse_edge_01) {
        GEO::index_t disuse_v, disuse_f0, disuse_f1;

        MeshOpt::collapse_triangle_edge(
            M_,
            4, 2,
            0.1,
            disuse_v,
            disuse_f0, disuse_f1);

        M_v_to_delete_[disuse_v] = 1;
        M_f_to_delete_[disuse_f0] = 1;
        EXPECT_NE(disuse_f1, GEO::NO_FACET);
        M_f_to_delete_[disuse_f1] = 1;
        clean_delete_elements();
        check_connections();
        EXPECT_TRUE(GEO::mesh_save(M_, get_current_test_name()+".geogram"));
    }

    TEST_F(TriangleOperatorsTest, collapse_edge_06_border) {
        GEO::index_t disuse_v, disuse_f0, disuse_f1;

        MeshOpt::collapse_triangle_edge(
            M_,
            0, 1,
            0.6,
            disuse_v,
            disuse_f0, disuse_f1);

        M_v_to_delete_[disuse_v] = 1;
        M_f_to_delete_[disuse_f0] = 1;
        EXPECT_NE(disuse_f1, GEO::NO_FACET);
        M_f_to_delete_[disuse_f1] = 1;
        clean_delete_elements();
        check_connections();
        EXPECT_TRUE(GEO::mesh_save(M_, get_current_test_name()+".geogram"));
    }
}
