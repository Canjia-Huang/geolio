//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/3/13.
// Copyright (c) 2026 Graphics@XMU. All rights reserved.
//

#include <gtest/gtest.h>
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_io.h>
#include "mesh_opt/triangle_operators.h"
#include "utils.h"

namespace
{
    class TriangleOperatorsTest : public ::testing::Test {
        void SetUp() override {
            ASSERT_TRUE(GEO::mesh_load(std::string(TEST_DATA_PATH) + "case0.obj", M_));
        }
    public:
        void check_connections() {
            std::vector<GEO::index_t> current_connections(3*M_.facets.nb(), GEO::NO_FACET);
            for (const auto& f : M_.facets) {
                for (GEO::index_t lf = 0; lf < 3; ++lf)
                    current_connections[3*f+lf] = M_.facets.adjacent(f, lf);
            }

            M_.facets.connect();
            for (const auto& f : M_.facets) {
                for (GEO::index_t lf = 0; lf < 3; ++lf)
                    EXPECT_EQ(current_connections[3*f+lf], M_.facets.adjacent(f, lf));
            }
        }

        GEO::Mesh M_;
    };

    TEST_F(TriangleOperatorsTest, split_edge_05) {
        const GEO::index_t new_v = M_.vertices.create_vertices(1);
        const GEO::index_t new_f = M_.facets.create_triangles(2);

        MeshOpt::triangle_split_edge(
            M_,
            4, 2,
            new_v, 0.5,
            new_f, new_f+1);

        check_connections();
        EXPECT_TRUE(GEO::mesh_save(M_, get_current_test_name()+".geogram"));
    }

    TEST_F(TriangleOperatorsTest, split_edge_02) {
        const GEO::index_t new_v = M_.vertices.create_vertices(1);
        const GEO::index_t new_f = M_.facets.create_triangles(2);

        MeshOpt::triangle_split_edge(
            M_,
            4, 2,
            new_v, 0.2,
            new_f, new_f+1);

        check_connections();
        EXPECT_TRUE(GEO::mesh_save(M_, get_current_test_name()+".geogram"));
    }

    TEST_F(TriangleOperatorsTest, split_edge_07_border) {
        for (const auto& f : M_.facets) {
            for (GEO::index_t lv = 0; lv < 3; ++lv) {
                if (M_.facets.adjacent(f, lv) == GEO::NO_FACET) {
                    const GEO::index_t new_v = M_.vertices.create_vertices(1);
                    const GEO::index_t new_f = M_.facets.create_triangles(1);

                    MeshOpt::triangle_split_edge(
                        M_,
                        f, lv,
                        new_v, 0.7,
                        new_f, GEO::NO_FACET);

                    check_connections();
                    EXPECT_TRUE(GEO::mesh_save(M_, get_current_test_name()+".geogram"));

                    return;
                }
            }
        }
    }
}