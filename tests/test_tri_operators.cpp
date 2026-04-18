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

namespace GEO::MeshUtils::Test
{
    using namespace GEO::MeshUtils;

    class TriangleOperatorsTest : public ::testing::TestWithParam<std::pair<GEO::index_t, GEO::index_t>> {
    protected:
        void SetUp() override {
            /* Build mesh */
            GEO::index_t new_v = M_.vertices.create_vertices(16);
            for (GEO::index_t i = 0; i < 4; ++i) {
                for (GEO::index_t j = 0; j < 4; ++j)
                    M_.vertices.point(new_v++) = GEO::vec3(j, i, 0);
            }
            ASSERT_EQ(M_.vertices.nb(), 16);
            M_.vertices.point(5).z += 0.5;
            M_.vertices.point(10).z += 0.5;

            GEO::index_t new_f = M_.facets.create_triangles(18);
            for (GEO::index_t i = 0; i < 3; ++i) {
                for (GEO::index_t j = 0; j < 3; ++j) {
                    const GEO::index_t v = 4*i+j;
                    M_.facets.set_vertex(new_f, 0, v);
                    M_.facets.set_vertex(new_f, 1, v+1);
                    M_.facets.set_vertex(new_f, 2, v+4);
                    M_.facets.set_vertex(new_f+1, 0, v+5);
                    M_.facets.set_vertex(new_f+1, 1, v+4);
                    M_.facets.set_vertex(new_f+1, 2, v+1);
                    new_f += 2;
                }
            }
            ASSERT_EQ(new_f, 18);

            GEO::Attribute<GEO::index_t> M_v_idx(M_.vertices.attributes(), "idx");
            for (const auto& v : M_.vertices)
                M_v_idx[v] = v;

            GEO::Attribute<GEO::index_t> M_f_idx(M_.facets.attributes(), "idx");
            for (const auto& f : M_.facets)
                M_f_idx[f] = f;

            M_.facets.connect();

            LOG::DEBUG("M #V:{}, #F: {}", M_.vertices.nb(), M_.facets.nb());

            /* Find all interior and border f_lv pair */
            std::vector<bool> M_v_border(M_.vertices.nb(), false);
            for (const auto& f : M_.facets) {
                for (GEO::index_t lv = 0; lv < 3; ++lv) {
                    if (M_.facets.adjacent(f, lv) == GEO::NO_FACET) {
                        border_edges_f_lv.emplace_back(f, lv);
                        M_v_border[M_.facets.vertex(f, lv)] = true;
                        M_v_border[M_.facets.vertex(f, (lv+1)%3)] = true;
                    }
                    else
                        interior_edges_f_lv.emplace_back(f, lv);
                }
            }
            for (const auto& f : M_.facets) {
                for (GEO::index_t lv = 0; lv < 3; ++lv) {
                    if (M_v_border[M_.facets.vertex(f, lv)])
                        border_vertices_f_lv.emplace_back(f, lv);
                    else
                        interior_vertices_f_lv.emplace_back(f, lv);
                }
            }
        }

        GEO::Mesh M_;
        std::vector<std::pair<GEO::index_t, GEO::index_t>> interior_vertices_f_lv;
        std::vector<std::pair<GEO::index_t, GEO::index_t>> border_vertices_f_lv;
        std::vector<std::pair<GEO::index_t, GEO::index_t>> interior_edges_f_lv;
        std::vector<std::pair<GEO::index_t, GEO::index_t>> border_edges_f_lv;

    public:
        static void clean_delete_elements(
            GEO::Mesh& M,
            GEO::vector<GEO::index_t>& facets_to_delete
            ) {
            ASSERT_EQ(facets_to_delete.size(), M.facets.nb());
            M.facets.delete_elements(facets_to_delete);
        }

        static void check_connections(
            GEO::Mesh& M
            ) {
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

        static void save_results(
            const GEO::Mesh& M,
            const GEO::index_t f,
            const GEO::index_t lv) {
            EXPECT_TRUE(GEO::mesh_save(M, get_current_test_name()
                                        + "_" + std::to_string(f)
                                        + "_" + std::to_string(lv)
                                        + ".geogram"));
        }
    };

    /* == GetIncidentTrianglesTest ================================================================================= */
    class GetIncidentTrianglesTest : public TriangleOperatorsTest {
    public:
        bool compute(
            const GEO::Mesh& M_out,
            const GEO::index_t start_f,
            const GEO::index_t start_lv
            ) {
            start_f_ = start_f;
            start_lv_ = start_lv;
            return get_vertex_incident_triangles(M_out, start_f_, start_lv_, ordered_f_and_lv);
        }

        void check_incident(
            const GEO::Mesh& M_out
            ) {
            const GEO::index_t v = M_out.facets.vertex(start_f_, start_lv_);
            for (const auto& [f, lv] : ordered_f_and_lv)
                EXPECT_EQ(M_out.facets.vertex(f, lv), v);
        }

        void check_loop(
            const GEO::Mesh& M_out
            ) {
            for (GEO::index_t i = 0, i_end = ordered_f_and_lv.size(); i < i_end; ++i) {
                const auto& [f, lv] =  ordered_f_and_lv[i];
                EXPECT_EQ(M_out.facets.adjacent(f, lv), ordered_f_and_lv[(i+1)%i_end].first);
            }
        }

        void check_loop_on_border(
            const GEO::Mesh& M_out
            ) {
            for (GEO::index_t i = 0, i_end = ordered_f_and_lv.size(); i < i_end; ++i) {
                const auto& [f, lv] =  ordered_f_and_lv[i];
                if (i == 0)
                    EXPECT_EQ(M_out.facets.adjacent(f, (lv+2)%3), GEO::NO_FACET);
                else if (i == i_end-1)
                    EXPECT_EQ(M_out.facets.adjacent(f, lv), GEO::NO_FACET);
                else
                    EXPECT_EQ(M_out.facets.adjacent(f, lv), ordered_f_and_lv[i+1].first);
            }
        }


        std::vector<std::pair<GEO::index_t, GEO::index_t>> ordered_f_and_lv;
        GEO::index_t start_f_{};
        GEO::index_t start_lv_{};
    };

    TEST_F(GetIncidentTrianglesTest, interior_vertex) {
        for (const auto& [f, lv] : interior_vertices_f_lv) {
            GEO::Mesh M_out;
            M_out.copy(M_);

            EXPECT_FALSE(compute(M_out, f, lv));
            check_incident(M_out);
            check_loop(M_out);
        }
    }

    TEST_F(GetIncidentTrianglesTest, border_vertex) {
        for (const auto& [f, lv] : border_vertices_f_lv) {
            GEO::Mesh M_out;
            M_out.copy(M_);

            EXPECT_TRUE(compute(M_out, f, lv));
            check_incident(M_out);
            check_loop_on_border(M_out);
        }
    }

    /* == SplitEdgeTest ============================================================================================ */
    class SplitEdgeTest : public TriangleOperatorsTest {
    public:
        static void compute(
            GEO::Mesh& M_out,
            const GEO::index_t f,
            const GEO::index_t lv,
            const double r
            ) {
            ASSERT_NE(M_out.facets.adjacent(f, lv), GEO::NO_FACET);
            const GEO::index_t new_v = M_out.vertices.create_vertices(1);
            const GEO::index_t new_f = M_out.facets.create_triangles(2);

            edge_split(
                M_out,
                f, lv,
                r,
                new_v,
                new_f, new_f+1);
        }

        static void compute_on_border(
            GEO::Mesh& M_out,
            const GEO::index_t f,
            const GEO::index_t lv,
            const double r
            ) {
            ASSERT_EQ(M_out.facets.adjacent(f, lv), GEO::NO_FACET);
            const GEO::index_t new_v = M_out.vertices.create_vertices(1);
            const GEO::index_t new_f = M_out.facets.create_triangles(1);

            edge_split(
                M_out,
                f, lv,
                r,
                new_v,
                new_f, GEO::NO_FACET);
        }
    };

    TEST_F(SplitEdgeTest, interior_edges) {
        for (const auto& [f, lv] : interior_edges_f_lv) {
            GEO::Mesh M_out;
            M_out.copy(M_);

            compute(M_out, f, lv, GEO::Numeric::random_float32());
            check_connections(M_out);
            save_results(M_out, f, lv);
        }
    }

    TEST_F(SplitEdgeTest, border_edges) {
        for (const auto& [f, lv] : border_edges_f_lv) {
            GEO::Mesh M_out;
            M_out.copy(M_);

            compute_on_border(M_out, f, lv, GEO::Numeric::random_float32());
            check_connections(M_out);
            save_results(M_out, f, lv);
        }
    }

    /* == CollapseEdgeTest ========================================================================================= */
    class CollapseEdgeTest : public TriangleOperatorsTest {
    public:
        static void compute(
            GEO::Mesh& M_out,
            const GEO::index_t f,
            const GEO::index_t lv,
            const double r
            ) {
            GEO::index_t disuse_v, disuse_f0, disuse_f1;

            edge_collapse(
                M_out,
                f, lv,
                r,
                disuse_v,
                disuse_f0, disuse_f1);

            /* Clean disuse vertices and facets */
            GEO::vector<GEO::index_t> facets_to_delete(M_out.facets.nb(), 0);
            facets_to_delete[disuse_f0] = 1;
            EXPECT_NE(disuse_f1, GEO::NO_FACET);
            facets_to_delete[disuse_f1] = 1;
            M_out.facets.delete_elements(facets_to_delete);
        }

        static void compute_on_border(
            GEO::Mesh& M_out,
            const GEO::index_t f,
            const GEO::index_t lv,
            const double r
            ) {
            GEO::index_t disuse_v, disuse_f0, disuse_f1;

            edge_collapse(
                M_out,
                f, lv,
                r,
                disuse_v,
                disuse_f0, disuse_f1);

            /* Clean disuse vertices and facets */
            GEO::vector<GEO::index_t> facets_to_delete(M_out.facets.nb(), 0);
            facets_to_delete[disuse_f0] = 1;
            M_out.facets.delete_elements(facets_to_delete);
        }
    };

    TEST_F(CollapseEdgeTest, interior_edges) {
        for (const auto& [f, lv] : interior_edges_f_lv) {
            GEO::Mesh M_out;
            M_out.copy(M_);

            compute(M_out, f, lv, GEO::Numeric::random_float32());
            check_connections(M_out);
            save_results(M_out, f, lv);
        }
    }

    TEST_F(CollapseEdgeTest, border_edges) {
        for (const auto& [f, lv] : border_edges_f_lv) {
            GEO::Mesh M_out;
            M_out.copy(M_);

            compute_on_border(M_out, f, lv, GEO::Numeric::random_float32());
            check_connections(M_out);
            save_results(M_out, f, lv);
        }
    }

    /* == FlipEdgeTest ============================================================================================= */
    class FlipEdgeTest : public TriangleOperatorsTest {
    public:
        static void compute(
            GEO::Mesh& M_out,
            const GEO::index_t f,
            const GEO::index_t lv
            ) {
            edge_swap(M_out, f, lv);
        }
    };

    TEST_F(FlipEdgeTest, interior_edges) {
        for (const auto& [f, lv] : interior_edges_f_lv) {
            GEO::Mesh M_out;
            M_out.copy(M_);

            compute(M_out, f, lv);
            check_connections(M_out);
            save_results(M_out, f, lv);
        }
    }
}
