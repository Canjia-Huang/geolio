//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/5/2.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//

#include <unordered_set>
#include <gtest/gtest.h>
#include <geogram/mesh/mesh.h>
#include <geolio/hex_operations.h>

namespace geolio::test
{
    class HexOperatorsTest : public ::testing::Test {
    public:
        void SetUp() override {
            M.vertices.create_vertices(8);
            M.vertices.point(0) = GEO::vec3(0,0,0);
            M.vertices.point(1) = GEO::vec3(1,0,0);
            M.vertices.point(2) = GEO::vec3(0,1,0);
            M.vertices.point(3) = GEO::vec3(1,1,0);
            M.vertices.point(4) = GEO::vec3(0,0,1);
            M.vertices.point(5) = GEO::vec3(1,0,1);
            M.vertices.point(6) = GEO::vec3(0,1,1);
            M.vertices.point(7) = GEO::vec3(1,1,1);
            M.cells.create_hex(0,1,2,3,4,5,6,7);
        }

        GEO::Mesh M;
        const GEO::index_t c = 0;
    };

    TEST_F(HexOperatorsTest, find_hex_vertex) {
        for (GEO::index_t lv = 0; lv < M.cells.nb_vertices(c); ++lv)
            EXPECT_EQ(find_hex_vertex(M, c, M.cells.vertex(c, lv)), lv);
    }

    TEST_F(HexOperatorsTest, find_hex_edge_from_local_vertices) {
        for (GEO::index_t le = 0; le < M.cells.nb_edges(c); ++le) {
            const auto& lv0 = HEX_LE_INCIDENT_LV[le][0];
            const auto& lv1 = HEX_LE_INCIDENT_LV[le][1];
            EXPECT_EQ(find_hex_edge_from_local_vertices(lv0, lv1), le);
            EXPECT_EQ(find_hex_edge_from_local_vertices(lv1, lv0), le);
        }
    }

    TEST_F(HexOperatorsTest, find_hex_edge) {
        for (GEO::index_t le = 0; le < M.cells.nb_edges(c); ++le) {
            const auto& ev0 = M.cells.edge_vertex(c, le, 0);
            const auto& ev1 = M.cells.edge_vertex(c, le, 1);
            EXPECT_EQ(find_hex_edge(M, c, ev0, ev1), le);
            EXPECT_EQ(find_hex_edge(M, c, ev1, ev0), le);
        }
    }

    TEST_F(HexOperatorsTest, find_hex_facet_from_local_vertices) {
        for (GEO::index_t i = 0; i < M.cells.nb_vertices(c); ++i) {
            const auto& vi = M.cells.vertex(c, i);
            for (GEO::index_t j = 0; j < M.cells.nb_vertices(c); ++j) {
                const auto& vj = M.cells.vertex(c, j);
                for (GEO::index_t k = 0; k < M.cells.nb_vertices(c); ++k) {
                    const auto& vk = M.cells.vertex(c, k);
                    for (GEO::index_t l = 0; l < M.cells.nb_vertices(c); ++l) {
                        const auto& vl = M.cells.vertex(c, l);

                        for (GEO::index_t lf = 0; lf < M.cells.nb_facets(c); ++lf) {
                            bool found_vi = false;
                            bool found_vj = false;
                            bool found_vk = false;
                            bool found_vl = false;
                            for (GEO::index_t lv = 0; lv < M.cells.facet_nb_vertices(c, lf); ++lv) {
                                if (const auto& v = M.cells.facet_vertex(c, lf, lv);
                                    v == vi) {
                                    EXPECT_FALSE(found_vi);
                                    found_vi = true;
                                    }
                                else if (v == vj) {
                                    EXPECT_FALSE(found_vj);
                                    found_vj = true;
                                }
                                else if (v == vk) {
                                    EXPECT_FALSE(found_vk);
                                    found_vk = true;
                                }
                                else if (v == vl) {
                                    EXPECT_FALSE(found_vl);
                                    found_vl = true;
                                }
                            }

                            if (found_vi && found_vj && found_vk && found_vl)
                                EXPECT_EQ(find_hex_facet_from_local_vertices(i, j, k, l), lf);
                            else
                                EXPECT_NE(find_hex_facet_from_local_vertices(i, j, k, l), lf);

                            if (found_vi && found_vj && found_vk) {
                                EXPECT_EQ(find_hex_facet_from_local_vertices(i, j, k), lf);
                                EXPECT_EQ(find_hex_facet_from_local_vertices(i, k, j), lf);
                                EXPECT_EQ(find_hex_facet_from_local_vertices(j, i, k), lf);
                                EXPECT_EQ(find_hex_facet_from_local_vertices(j, k, i), lf);
                                EXPECT_EQ(find_hex_facet_from_local_vertices(k, i, j), lf);
                                EXPECT_EQ(find_hex_facet_from_local_vertices(k, j, i), lf);
                            }
                            else {
                                EXPECT_NE(find_hex_facet_from_local_vertices(i, j, k), lf);
                                EXPECT_NE(find_hex_facet_from_local_vertices(i, k, j), lf);
                                EXPECT_NE(find_hex_facet_from_local_vertices(j, i, k), lf);
                                EXPECT_NE(find_hex_facet_from_local_vertices(j, k, i), lf);
                                EXPECT_NE(find_hex_facet_from_local_vertices(k, i, j), lf);
                                EXPECT_NE(find_hex_facet_from_local_vertices(k, j, i), lf);
                            }
                        }
                    }
                }
            }
        }
    }

    TEST_F(HexOperatorsTest, find_hex_facet) {
        for (GEO::index_t i = 0; i < M.cells.nb_vertices(c); ++i) {
            const auto& vi = M.cells.vertex(c, i);
            for (GEO::index_t j = 0; j < M.cells.nb_vertices(c); ++j) {
                const auto& vj = M.cells.vertex(c, j);
                for (GEO::index_t k = 0; k < M.cells.nb_vertices(c); ++k) {
                    const auto& vk = M.cells.vertex(c, k);

                    GEO::index_t found_lf = GEO::NO_INDEX;
                    for (GEO::index_t lf = 0; lf < M.cells.nb_facets(c); ++lf) {
                        bool found_vi = false;
                        bool found_vj = false;
                        bool found_vk = false;
                        for (GEO::index_t lv = 0; lv < M.cells.facet_nb_vertices(c, lf); ++lv) {
                            if (const auto& v = M.cells.facet_vertex(c, lf, lv);
                                v == vi) {
                                EXPECT_FALSE(found_vi);
                                found_vi = true;
                            }
                            else if (v == vj) {
                                EXPECT_FALSE(found_vj);
                                found_vj = true;
                            }
                            else if (v == vk) {
                                EXPECT_FALSE(found_vk);
                                found_vk = true;
                            }
                        }

                        if (found_vi && found_vj && found_vk) {
                            EXPECT_EQ(found_lf, GEO::NO_INDEX);
                            found_lf = lf;
                        }
                    }

                    EXPECT_EQ(find_hex_facet(M, c, vi, vj, vk), found_lf);
                }
            }
        }
    }
}