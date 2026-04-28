//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/4/5.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//

#include <gtest/gtest.h>
#include "geogram_mesh_utils/hex_descriptor.h"
#include <geogram/mesh/mesh.h>
#include <unordered_set>

namespace GEO::MeshUtils::Test
{
    using namespace GEO::MeshUtils;

    class HexDescriptorTest : public ::testing::Test {
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

    TEST_F(HexDescriptorTest, HEX_LV_ADJACENT_LV) {
        ASSERT_EQ(M.cells.nb_vertices(c), HEX_LV_ADJACENT_LV.size());

        for (GEO::index_t lv = 0; lv < M.cells.nb_vertices(c); ++lv) {
            ASSERT_EQ(HEX_LV_ADJACENT_LV[lv].size(), 3);

            {
                const auto v = M.cells.vertex(c, lv);

                std::unordered_set<GEO::index_t> adj_vertices;
                for (GEO::index_t le = 0; le < M.cells.nb_edges(c); ++le) {
                    const auto& ev0 = M.cells.edge_vertex(c, le, 0);
                    const auto& ev1 = M.cells.edge_vertex(c, le, 1);
                    if (ev0 == v)
                        adj_vertices.insert(ev1);
                    if (ev1 == v)
                        adj_vertices.insert(ev0);
                }

                EXPECT_EQ(adj_vertices.size(), HEX_LV_ADJACENT_LV[lv].size());
                for (const auto& adj_v : HEX_LV_ADJACENT_LV[lv])
                    EXPECT_TRUE(adj_vertices.contains(adj_v));
            }

            { // orientation
                const auto& p = M.cells.point(c, lv);

                const auto& lv0 = HEX_LV_ADJACENT_LV[lv][0];
                const auto& lv1 = HEX_LV_ADJACENT_LV[lv][1];
                const auto& lv2 = HEX_LV_ADJACENT_LV[lv][2];
                const auto& p0 = M.cells.point(c, lv0);
                const auto& p1 = M.cells.point(c, lv1);
                const auto& p2 = M.cells.point(c, lv2);
                const auto pp0 = p0 - p;
                const auto pp1 = p1 - p;
                const auto pp2 = p2 - p;

                EXPECT_NEAR(GEO::Geom::cos_angle(GEO::cross(pp0, pp1), pp2), 1, 1e-10);
                EXPECT_NEAR(GEO::Geom::cos_angle(GEO::cross(pp1, pp2), pp0), 1, 1e-10);
                EXPECT_NEAR(GEO::Geom::cos_angle(GEO::cross(pp2, pp0), pp1), 1, 1e-10);
            }
        }
    }

    TEST_F(HexDescriptorTest, HEX_LV_INCIDENT_LE) {
        ASSERT_EQ(M.cells.nb_vertices(c), HEX_LV_INCIDENT_LE.size());

        for (GEO::index_t lv = 0; lv < M.cells.nb_vertices(c); ++lv) {
            ASSERT_EQ(HEX_LV_ADJACENT_LV[lv].size(), 3);

            {
                const auto v = M.cells.vertex(c, lv);

                std::unordered_set<GEO::index_t> adj_edges;
                for (GEO::index_t le = 0; le < M.cells.nb_edges(c); ++le) {
                    const auto& ev0 = M.cells.edge_vertex(c, le, 0);
                    const auto& ev1 = M.cells.edge_vertex(c, le, 1);
                    if (ev0 == v || ev1 == v)
                        adj_edges.insert(le);
                }

                EXPECT_EQ(adj_edges.size(), HEX_LV_INCIDENT_LE[lv].size());
                for (const auto& adj_e : HEX_LV_INCIDENT_LE[lv])
                    EXPECT_TRUE(adj_edges.contains(adj_e));
            }

            { // orientation
                const auto& v = M.cells.vertex(c, lv);
                const auto& p = M.cells.point(c, lv);


                const auto& le0 = HEX_LV_INCIDENT_LE[lv][0];
                const auto& le1 = HEX_LV_INCIDENT_LE[lv][1];
                const auto& le2 = HEX_LV_INCIDENT_LE[lv][2];
                auto v0 = M.cells.edge_vertex(c, le0, 0);
                if (v0 == v)
                    v0 = M.cells.edge_vertex(c, le0, 1);
                auto v1 = M.cells.edge_vertex(c, le1, 0);
                if (v1 == v)
                    v1 = M.cells.edge_vertex(c, le1, 1);
                auto v2 = M.cells.edge_vertex(c, le2, 0);
                if (v2 == v)
                    v2 = M.cells.edge_vertex(c, le2, 1);
                const auto& p0 = M.vertices.point(v0);
                const auto& p1 = M.vertices.point(v1);
                const auto& p2 = M.vertices.point(v2);

                const auto pp0 = p0 - p;
                const auto pp1 = p1 - p;
                const auto pp2 = p2 - p;

                EXPECT_NEAR(GEO::Geom::cos_angle(GEO::cross(pp0, pp1), pp2), 1, 1e-10);
                EXPECT_NEAR(GEO::Geom::cos_angle(GEO::cross(pp1, pp2), pp0), 1, 1e-10);
                EXPECT_NEAR(GEO::Geom::cos_angle(GEO::cross(pp2, pp0), pp1), 1, 1e-10);
            }
        }
    }

    TEST_F(HexDescriptorTest, HEX_LV_INCIDENT_LF) {
        ASSERT_EQ(M.cells.nb_vertices(c), HEX_LV_INCIDENT_LF.size());

        for (GEO::index_t lv = 0; lv < M.cells.nb_vertices(c); ++lv) {
            ASSERT_EQ(HEX_LV_INCIDENT_LF[lv].size(), 3);

            {
                const auto v = M.cells.vertex(c, lv);

                std::unordered_set<GEO::index_t> adj_facets;
                for (GEO::index_t lf = 0; lf < M.cells.nb_facets(c); ++lf) {
                    for (GEO::index_t lfv = 0; lfv < M.cells.facet_nb_vertices(c, lf); ++lfv) {
                        if (M.cells.facet_vertex(c, lf, lfv) == v) {
                            adj_facets.insert(lf);
                            break;
                        }
                    }
                }

                EXPECT_EQ(adj_facets.size(), HEX_LV_INCIDENT_LF[lv].size());
                for (const auto& adj_f : HEX_LV_INCIDENT_LF[lv])
                    EXPECT_TRUE(adj_facets.contains(adj_f));
            }

            { // orientation
                const auto& lf0 = HEX_LV_INCIDENT_LF[lv][0];
                const auto& lf1 = HEX_LV_INCIDENT_LF[lv][1];
                const auto& lf2 = HEX_LV_INCIDENT_LF[lv][2];
                ASSERT_LE(lf0, M.cells.nb_facets(c));
                ASSERT_LE(lf1, M.cells.nb_facets(c));
                ASSERT_LE(lf2, M.cells.nb_facets(c));

                const auto n0 = -GEO::normalize(GEO::Geom::triangle_normal(
                    M.vertices.point(M.cells.facet_vertex(c, lf0, 0)),
                    M.vertices.point(M.cells.facet_vertex(c, lf0, 1)),
                    M.vertices.point(M.cells.facet_vertex(c, lf0, 2))));
                const auto n1 = -GEO::normalize(GEO::Geom::triangle_normal(
                    M.vertices.point(M.cells.facet_vertex(c, lf1, 0)),
                    M.vertices.point(M.cells.facet_vertex(c, lf1, 1)),
                    M.vertices.point(M.cells.facet_vertex(c, lf1, 2))));
                const auto n2 = -GEO::normalize(GEO::Geom::triangle_normal(
                    M.vertices.point(M.cells.facet_vertex(c, lf2, 0)),
                    M.vertices.point(M.cells.facet_vertex(c, lf2, 1)),
                    M.vertices.point(M.cells.facet_vertex(c, lf2, 2))));
                EXPECT_NEAR(GEO::Geom::cos_angle(GEO::cross(n0, n1), n2), -1, 1e-10);
                EXPECT_NEAR(GEO::Geom::cos_angle(GEO::cross(n1, n2), n0), -1, 1e-10);
                EXPECT_NEAR(GEO::Geom::cos_angle(GEO::cross(n2, n0), n1), -1, 1e-10);
            }
        }
    }

    TEST_F(HexDescriptorTest, HEX_LE_INCIDENT_LV) {
        ASSERT_EQ(M.cells.nb_edges(c), HEX_LE_INCIDENT_LV.size());

        for (GEO::index_t le = 0; le < M.cells.nb_edges(c); ++le) {
            std::unordered_set<GEO::index_t> adj_vertices;

            for (GEO::index_t i = 0; i < 2; ++i)
                adj_vertices.insert(M.cells.edge_vertex(c, le, i));

            EXPECT_EQ(adj_vertices.size(), HEX_LE_INCIDENT_LV[le].size());
            for (const auto& adj_v : HEX_LE_INCIDENT_LV[le])
                EXPECT_TRUE(adj_vertices.contains(adj_v));
        }
    }

    TEST_F(HexDescriptorTest, HEX_LE_INCIDENT_LF) {
        ASSERT_EQ(M.cells.nb_edges(c), HEX_LE_INCIDENT_LF.size());

        for (GEO::index_t le = 0; le < M.cells.nb_edges(c); ++le) {
            {
                for (GEO::index_t i = 0; i < 2; ++i)
                    EXPECT_EQ(M.cells.edge_adjacent_facet(c, le, i), HEX_LE_INCIDENT_LF[le][i]);
            }

            { // orientation
                const auto& p0 = M.vertices.point(M.cells.edge_vertex(c, le, 0));
                const auto& p1 = M.vertices.point(M.cells.edge_vertex(c, le, 1));

                ASSERT_EQ(HEX_LE_INCIDENT_LF[le].size(), 2);
                const auto& lf0 = HEX_LE_INCIDENT_LF[le][0];
                const auto& lf1 = HEX_LE_INCIDENT_LF[le][1];
                const auto n0 = -GEO::normalize(GEO::Geom::triangle_normal(
                    M.vertices.point(M.cells.facet_vertex(c, lf0, 0)),
                    M.vertices.point(M.cells.facet_vertex(c, lf0, 1)),
                    M.vertices.point(M.cells.facet_vertex(c, lf0, 2))));
                const auto n1 = -GEO::normalize(GEO::Geom::triangle_normal(
                    M.vertices.point(M.cells.facet_vertex(c, lf1, 0)),
                    M.vertices.point(M.cells.facet_vertex(c, lf1, 1)),
                    M.vertices.point(M.cells.facet_vertex(c, lf1, 2))));

                EXPECT_NEAR(GEO::Geom::cos_angle(GEO::cross(n0, n1), p1-p0), -1, 1e-10);
            }
        }
    }

    TEST_F(HexDescriptorTest, HEX_LE_LOOP_LE) {
        ASSERT_EQ(M.cells.nb_edges(c), HEX_LE_LOOP_LE.size());

        for (GEO::index_t le = 0; le < M.cells.nb_edges(c); ++le) {
            ASSERT_EQ(HEX_LE_LOOP_LE[le].size(), 4);

            {
                std::unordered_set<GEO::index_t> edges;
                for (const auto& e : HEX_LE_LOOP_LE[le])
                    edges.insert(e);
                EXPECT_EQ(edges.size(), 4);

                for (GEO::index_t i = 0; i < 4; ++i) {
                    const auto& le0 = HEX_LE_LOOP_LE[le][i];
                    const auto& le1 = HEX_LE_LOOP_LE[le][(i+1)%4];
                    const auto& lf0 = HEX_LE_INCIDENT_LF[le0][0];
                    const auto& lf1 = HEX_LE_INCIDENT_LF[le0][1];
                    const auto& lf2 = HEX_LE_INCIDENT_LF[le1][0];
                    const auto& lf3 = HEX_LE_INCIDENT_LF[le1][1];
                    EXPECT_TRUE(lf0 == lf2 || lf0 == lf3 || lf1 == lf2 || lf1 == lf3);
                }
            }

            { // orientation
                for (GEO::index_t i = 0; i < 4; ++i) {
                    const auto& lf0 = HEX_LE_LOOP_LF[le][i];
                    const auto& lf1 = HEX_LE_LOOP_LF[le][(i+1)%4];
                    EXPECT_EQ(HEX_LF_LF_COMMON_LE[lf0][lf1], HEX_LE_LOOP_LE[le][i]);
                }
            }
        }
    }

    TEST_F(HexDescriptorTest, HEX_LE_LOOP_LF) {
        ASSERT_EQ(M.cells.nb_edges(c), HEX_LE_LOOP_LF.size());

        for (GEO::index_t le = 0; le < M.cells.nb_edges(c); ++le) {
            ASSERT_EQ(HEX_LE_LOOP_LF[le].size(), 4);
            {
                std::unordered_set<GEO::index_t> facets;
                for (const auto lf : HEX_LE_LOOP_LF[le])
                    facets.insert(lf);
                EXPECT_EQ(facets.size(), 4);

                const GEO::index_t lf0 = HEX_LE_INCIDENT_LF[le][0];
                const GEO::index_t lf1 = HEX_LE_INCIDENT_LF[le][1];

                GEO::index_t nlf0 = GEO::NO_INDEX;
                for (const auto lf : HEX_LV_INCIDENT_LF[M.cells.edge_vertex(c, le, 0)]) {
                    if (lf != lf0 && lf != lf1) {
                        EXPECT_TRUE(nlf0 == GEO::NO_INDEX);
                        nlf0 = lf;
                    }
                }
                EXPECT_FALSE(nlf0 == GEO::NO_INDEX);

                GEO::index_t nlf1 = GEO::NO_INDEX;
                for (const auto lf : HEX_LV_INCIDENT_LF[M.cells.edge_vertex(c, le, 1)]) {
                    if (lf != lf0 && lf != lf1) {
                        EXPECT_TRUE(nlf1 == GEO::NO_INDEX);
                        nlf1 = lf;
                    }
                }
                EXPECT_FALSE(nlf1 == GEO::NO_INDEX);

                EXPECT_FALSE(facets.contains(nlf0));
                EXPECT_FALSE(facets.contains(nlf1));
            }

            { // orientation
                const GEO::vec3 en = M.vertices.point(M.cells.edge_vertex(c, le, 1)) - M.vertices.point(M.cells.edge_vertex(c, le, 0));
                for (GEO::index_t i = 0; i < 4; ++i) {
                    const auto& lf0 = HEX_LE_LOOP_LF[le][i];
                    const auto& lf1 = HEX_LE_LOOP_LF[le][(i+1)%4];
                    EXPECT_NE(HEX_LF_LF_COMMON_LE[lf0][lf1], GEO::NO_INDEX);

                    const auto n0 = GEO::Geom::triangle_normal(
                        M.vertices.point(M.cells.facet_vertex(c, lf0, 0)),
                        M.vertices.point(M.cells.facet_vertex(c, lf0, 1)),
                        M.vertices.point(M.cells.facet_vertex(c, lf0, 2)));
                    const auto n1 = GEO::Geom::triangle_normal(
                        M.vertices.point(M.cells.facet_vertex(c, lf1, 0)),
                        M.vertices.point(M.cells.facet_vertex(c, lf1, 1)),
                        M.vertices.point(M.cells.facet_vertex(c, lf1, 2)));
                    EXPECT_NEAR(GEO::dot(GEO::cross(n0, n1), -en), 1, 1e-10);
                }
            }
        }
    }

    TEST_F(HexDescriptorTest, HEX_ENCODED_LE) {
        ASSERT_EQ(M.cells.nb_edges(c), HEX_ENCODED_LE.size());

        for (GEO::index_t le = 0; le < M.cells.nb_edges(c); ++le) {
            const auto& ev0 = M.cells.edge_vertex(c, le, 0);
            const auto& ev1 = M.cells.edge_vertex(c, le, 1);
            EXPECT_EQ((1<<ev0) | (1<<ev1), HEX_ENCODED_LE[le]);
        }
    }

    TEST_F(HexDescriptorTest, HEX_LF_INCIDENT_LV) {
        ASSERT_EQ(M.cells.nb_facets(0), HEX_LF_INCIDENT_LV.size());

        for (GEO::index_t lf = 0; lf < M.cells.nb_facets(c); ++lf) {
            ASSERT_EQ(M.cells.facet_nb_vertices(c, lf), HEX_LF_INCIDENT_LV[lf].size());
            for (GEO::index_t lv = 0; lv < M.cells.facet_nb_vertices(c, lf); ++lv)
                EXPECT_EQ(M.cells.facet_vertex(c, lf, lv), HEX_LF_INCIDENT_LV[lf][lv]);
        }
    }

    TEST_F(HexDescriptorTest, HEX_LF_INCIDENT_LE) {
        ASSERT_EQ(M.cells.nb_facets(c), HEX_LF_INCIDENT_LE.size());

        for (GEO::index_t lf = 0; lf < M.cells.nb_facets(c); ++lf) {
            ASSERT_EQ(HEX_LF_INCIDENT_LE[lf].size(), 4);

            {
                std::unordered_set<GEO::index_t> facet_vertices;
                for (GEO::index_t lv = 0; lv < M.cells.facet_nb_vertices(c, lf); ++lv)
                    facet_vertices.insert(M.cells.facet_vertex(c, lf, lv));

                std::unordered_set<GEO::index_t> adj_edges;
                for (GEO::index_t le = 0; le < M.cells.nb_edges(c); ++le) {
                    const auto& ev0 = M.cells.edge_vertex(c, le, 0);
                    const auto& ev1 = M.cells.edge_vertex(c, le, 1);
                    if (facet_vertices.contains(ev0) && facet_vertices.contains(ev1))
                        adj_edges.insert(le);
                }

                EXPECT_EQ(adj_edges.size(), HEX_LF_INCIDENT_LE[lf].size());
                for (const auto& adj_e : HEX_LF_INCIDENT_LE[lf])
                    EXPECT_TRUE(adj_edges.contains(adj_e));
            }

            { // orientation
                for (GEO::index_t i = 0; i < 4; ++i) {
                    const auto& le0 = HEX_LF_INCIDENT_LE[lf][i];
                    const auto& le1 = HEX_LF_INCIDENT_LE[lf][(i+1)%4];

                    /* Check le0 and le1 are connected */
                    const auto& ev00 = M.cells.edge_vertex(c, le0, 0);
                    const auto& ev01 = M.cells.edge_vertex(c, le0, 1);
                    const auto& ev10 = M.cells.edge_vertex(c, le1, 0);
                    const auto& ev11 = M.cells.edge_vertex(c, le1, 1);
                    EXPECT_TRUE(ev00 == ev10 || ev00 == ev11 ||
                                ev01 == ev10 || ev01 == ev11);
                }
            }
        }
    }

    TEST_F(HexDescriptorTest, HEX_LF_OPPOSITE_LF) {
        ASSERT_EQ(M.cells.nb_facets(c), HEX_LF_OPPOSITE_LF.size());

        for (GEO::index_t lf = 0; lf < M.cells.nb_facets(c); ++lf) {
            std::unordered_set<GEO::index_t> facet_vertices;
            for (GEO::index_t lv = 0; lv < M.cells.facet_nb_vertices(c, lf); ++lv)
                facet_vertices.insert(M.cells.facet_vertex(c, lf, lv));

            for (GEO::index_t nlf = 0; nlf < M.cells.nb_facets(c); ++nlf) {
                bool opposite = true;
                for (GEO::index_t nlv = 0; nlv < M.cells.facet_nb_vertices(c, nlf); ++nlv) {
                    if (facet_vertices.contains(M.cells.facet_vertex(c, nlf, nlv))) {
                        opposite = false;
                        break;
                    }
                }
                if (opposite)
                    EXPECT_EQ(nlf, HEX_LF_OPPOSITE_LF[lf]);
            }
        }
    }

    TEST_F(HexDescriptorTest, HEX_LF_LF_COMMON_LE) {
        EXPECT_EQ(M.cells.nb_facets(c), HEX_LF_LF_COMMON_LE.size());

        for (GEO::index_t lf0 = 0; lf0 < M.cells.nb_facets(c); ++lf0) {
            EXPECT_EQ(M.cells.nb_facets(c), HEX_LF_LF_COMMON_LE[lf0].size());

            for (GEO::index_t lf1 = 0; lf1 < M.cells.nb_facets(c); ++lf1) {
                if (lf0 == lf1)
                    EXPECT_EQ(GEO::NO_INDEX, HEX_LF_LF_COMMON_LE[lf0][lf1]);
                else {
                    GEO::index_t common_le = GEO::NO_INDEX;
                    for (const auto& le0 : HEX_LF_INCIDENT_LE[lf0]) {
                        for (const auto& le1 : HEX_LF_INCIDENT_LE[lf1]) {
                            if (le0 == le1) {
                                EXPECT_EQ(common_le, GEO::NO_INDEX);
                                common_le = le0;
                            }
                        }
                    }

                    EXPECT_EQ(common_le, HEX_LF_LF_COMMON_LE[lf0][lf1]);
                }
            }
        }
    }

    TEST_F(HexDescriptorTest, HEX_ENCODED_LF) {
        ASSERT_EQ(M.cells.nb_facets(c), HEX_ENCODED_LF.size());

        for (GEO::index_t lf = 0; lf < M.cells.nb_facets(c); ++lf) {
            ASSERT_EQ(M.cells.facet_nb_vertices(c, lf), 4);
            const auto& ev0 = M.cells.facet_vertex(c, lf, 0);
            const auto& ev1 = M.cells.facet_vertex(c, lf, 1);
            const auto& ev2 = M.cells.facet_vertex(c, lf, 2);
            const auto& ev3 = M.cells.facet_vertex(c, lf, 3);
            EXPECT_EQ((1<<ev0) | (1<<ev1) | (1<<ev2) | (1<<ev3), HEX_ENCODED_LF[lf]);
        }
    }
}