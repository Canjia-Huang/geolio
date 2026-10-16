//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/4/5.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <gtest/gtest.h>
#include <geolio/mesh/hex_descriptor.h>
#include <geogram/mesh/mesh.h>
#include <algorithm>
#include <map>
#include <unordered_set>

#include "../utils.h"

namespace geolio::test
{
    class HexDescriptorTest : public ::testing::Test {
    protected:
        void SetUp() override {
            mesh.vertices.create_vertices(8);
            mesh.vertices.point(0) = GEO::vec3(0,0,0);
            mesh.vertices.point(1) = GEO::vec3(1,0,0);
            mesh.vertices.point(2) = GEO::vec3(0,1,0);
            mesh.vertices.point(3) = GEO::vec3(1,1,0);
            mesh.vertices.point(4) = GEO::vec3(0,0,1);
            mesh.vertices.point(5) = GEO::vec3(1,0,1);
            mesh.vertices.point(6) = GEO::vec3(0,1,1);
            mesh.vertices.point(7) = GEO::vec3(1,1,1);
            mesh.cells.create_hex(0,1,2,3,4,5,6,7);
        }

    public:
        GEO::Mesh mesh;
        const GEO::index_t c = 0;
    };

    TEST_F(HexDescriptorTest, HEX_LV_ADJACENT_LV) {
        ASSERT_EQ(mesh.cells.nb_vertices(c), HEX_LV_ADJACENT_LV.size());

        for (GEO::index_t lv = 0; lv < mesh.cells.nb_vertices(c); ++lv) {
            ASSERT_EQ(HEX_LV_ADJACENT_LV[lv].size(), 3);

            {
                const auto v = mesh.cells.vertex(c, lv);

                std::unordered_set<GEO::index_t> adj_vertices;
                for (GEO::index_t le = 0; le < mesh.cells.nb_edges(c); ++le) {
                    const auto& ev0 = mesh.cells.edge_vertex(c, le, 0);
                    const auto& ev1 = mesh.cells.edge_vertex(c, le, 1);
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
                const auto& p = mesh.cells.point(c, lv);

                const auto& lv0 = HEX_LV_ADJACENT_LV[lv][0];
                const auto& lv1 = HEX_LV_ADJACENT_LV[lv][1];
                const auto& lv2 = HEX_LV_ADJACENT_LV[lv][2];
                const auto& p0 = mesh.cells.point(c, lv0);
                const auto& p1 = mesh.cells.point(c, lv1);
                const auto& p2 = mesh.cells.point(c, lv2);
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
        ASSERT_EQ(mesh.cells.nb_vertices(c), HEX_LV_INCIDENT_LE.size());

        for (GEO::index_t lv = 0; lv < mesh.cells.nb_vertices(c); ++lv) {
            ASSERT_EQ(HEX_LV_ADJACENT_LV[lv].size(), 3);

            {
                const auto v = mesh.cells.vertex(c, lv);

                std::unordered_set<GEO::index_t> adj_edges;
                for (GEO::index_t le = 0; le < mesh.cells.nb_edges(c); ++le) {
                    const auto& ev0 = mesh.cells.edge_vertex(c, le, 0);
                    const auto& ev1 = mesh.cells.edge_vertex(c, le, 1);
                    if (ev0 == v || ev1 == v)
                        adj_edges.insert(le);
                }

                EXPECT_EQ(adj_edges.size(), HEX_LV_INCIDENT_LE[lv].size());
                for (const auto& adj_e : HEX_LV_INCIDENT_LE[lv])
                    EXPECT_TRUE(adj_edges.contains(adj_e));
            }

            { // orientation
                const auto& v = mesh.cells.vertex(c, lv);
                const auto& p = mesh.cells.point(c, lv);


                const auto& le0 = HEX_LV_INCIDENT_LE[lv][0];
                const auto& le1 = HEX_LV_INCIDENT_LE[lv][1];
                const auto& le2 = HEX_LV_INCIDENT_LE[lv][2];
                auto v0 = mesh.cells.edge_vertex(c, le0, 0);
                if (v0 == v)
                    v0 = mesh.cells.edge_vertex(c, le0, 1);
                auto v1 = mesh.cells.edge_vertex(c, le1, 0);
                if (v1 == v)
                    v1 = mesh.cells.edge_vertex(c, le1, 1);
                auto v2 = mesh.cells.edge_vertex(c, le2, 0);
                if (v2 == v)
                    v2 = mesh.cells.edge_vertex(c, le2, 1);
                const auto& p0 = mesh.vertices.point(v0);
                const auto& p1 = mesh.vertices.point(v1);
                const auto& p2 = mesh.vertices.point(v2);

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
        ASSERT_EQ(mesh.cells.nb_vertices(c), HEX_LV_INCIDENT_LF.size());

        for (GEO::index_t lv = 0; lv < mesh.cells.nb_vertices(c); ++lv) {
            ASSERT_EQ(HEX_LV_INCIDENT_LF[lv].size(), 3);

            {
                const auto v = mesh.cells.vertex(c, lv);

                std::unordered_set<GEO::index_t> adj_facets;
                for (GEO::index_t lf = 0; lf < mesh.cells.nb_facets(c); ++lf) {
                    for (GEO::index_t lfv = 0; lfv < mesh.cells.facet_nb_vertices(c, lf); ++lfv) {
                        if (mesh.cells.facet_vertex(c, lf, lfv) == v) {
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
                ASSERT_LE(lf0, mesh.cells.nb_facets(c));
                ASSERT_LE(lf1, mesh.cells.nb_facets(c));
                ASSERT_LE(lf2, mesh.cells.nb_facets(c));

                const auto n0 = -GEO::normalize(GEO::Geom::triangle_normal(
                    mesh.vertices.point(mesh.cells.facet_vertex(c, lf0, 0)),
                    mesh.vertices.point(mesh.cells.facet_vertex(c, lf0, 1)),
                    mesh.vertices.point(mesh.cells.facet_vertex(c, lf0, 2))));
                const auto n1 = -GEO::normalize(GEO::Geom::triangle_normal(
                    mesh.vertices.point(mesh.cells.facet_vertex(c, lf1, 0)),
                    mesh.vertices.point(mesh.cells.facet_vertex(c, lf1, 1)),
                    mesh.vertices.point(mesh.cells.facet_vertex(c, lf1, 2))));
                const auto n2 = -GEO::normalize(GEO::Geom::triangle_normal(
                    mesh.vertices.point(mesh.cells.facet_vertex(c, lf2, 0)),
                    mesh.vertices.point(mesh.cells.facet_vertex(c, lf2, 1)),
                    mesh.vertices.point(mesh.cells.facet_vertex(c, lf2, 2))));
                EXPECT_NEAR(GEO::Geom::cos_angle(GEO::cross(n0, n1), n2), -1, 1e-10);
                EXPECT_NEAR(GEO::Geom::cos_angle(GEO::cross(n1, n2), n0), -1, 1e-10);
                EXPECT_NEAR(GEO::Geom::cos_angle(GEO::cross(n2, n0), n1), -1, 1e-10);
            }
        }
    }

    TEST_F(HexDescriptorTest, HEX_LE_INCIDENT_LV) {
        ASSERT_EQ(mesh.cells.nb_edges(c), HEX_LE_INCIDENT_LV.size());

        for (GEO::index_t le = 0; le < mesh.cells.nb_edges(c); ++le) {
            std::unordered_set<GEO::index_t> adj_vertices;

            for (GEO::index_t i = 0; i < 2; ++i)
                adj_vertices.insert(mesh.cells.edge_vertex(c, le, i));

            EXPECT_EQ(adj_vertices.size(), HEX_LE_INCIDENT_LV[le].size());
            for (const auto& adj_v : HEX_LE_INCIDENT_LV[le])
                EXPECT_TRUE(adj_vertices.contains(adj_v));
        }
    }

    TEST_F(HexDescriptorTest, HEX_LE_INCIDENT_LF) {
        ASSERT_EQ(mesh.cells.nb_edges(c), HEX_LE_INCIDENT_LF.size());

        for (GEO::index_t le = 0; le < mesh.cells.nb_edges(c); ++le) {
            {
                for (GEO::index_t i = 0; i < 2; ++i)
                    EXPECT_EQ(mesh.cells.edge_adjacent_facet(c, le, i), HEX_LE_INCIDENT_LF[le][i]);
            }

            { // orientation
                const auto& p0 = mesh.vertices.point(mesh.cells.edge_vertex(c, le, 0));
                const auto& p1 = mesh.vertices.point(mesh.cells.edge_vertex(c, le, 1));

                ASSERT_EQ(HEX_LE_INCIDENT_LF[le].size(), 2);
                const auto& lf0 = HEX_LE_INCIDENT_LF[le][0];
                const auto& lf1 = HEX_LE_INCIDENT_LF[le][1];
                const auto n0 = -GEO::normalize(GEO::Geom::triangle_normal(
                    mesh.vertices.point(mesh.cells.facet_vertex(c, lf0, 0)),
                    mesh.vertices.point(mesh.cells.facet_vertex(c, lf0, 1)),
                    mesh.vertices.point(mesh.cells.facet_vertex(c, lf0, 2))));
                const auto n1 = -GEO::normalize(GEO::Geom::triangle_normal(
                    mesh.vertices.point(mesh.cells.facet_vertex(c, lf1, 0)),
                    mesh.vertices.point(mesh.cells.facet_vertex(c, lf1, 1)),
                    mesh.vertices.point(mesh.cells.facet_vertex(c, lf1, 2))));

                EXPECT_NEAR(GEO::Geom::cos_angle(GEO::cross(n0, n1), p1-p0), -1, 1e-10);
            }
        }
    }

    TEST_F(HexDescriptorTest, HEX_LE_LOOP_LE) {
        ASSERT_EQ(mesh.cells.nb_edges(c), HEX_LE_LOOP_LE.size());

        for (GEO::index_t le = 0; le < mesh.cells.nb_edges(c); ++le) {
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

    TEST_F(HexDescriptorTest, HEX_LE_LOOP_LE_ORIENT) {
        ASSERT_EQ(mesh.cells.nb_edges(c), HEX_LE_LOOP_LE_ORIENT.size());

        for (GEO::index_t le = 0; le < mesh.cells.nb_edges(c); ++le) {
            ASSERT_EQ(HEX_LE_LOOP_LE_ORIENT[le].size(), 4);

            const auto& le0 = HEX_LE_LOOP_LE[le][0];
            const auto& le1 = HEX_LE_LOOP_LE[le][1];
            const auto& le2 = HEX_LE_LOOP_LE[le][2];
            const auto& le3 = HEX_LE_LOOP_LE[le][3];
            const auto d0 = mesh.vertices.point(mesh.cells.edge_vertex(c, le0, 1))-mesh.vertices.point(mesh.cells.edge_vertex(c, le0, 0));
            auto d1 = mesh.vertices.point(mesh.cells.edge_vertex(c, le1, 1))-mesh.vertices.point(mesh.cells.edge_vertex(c, le1, 0));
            auto d2 = mesh.vertices.point(mesh.cells.edge_vertex(c, le2, 1))-mesh.vertices.point(mesh.cells.edge_vertex(c, le2, 0));
            auto d3 = mesh.vertices.point(mesh.cells.edge_vertex(c, le3, 1))-mesh.vertices.point(mesh.cells.edge_vertex(c, le3, 0));

            EXPECT_TRUE(HEX_LE_LOOP_LE_ORIENT[le][0]);
            if (!HEX_LE_LOOP_LE_ORIENT[le][1])
                d1 = -d1;
            if (!HEX_LE_LOOP_LE_ORIENT[le][2])
                d2 = -d2;
            if (!HEX_LE_LOOP_LE_ORIENT[le][3])
                d3 = -d3;

            EXPECT_NEAR(GEO::dot(d0, d1), 1, 1e-10);
            EXPECT_NEAR(GEO::dot(d0, d2), 1, 1e-10);
            EXPECT_NEAR(GEO::dot(d0, d3), 1, 1e-10);
        }
    }

    TEST_F(HexDescriptorTest, HEX_LE_LOOP_LF) {
        ASSERT_EQ(mesh.cells.nb_edges(c), HEX_LE_LOOP_LF.size());

        for (GEO::index_t le = 0; le < mesh.cells.nb_edges(c); ++le) {
            ASSERT_EQ(HEX_LE_LOOP_LF[le].size(), 4);
            {
                std::unordered_set<GEO::index_t> facets;
                for (const auto lf : HEX_LE_LOOP_LF[le])
                    facets.insert(lf);
                EXPECT_EQ(facets.size(), 4);

                const GEO::index_t lf0 = HEX_LE_INCIDENT_LF[le][0];
                const GEO::index_t lf1 = HEX_LE_INCIDENT_LF[le][1];

                GEO::index_t nlf0 = GEO::NO_INDEX;
                for (const auto lf : HEX_LV_INCIDENT_LF[mesh.cells.edge_vertex(c, le, 0)]) {
                    if (lf != lf0 && lf != lf1) {
                        EXPECT_TRUE(nlf0 == GEO::NO_INDEX);
                        nlf0 = lf;
                    }
                }
                EXPECT_FALSE(nlf0 == GEO::NO_INDEX);

                GEO::index_t nlf1 = GEO::NO_INDEX;
                for (const auto lf : HEX_LV_INCIDENT_LF[mesh.cells.edge_vertex(c, le, 1)]) {
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
                const GEO::vec3 en = mesh.vertices.point(mesh.cells.edge_vertex(c, le, 1)) - mesh.vertices.point(mesh.cells.edge_vertex(c, le, 0));
                for (GEO::index_t i = 0; i < 4; ++i) {
                    const auto& lf0 = HEX_LE_LOOP_LF[le][i];
                    const auto& lf1 = HEX_LE_LOOP_LF[le][(i+1)%4];
                    EXPECT_NE(HEX_LF_LF_COMMON_LE[lf0][lf1], GEO::NO_INDEX);

                    const auto n0 = GEO::Geom::triangle_normal(
                        mesh.vertices.point(mesh.cells.facet_vertex(c, lf0, 0)),
                        mesh.vertices.point(mesh.cells.facet_vertex(c, lf0, 1)),
                        mesh.vertices.point(mesh.cells.facet_vertex(c, lf0, 2)));
                    const auto n1 = GEO::Geom::triangle_normal(
                        mesh.vertices.point(mesh.cells.facet_vertex(c, lf1, 0)),
                        mesh.vertices.point(mesh.cells.facet_vertex(c, lf1, 1)),
                        mesh.vertices.point(mesh.cells.facet_vertex(c, lf1, 2)));
                    EXPECT_NEAR(GEO::dot(GEO::cross(n0, n1), -en), 1, 1e-10);
                }
            }
        }
    }

    TEST_F(HexDescriptorTest, HEX_ENCODED_LE) {
        ASSERT_EQ(mesh.cells.nb_edges(c), HEX_ENCODED_LE.size());

        for (GEO::index_t le = 0; le < mesh.cells.nb_edges(c); ++le) {
            const auto& ev0 = mesh.cells.edge_vertex(c, le, 0);
            const auto& ev1 = mesh.cells.edge_vertex(c, le, 1);
            EXPECT_EQ((1<<ev0) | (1<<ev1), HEX_ENCODED_LE[le]);
        }
    }

    TEST_F(HexDescriptorTest, HEX_LF_INCIDENT_LV) {
        ASSERT_EQ(mesh.cells.nb_facets(0), HEX_LF_INCIDENT_LV.size());

        for (GEO::index_t lf = 0; lf < mesh.cells.nb_facets(c); ++lf) {
            ASSERT_EQ(mesh.cells.facet_nb_vertices(c, lf), HEX_LF_INCIDENT_LV[lf].size());
            for (GEO::index_t lv = 0; lv < mesh.cells.facet_nb_vertices(c, lf); ++lv)
                EXPECT_EQ(mesh.cells.facet_vertex(c, lf, lv), HEX_LF_INCIDENT_LV[lf][lv]);
        }
    }

    TEST_F(HexDescriptorTest, HEX_LF_INCIDENT_LE) {
        ASSERT_EQ(mesh.cells.nb_facets(c), HEX_LF_INCIDENT_LE.size());

        for (GEO::index_t lf = 0; lf < mesh.cells.nb_facets(c); ++lf) {
            ASSERT_EQ(HEX_LF_INCIDENT_LE[lf].size(), 4);

            {
                std::unordered_set<GEO::index_t> facet_vertices;
                for (GEO::index_t lv = 0; lv < mesh.cells.facet_nb_vertices(c, lf); ++lv)
                    facet_vertices.insert(mesh.cells.facet_vertex(c, lf, lv));

                std::unordered_set<GEO::index_t> adj_edges;
                for (GEO::index_t le = 0; le < mesh.cells.nb_edges(c); ++le) {
                    const auto& ev0 = mesh.cells.edge_vertex(c, le, 0);
                    const auto& ev1 = mesh.cells.edge_vertex(c, le, 1);
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
                    const auto& ev00 = mesh.cells.edge_vertex(c, le0, 0);
                    const auto& ev01 = mesh.cells.edge_vertex(c, le0, 1);
                    const auto& ev10 = mesh.cells.edge_vertex(c, le1, 0);
                    const auto& ev11 = mesh.cells.edge_vertex(c, le1, 1);
                    EXPECT_TRUE(ev00 == ev10 || ev00 == ev11 ||
                                ev01 == ev10 || ev01 == ev11);
                }
            }
        }
    }

    TEST_F(HexDescriptorTest, HEX_LF_OPPOSITE_LF) {
        ASSERT_EQ(mesh.cells.nb_facets(c), HEX_LF_OPPOSITE_LF.size());

        for (GEO::index_t lf = 0; lf < mesh.cells.nb_facets(c); ++lf) {
            std::unordered_set<GEO::index_t> facet_vertices;
            for (GEO::index_t lv = 0; lv < mesh.cells.facet_nb_vertices(c, lf); ++lv)
                facet_vertices.insert(mesh.cells.facet_vertex(c, lf, lv));

            for (GEO::index_t nlf = 0; nlf < mesh.cells.nb_facets(c); ++nlf) {
                bool opposite = true;
                for (GEO::index_t nlv = 0; nlv < mesh.cells.facet_nb_vertices(c, nlf); ++nlv) {
                    if (facet_vertices.contains(mesh.cells.facet_vertex(c, nlf, nlv))) {
                        opposite = false;
                        break;
                    }
                }
                if (opposite)
                    EXPECT_EQ(nlf, HEX_LF_OPPOSITE_LF[lf]);
            }
        }
    }

    TEST_F(HexDescriptorTest, HEX_LF_LF_LF_COMMON_LV) {
        for (GEO::index_t lf0 = 0; lf0 < mesh.cells.nb_facets(c); ++lf0) {
            for (GEO::index_t lf1 = 0; lf1 < mesh.cells.nb_facets(c); ++lf1) {
                if (lf0 == lf1)
                    continue;

                for (GEO::index_t lf2 = 0; lf2 < mesh.cells.nb_facets(c); ++lf2) {
                    if (lf0 == lf2 || lf1 == lf2)
                        continue;

                    GEO::index_t common_lv = GEO::NO_INDEX;

                    for (GEO::index_t lv0 = 0; lv0 < mesh.cells.facet_nb_vertices(c, lf0); ++lv0) {
                        const auto& v0 = mesh.cells.facet_vertex(c, lf0, lv0);
                        for (GEO::index_t lv1 = 0; lv1 < mesh.cells.facet_nb_vertices(c, lf1); ++lv1) {
                            const auto& v1 = mesh.cells.facet_vertex(c, lf1, lv1);
                            for (GEO::index_t lv2 = 0; lv2 < mesh.cells.facet_nb_vertices(c, lf2); ++lv2) {
                                const auto& v2 = mesh.cells.facet_vertex(c, lf2, lv2);

                                if (v0 == v1 && v1 == v2) {
                                    EXPECT_EQ(common_lv, GEO::NO_INDEX); // only one common vertex
                                    for (GEO::index_t lv = 0; lv < mesh.cells.nb_vertices(c); ++lv) {
                                        if (mesh.cells.vertex(c, lv) == v0) {
                                            common_lv = lv;
                                            break;
                                        }
                                    }
                                }
                            }
                        }
                    }

                    EXPECT_EQ(common_lv, HEX_LF_LF_LF_COMMON_LV(lf0, lf1, lf2));
                }
            }
        }
    }

    TEST_F(HexDescriptorTest, HEX_LF_LF_COMMON_LE) {
        EXPECT_EQ(mesh.cells.nb_facets(c), HEX_LF_LF_COMMON_LE.size());

        for (GEO::index_t lf0 = 0; lf0 < mesh.cells.nb_facets(c); ++lf0) {
            EXPECT_EQ(mesh.cells.nb_facets(c), HEX_LF_LF_COMMON_LE[lf0].size());

            for (GEO::index_t lf1 = 0; lf1 < mesh.cells.nb_facets(c); ++lf1) {
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
        ASSERT_EQ(mesh.cells.nb_facets(c), HEX_ENCODED_LF.size());

        for (GEO::index_t lf = 0; lf < mesh.cells.nb_facets(c); ++lf) {
            ASSERT_EQ(mesh.cells.facet_nb_vertices(c, lf), 4);
            const auto& ev0 = mesh.cells.facet_vertex(c, lf, 0);
            const auto& ev1 = mesh.cells.facet_vertex(c, lf, 1);
            const auto& ev2 = mesh.cells.facet_vertex(c, lf, 2);
            const auto& ev3 = mesh.cells.facet_vertex(c, lf, 3);
            EXPECT_EQ((1<<ev0) | (1<<ev1) | (1<<ev2) | (1<<ev3), HEX_ENCODED_LF[lf]);
        }
    }

    /**
     * @brief One entry of `HEX_TO_TET` and its main diagonal.
     *
     * `HEX_TO_TET[apex]` cuts the unit hexahedron along the main diagonal
     * (`apex`, `antipode`) into the six tetrahedra `{apex, P, Q, antipode}`,
     * where `(P, Q)` are the two intermediate prefixes of a permutation of the
     * three axis-aligned edges `U`, `V` and `W` incident to `apex`.
     *
     * The unified convention shared by the four entries of `HEX_TO_TET` is:
     *   - `U`, `V` and `W` are ordered by the x, y and z axis, and signed
     *     toward `antipode`;
     *   - the six tetrahedra are stored in the order `UVW`, `UWV`, `VUW`,
     *     `VWU`, `WUV`, `WVU`;
     *   - the first local vertex is always `apex`, the last one always
     *     `antipode`, and the two middle ones are ordered so that the signed
     *     volume of the tetrahedron is positive.
     */
    struct HexToTetCase {
        const char* name;      //!< Name of the decomposition table.
        GEO::index_t apex;     //!< First endpoint of the main diagonal; its index in `HEX_TO_TET`.
        GEO::index_t antipode; //!< Second endpoint of the main diagonal.
    };

    //! Prints the table name instead of the raw bytes in failure messages.
    inline void PrintTo(const HexToTetCase& hex_to_tet, std::ostream* os) {
        *os << hex_to_tet.name;
    }

    /**
     * @brief File name of the mesh exported by the current parameterized test.
     *
     * `get_current_test_name()` splices parameterized names with `/`, which is
     * not a portable file name character, so it is replaced by `_`.
     */
    inline std::string current_test_file_name() {
        std::string name = get_current_test_name();
        std::replace(name.begin(), name.end(), '/', '_');
        return name;
    }

    class HexToTetDescriptorTest : public HexDescriptorTest,
                                   public ::testing::WithParamInterface<HexToTetCase> {
    protected:
        void SetUp() override {
            HexDescriptorTest::SetUp();
            decompose(HEX_TO_TETS[GetParam().apex]);
        }

        void decompose(const std::array<GEO::index_t, 24>& hex_to_tet) {
            mesh.cells.clear();
            mesh.cells.create_tets(6);
            for (const auto& c : mesh.cells) {
                for (GEO::index_t lv = 0; lv < 4; ++lv)
                    mesh.cells.set_vertex(c, lv, hex_to_tet[4*c+lv]);
            }
        }

        /// Signed volume of tetrahedron `t` of the decomposed mesh.
        double tet_signed_volume(const GEO::index_t t) const {
            return GEO::Geom::tetra_signed_volume(
                mesh.cells.point(t, 0),
                mesh.cells.point(t, 1),
                mesh.cells.point(t, 2),
                mesh.cells.point(t, 3));
        }

        /// Every tetrahedron has to be positively oriented and of volume 1/6.
        void check_volumes() {
            for (const auto& c : mesh.cells)
                EXPECT_EQ(tet_signed_volume(c), 1.0/6);
        }

        /// Number of coordinates along which vertices `v0` and `v1` differ.
        GEO::index_t nb_axes_differ(const GEO::index_t v0, const GEO::index_t v1) const {
            const GEO::vec3 p0 = mesh.vertices.point(v0);
            const GEO::vec3 p1 = mesh.vertices.point(v1);

            GEO::index_t nb_axes = 0;
            for (GEO::index_t axis = 0; axis < 3; ++axis) {
                if (p0[axis] != p1[axis])
                    ++nb_axes;
            }
            return nb_axes;
        }

        /// The local frame (U, V, W) of the decomposition: the three cube-edge
        /// neighbours of the apex, ordered by the x, y and z axis and signed
        /// toward the antipode.
        std::array<GEO::index_t, 3> local_frame() const {
            const GEO::index_t apex = GetParam().apex;
            const GEO::vec3 origin = mesh.vertices.point(apex);
            const GEO::vec3 diagonal = mesh.vertices.point(GetParam().antipode) - origin;

            std::array<GEO::index_t, 3> uvw = {GEO::NO_INDEX, GEO::NO_INDEX, GEO::NO_INDEX};
            for (GEO::index_t v = 0; v < mesh.vertices.nb(); ++v) {
                if (v == apex)
                    continue;

                const GEO::vec3 offset = mesh.vertices.point(v) - origin;
                GEO::index_t axis = GEO::NO_INDEX;
                GEO::index_t nb_non_zero = 0;
                for (GEO::index_t k = 0; k < 3; ++k) {
                    if (offset[k] != 0) {
                        axis = k;
                        ++nb_non_zero;
                    }
                }

                // Only the three cube edges incident to the apex are axis aligned.
                if (nb_non_zero != 1)
                    continue;
                if (offset[axis]*diagonal[axis] > 0)
                    uvw[axis] = v;
            }
            return uvw;
        }

        /// Vertex located at `apex + offset`, or `GEO::NO_INDEX` if there is none.
        GEO::index_t vertex_at_offset(const GEO::vec3& offset) const {
            const GEO::vec3 p = mesh.vertices.point(GetParam().apex) + offset;
            for (GEO::index_t v = 0; v < mesh.vertices.nb(); ++v) {
                const GEO::vec3 q = mesh.vertices.point(v);
                if (p[0] == q[0] && p[1] == q[1] && p[2] == q[2])
                    return v;
            }
            return GEO::NO_INDEX;
        }

        /// The six tetrahedra prescribed by the unified convention, in the
        /// order `UVW`, `UWV`, `VUW`, `VWU`, `WUV`, `WVU`.
        std::array<std::array<GEO::index_t, 4>, 6> expected_tets() const {
            static constexpr std::array<std::array<GEO::index_t, 3>, 6> permutations = {
                {
                    {0, 1, 2}, // UVW
                    {0, 2, 1}, // UWV
                    {1, 0, 2}, // VUW
                    {1, 2, 0}, // VWU
                    {2, 0, 1}, // WUV
                    {2, 1, 0}  // WVU
                }
            };

            const GEO::vec3 origin = mesh.vertices.point(GetParam().apex);
            const auto uvw = local_frame();

            std::array<GEO::vec3, 3> offsets;
            for (GEO::index_t k = 0; k < 3; ++k)
                offsets[k] = mesh.vertices.point(uvw[k]) - origin;

            std::array<std::array<GEO::index_t, 4>, 6> tets;
            for (GEO::index_t i = 0; i < 6; ++i) {
                const auto& permutation = permutations[i];
                tets[i] = {
                    GetParam().apex,
                    vertex_at_offset(offsets[permutation[0]]),
                    vertex_at_offset(offsets[permutation[0]] + offsets[permutation[1]]),
                    GetParam().antipode
                };
            }
            return tets;
        }

        /// If the triangle lies in a boundary plane of the unit hexahedron,
        /// returns `{axis, coordinate}`; otherwise `{NO_INDEX, NO_INDEX}`.
        std::pair<GEO::index_t, GEO::index_t> boundary_plane(
            const std::array<GEO::index_t, 3>& triangle
            ) const {
            for (GEO::index_t axis = 0; axis < 3; ++axis) {
                const double coord = mesh.vertices.point(triangle[0])[axis];
                if (coord != 0 && coord != 1)
                    continue;
                if (mesh.vertices.point(triangle[1])[axis] == coord &&
                    mesh.vertices.point(triangle[2])[axis] == coord)
                    return {axis, coord == 0 ? 0 : 1};
            }
            return {GEO::NO_INDEX, GEO::NO_INDEX};
        }
    };

    //! All four entries of `HEX_TO_TET` are checked by the very same test items.
    INSTANTIATE_TEST_SUITE_P(
        HexToTet,
        HexToTetDescriptorTest,
        ::testing::Values(
            HexToTetCase{"HEX_TO_TET_0_7", 0, 7},
            HexToTetCase{"HEX_TO_TET_1_6", 1, 6},
            HexToTetCase{"HEX_TO_TET_2_5", 2, 5},
            HexToTetCase{"HEX_TO_TET_3_4", 3, 4}
        ),
        [](const ::testing::TestParamInfo<HexToTetCase>& info) {
            return std::string(info.param.name);
        });

    TEST_P(HexToTetDescriptorTest, HEX_TO_TET_TABLE) {
        ASSERT_EQ(HEX_TO_TETS.size(), 4);
        ASSERT_EQ(HEX_TO_TETS[GetParam().apex].size(), 24);
        ASSERT_EQ(mesh.vertices.nb(), 8);
        ASSERT_EQ(mesh.cells.nb(), 6);

        std::vector<std::unordered_set<GEO::index_t>> tets(6);
        std::array<GEO::index_t, 8> nb_incident_tets = {0, 0, 0, 0, 0, 0, 0, 0};

        for (GEO::index_t t = 0; t < 6; ++t) {
            for (GEO::index_t lv = 0; lv < 4; ++lv) {
                const auto v = mesh.cells.vertex(t, lv);
                ASSERT_LT(v, mesh.vertices.nb());
                EXPECT_TRUE(tets[t].insert(v).second)
                    << "tetrahedron " << t << " uses local vertex " << v << " twice";
            }
            EXPECT_EQ(tets[t].size(), 4);

            for (const auto v : tets[t])
                ++nb_incident_tets[v];
        }

        { // every cube corner is used, the diagonal endpoints by all six tetrahedra
            for (GEO::index_t v = 0; v < 8; ++v) {
                const GEO::index_t expected_nb =
                        (v == GetParam().apex || v == GetParam().antipode) ? 6 : 2;
                EXPECT_EQ(nb_incident_tets[v], expected_nb) << "cube corner " << v;
            }
        }

        { // the six tetrahedra are pairwise distinct
            for (GEO::index_t t0 = 0; t0 < 6; ++t0) {
                for (GEO::index_t t1 = t0 + 1; t1 < 6; ++t1) {
                    EXPECT_FALSE(tets[t0] == tets[t1])
                        << "tetrahedra " << t0 << " and " << t1 << " are identical";
                }
            }
        }

        { // each tetrahedron has 3 cube edges, 2 face diagonals and 1 body diagonal
            for (GEO::index_t t = 0; t < 6; ++t) {
                GEO::index_t nb_cube_edges = 0;
                GEO::index_t nb_face_diagonals = 0;
                GEO::index_t nb_body_diagonals = 0;

                for (GEO::index_t lv0 = 0; lv0 < 4; ++lv0) {
                    for (GEO::index_t lv1 = lv0 + 1; lv1 < 4; ++lv1) {
                        switch (nb_axes_differ(mesh.cells.vertex(t, lv0), mesh.cells.vertex(t, lv1))) {
                            case 1: ++nb_cube_edges; break;
                            case 2: ++nb_face_diagonals; break;
                            case 3: ++nb_body_diagonals; break;
                            default: ADD_FAILURE() << "tetrahedron " << t << " has a degenerate edge";
                        }
                    }
                }

                EXPECT_EQ(nb_cube_edges, 3) << "tetrahedron " << t;
                EXPECT_EQ(nb_face_diagonals, 2) << "tetrahedron " << t;
                EXPECT_EQ(nb_body_diagonals, 1) << "tetrahedron " << t;
            }
        }
    }

    TEST_P(HexToTetDescriptorTest, HEX_TO_TET_DIAGONAL) {
        const GEO::index_t apex = GetParam().apex;
        const GEO::index_t antipode = GetParam().antipode;

        EXPECT_LT(apex, antipode);
        EXPECT_EQ(nb_axes_differ(apex, antipode), 3); // the two endpoints are opposite corners

        for (GEO::index_t t = 0; t < 6; ++t) {
            EXPECT_EQ(mesh.cells.vertex(t, 0), apex) << "tetrahedron " << t;
            EXPECT_EQ(mesh.cells.vertex(t, 3), antipode) << "tetrahedron " << t;

            for (GEO::index_t lv = 1; lv < 3; ++lv) {
                EXPECT_NE(mesh.cells.vertex(t, lv), apex) << "tetrahedron " << t;
                EXPECT_NE(mesh.cells.vertex(t, lv), antipode) << "tetrahedron " << t;
            }
        }

        { // the main diagonal is the only pair shared by all six tetrahedra
            for (GEO::index_t v0 = 0; v0 < mesh.vertices.nb(); ++v0) {
                for (GEO::index_t v1 = v0 + 1; v1 < mesh.vertices.nb(); ++v1) {
                    GEO::index_t nb_common_tets = 0;
                    for (GEO::index_t t = 0; t < 6; ++t) {
                        bool has_v0 = false;
                        bool has_v1 = false;
                        for (GEO::index_t lv = 0; lv < 4; ++lv) {
                            const auto v = mesh.cells.vertex(t, lv);
                            has_v0 = has_v0 || v == v0;
                            has_v1 = has_v1 || v == v1;
                        }
                        if (has_v0 && has_v1)
                            ++nb_common_tets;
                    }

                    if (v0 == apex && v1 == antipode)
                        EXPECT_EQ(nb_common_tets, 6);
                    else
                        EXPECT_LE(nb_common_tets, 2) << "vertices " << v0 << " and " << v1;
                }
            }
        }
    }

    TEST_P(HexToTetDescriptorTest, HEX_TO_TET_ORIENTATION) {
        // A positive signed volume for every tetrahedron is the unified winding
        // convention of the four decomposition tables.
        check_volumes();

        for (const auto& c : mesh.cells)
            EXPECT_GT(tet_signed_volume(c), 0) << "tetrahedron " << c << " is inverted";
    }

    TEST_P(HexToTetDescriptorTest, HEX_TO_TET_TILING) {
        { // the six tetrahedra fill the unit hexahedron without overlapping
            double volume = 0;
            for (const auto& c : mesh.cells)
                volume += tet_signed_volume(c);
            EXPECT_NEAR(volume, 1.0, 1e-12);
        }

        // Count, for every triangular face of the decomposition, the tetrahedra
        // it belongs to.
        std::map<std::array<GEO::index_t, 3>, GEO::index_t> faces;
        for (GEO::index_t t = 0; t < 6; ++t) {
            for (GEO::index_t lv = 0; lv < 4; ++lv) {
                std::array<GEO::index_t, 3> triangle;
                GEO::index_t corner = 0;
                for (GEO::index_t i = 0; i < 4; ++i) {
                    if (i != lv)
                        triangle[corner++] = mesh.cells.vertex(t, i);
                }
                std::sort(triangle.begin(), triangle.end());
                ++faces[triangle];
            }
        }
        ASSERT_EQ(faces.size(), 18); // 12 boundary faces + 6 interior faces

        std::map<std::array<GEO::index_t, 2>, std::vector<std::array<GEO::index_t, 3>>> planes;
        GEO::index_t nb_boundary_faces = 0;
        GEO::index_t nb_interior_faces = 0;

        for (const auto& [triangle, nb_incident_tets] : faces) {
            const auto plane = boundary_plane(triangle);
            if (plane.first == GEO::NO_INDEX) {
                EXPECT_EQ(nb_incident_tets, 2) << "interior face must be shared by two tetrahedra";
                ++nb_interior_faces;
            }
            else {
                EXPECT_EQ(nb_incident_tets, 1) << "boundary face must belong to one tetrahedron";
                ++nb_boundary_faces;
                planes[{plane.first, plane.second}].push_back(triangle);
            }
        }
        EXPECT_EQ(nb_boundary_faces, 12);
        EXPECT_EQ(nb_interior_faces, 6);

        { // each cube face is halved by a diagonal and covered by two tetrahedra
            ASSERT_EQ(planes.size(), 6);
            for (const auto& [plane, triangles] : planes) {
                ASSERT_EQ(triangles.size(), 2)
                    << "cube face on axis " << plane[0] << " at " << plane[1];

                std::unordered_set<GEO::index_t> corners;
                for (const auto& triangle : triangles)
                    corners.insert(triangle.begin(), triangle.end());
                EXPECT_EQ(corners.size(), 4)
                    << "cube face on axis " << plane[0] << " at " << plane[1];

                GEO::index_t nb_common_corners = 0;
                for (const auto& v : triangles[0]) {
                    if (std::find(triangles[1].begin(), triangles[1].end(), v) != triangles[1].end())
                        ++nb_common_corners;
                }
                EXPECT_EQ(nb_common_corners, 2); // the two triangles share the face diagonal
            }
        }
    }

    TEST_P(HexToTetDescriptorTest, HEX_TO_TET_CONVENTION) {
        // The local frame has to be readable from the cube itself.
        const auto uvw = local_frame();
        for (GEO::index_t k = 0; k < 3; ++k) {
            ASSERT_NE(uvw[k], GEO::NO_INDEX) << "local frame direction " << k;
            EXPECT_EQ(nb_axes_differ(GetParam().apex, uvw[k]), 1) << "local frame direction " << k;
        }

        // All four tables have to follow the same rule: the i-th tetrahedron is
        // built from the i-th permutation of (U, V, W) and is wound positively.
        const auto expected = expected_tets();
        for (GEO::index_t t = 0; t < 6; ++t) {
            SCOPED_TRACE(t);

            EXPECT_EQ(mesh.cells.vertex(t, 0), expected[t][0]);
            EXPECT_EQ(mesh.cells.vertex(t, 3), expected[t][3]);

            const double prescribed_volume = GEO::Geom::tetra_signed_volume(
                mesh.vertices.point(expected[t][0]),
                mesh.vertices.point(expected[t][1]),
                mesh.vertices.point(expected[t][2]),
                mesh.vertices.point(expected[t][3]));
            ASSERT_NE(prescribed_volume, 0);

            if (prescribed_volume > 0) {
                EXPECT_EQ(mesh.cells.vertex(t, 1), expected[t][1]);
                EXPECT_EQ(mesh.cells.vertex(t, 2), expected[t][2]);
            }
            else {
                EXPECT_EQ(mesh.cells.vertex(t, 1), expected[t][2]);
                EXPECT_EQ(mesh.cells.vertex(t, 2), expected[t][1]);
            }
        }
    }

    TEST_P(HexToTetDescriptorTest, HEX_TO_TET_SAVE) {
        mesh.save(current_test_file_name()+".geogram");
    }
}
