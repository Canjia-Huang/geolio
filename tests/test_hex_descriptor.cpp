//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/4/5.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//

#include <gtest/gtest.h>
#include "mesh_utils/hex_descriptor.h"
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
    }

    TEST_F(HexDescriptorTest, HEX_LV_INCIDENT_LE) {
        ASSERT_EQ(M.cells.nb_vertices(c), HEX_LV_INCIDENT_LE.size());

        for (GEO::index_t lv = 0; lv < M.cells.nb_vertices(c); ++lv) {
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
    }

    TEST_F(HexDescriptorTest, HEX_LV_INCIDENT_LF) {
        ASSERT_EQ(M.cells.nb_vertices(c), HEX_LV_INCIDENT_LF.size());

        for (GEO::index_t lv = 0; lv < M.cells.nb_vertices(c); ++lv) {
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
            for (GEO::index_t i = 0; i < 2; ++i)
                EXPECT_EQ(M.cells.edge_adjacent_facet(c, le, i), HEX_LE_INCIDENT_LF[le][i]);
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