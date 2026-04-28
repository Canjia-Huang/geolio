//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/4/6.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//

#include <gtest/gtest.h>
#include "geogram_mesh_utils/tet_descriptor.h"
#include <geogram/mesh/mesh.h>
#include <unordered_set>

using namespace GEO::MeshUtils;

namespace GEO::MeshUtils::Test
{
    class TetDescriptorTest : public ::testing::Test {
    public:
        void SetUp() override {
            M.vertices.create_vertices(4);
            M.vertices.point(0) = GEO::vec3(0,0,0);
            M.vertices.point(1) = GEO::vec3(1,0,0);
            M.vertices.point(2) = GEO::vec3(0,1,0);
            M.vertices.point(3) = GEO::vec3(0,0,1);
            M.cells.create_tet(0,1,2,3);
        }

        GEO::Mesh M;
        const GEO::index_t c = 0;
    };

    TEST_F(TetDescriptorTest, TET_LV_ADJACENT_LV) {
        ASSERT_EQ(M.cells.nb_vertices(c), TET_LV_ADJACENT_LV.size());

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

            EXPECT_EQ(adj_vertices.size(), TET_LV_ADJACENT_LV[lv].size());
            for (const auto& adj_v : TET_LV_ADJACENT_LV[lv])
                EXPECT_TRUE(adj_vertices.contains(adj_v));
        }
    }

    TEST_F(TetDescriptorTest, TET_LV_ADJACENT_LV_orientation) {
        ASSERT_EQ(M.cells.nb_vertices(c), TET_LV_ADJACENT_LV.size());

        for (GEO::index_t lv = 0; lv < M.cells.nb_vertices(c); ++lv) {
            const auto lf = lv;
            for (GEO::index_t i = 0; i < 3; ++i)
                EXPECT_EQ(TET_LV_ADJACENT_LV[lv][i], M.cells.facet_vertex(c, lf, i));
        }
    }

    TEST_F(TetDescriptorTest, TET_LV_INCIDENT_LE) {
        ASSERT_EQ(M.cells.nb_vertices(c), TET_LV_INCIDENT_LE.size());

        for (GEO::index_t lv = 0; lv < M.cells.nb_vertices(c); ++lv) {
            const auto v = M.cells.vertex(c, lv);

            std::unordered_set<GEO::index_t> adj_edges;
            for (GEO::index_t le = 0; le < M.cells.nb_edges(c); ++le) {
                const auto& ev0 = M.cells.edge_vertex(c, le, 0);
                const auto& ev1 = M.cells.edge_vertex(c, le, 1);
                if (ev0 == v || ev1 == v)
                    adj_edges.insert(le);
            }

            EXPECT_EQ(adj_edges.size(), TET_LV_INCIDENT_LE[lv].size());
            for (const auto& adj_e : TET_LV_INCIDENT_LE[lv])
                EXPECT_TRUE(adj_edges.contains(adj_e));
        }
    }

    TEST_F(TetDescriptorTest, TET_LV_INCIDENT_LF) {
        ASSERT_EQ(M.cells.nb_vertices(c), TET_LV_INCIDENT_LF.size());

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

            EXPECT_EQ(adj_facets.size(), TET_LV_INCIDENT_LF[lv].size());
            for (const auto& adj_f : TET_LV_INCIDENT_LF[lv])
                EXPECT_TRUE(adj_facets.contains(adj_f));
        }
    }

    TEST_F(TetDescriptorTest, TET_LV_INCIDENT_LF_orientation) {
        ASSERT_EQ(M.cells.nb_vertices(c), TET_LV_INCIDENT_LF.size());

        for (GEO::index_t lv = 0; lv < M.cells.nb_vertices(c); ++lv) {
            ASSERT_EQ(TET_LV_INCIDENT_LF[lv].size(), 3);
            const auto& lf0 = TET_LV_INCIDENT_LF[lv][0];
            const auto& lf1 = TET_LV_INCIDENT_LF[lv][1];
            const auto& lf2 = TET_LV_INCIDENT_LF[lv][2];
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
            EXPECT_LE(GEO::Geom::cos_angle(GEO::cross(n0, n1), n2), 0);
        }
    }

    TEST_F(TetDescriptorTest, TET_LE_INCIDENT_LV) {
        ASSERT_EQ(M.cells.nb_edges(c), TET_LE_INCIDENT_LV.size());

        for (GEO::index_t le = 0; le < M.cells.nb_edges(c); ++le) {
            std::unordered_set<GEO::index_t> adj_vertices;

            for (GEO::index_t i = 0; i < 2; ++i)
                adj_vertices.insert(M.cells.edge_vertex(c, le, i));

            EXPECT_EQ(adj_vertices.size(), TET_LE_INCIDENT_LV[le].size());
            for (const auto& adj_v : TET_LE_INCIDENT_LV[le])
                EXPECT_TRUE(adj_vertices.contains(adj_v));
        }
    }

    TEST_F(TetDescriptorTest, TET_LE_OPPOSITE_LE) {
        ASSERT_EQ(M.cells.nb_edges(0), TET_LE_OPPOSITE_LE.size());

        for (GEO::index_t le = 0; le < M.cells.nb_edges(0); ++le) {
            const auto& ev0 = M.cells.edge_vertex(c, le, 0);
            const auto& ev1 = M.cells.edge_vertex(c, le, 1);

            GEO::index_t oppo_le = GEO::NO_INDEX;
            for (GEO::index_t ole = 0; ole < M.cells.nb_edges(c); ++ole) {
                const auto& ev2 = M.cells.edge_vertex(c, ole, 0);
                const auto& ev3 = M.cells.edge_vertex(c, ole, 1);
                if (ev2 != ev0 && ev2 != ev1 && ev3 != ev0 && ev3 != ev1) {
                    EXPECT_EQ(oppo_le, GEO::NO_INDEX);
                    oppo_le = ole;
                }
            }

            EXPECT_EQ(oppo_le, TET_LE_OPPOSITE_LE[le]);
        }
    }

    TEST_F(TetDescriptorTest, TET_LE_INCIDENT_LF) {
        ASSERT_EQ(M.cells.nb_edges(c), TET_LE_INCIDENT_LF.size());

        for (GEO::index_t le = 0; le < M.cells.nb_edges(c); ++le) {
            for (GEO::index_t i = 0; i < 2; ++i)
                EXPECT_EQ(M.cells.edge_adjacent_facet(c, le, i), TET_LE_INCIDENT_LF[le][i]);
        }
    }

    TEST_F(TetDescriptorTest, TET_LF_INCIDENT_LV) {
        ASSERT_EQ(M.cells.nb_facets(0), TET_LF_INCIDENT_LV.size());

        for (GEO::index_t lf = 0; lf < M.cells.nb_facets(c); ++lf) {
            ASSERT_EQ(M.cells.facet_nb_vertices(c, lf), TET_LF_INCIDENT_LV[lf].size());
            for (GEO::index_t lv = 0; lv < M.cells.facet_nb_vertices(c, lf); ++lv)
                EXPECT_EQ(M.cells.facet_vertex(c, lf, lv), TET_LF_INCIDENT_LV[lf][lv]);
        }
    }

    TEST_F(TetDescriptorTest, TET_LF_INCIDENT_LE) {
        ASSERT_EQ(M.cells.nb_facets(c), TET_LF_INCIDENT_LE.size());

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

            EXPECT_EQ(adj_edges.size(), TET_LF_INCIDENT_LE[lf].size());
            for (const auto& adj_e : TET_LF_INCIDENT_LE[lf])
                EXPECT_TRUE(adj_edges.contains(adj_e));
        }
    }

    TEST_F(TetDescriptorTest, TET_LF_LF_COMMON_LE) {
        EXPECT_EQ(M.cells.nb_facets(c), TET_LF_LF_COMMON_LE.size());

        for (GEO::index_t lf0 = 0; lf0 < M.cells.nb_facets(c); ++lf0) {
            EXPECT_EQ(M.cells.nb_facets(c), TET_LF_LF_COMMON_LE[lf0].size());

            for (GEO::index_t lf1 = 0; lf1 < M.cells.nb_facets(c); ++lf1) {
                if (lf0 == lf1)
                    EXPECT_EQ(GEO::NO_INDEX, TET_LF_LF_COMMON_LE[lf0][lf1]);
                else {
                    GEO::index_t common_le = GEO::NO_INDEX;
                    for (const auto& le0 : TET_LF_INCIDENT_LE[lf0]) {
                        for (const auto& le1 : TET_LF_INCIDENT_LE[lf1]) {
                            if (le0 == le1) {
                                EXPECT_EQ(common_le, GEO::NO_INDEX);
                                common_le = le0;
                            }
                        }
                    }

                    EXPECT_EQ(common_le, TET_LF_LF_COMMON_LE[lf0][lf1]);
                }
            }
        }
    }
}