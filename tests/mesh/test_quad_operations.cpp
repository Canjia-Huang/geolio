//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/21.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <bit>
#include <ranges>
#include <unordered_set>
#include <geogram/mesh/mesh_repair.h>
#include <geolio/mesh/quad_operations.h>
#include <gtest/gtest.h>
#include "../utils.h"

namespace geolio::test
{
    class LoopQuadsOperationTest : public ::testing::Test {
    protected:
        void save_sheet(const std::vector<std::pair<GEO::index_t, GEO::Numeric::uint8>>& sheet_quads) {
            GEO::Attribute<bool> mesh_f_sheet(mesh.facets.attributes(), "sheet");
            mesh_f_sheet.fill(false);

            GEO::index_t sheet_edges_nb = 0;
            for (const auto& [f, type] : sheet_quads) {
                mesh_f_sheet[f] = true;

                EXPECT_FALSE(type & 0b110000); // only first four bits
                sheet_edges_nb += std::popcount(type);
            }

            GEO::Attribute<GEO::index_t> mesh_e_type(mesh.edges.attributes(), "type");
            GEO::index_t new_v = mesh.vertices.create_vertices(4*sheet_edges_nb);
            GEO::index_t new_e = mesh.edges.create_edges(sheet_edges_nb);
            GEO::index_t v0, v1, v2, v3;
            auto create_edge = [&](const GEO::index_t type) {
                if (mesh.vertices.dimension() == 3) {
                    mesh.vertices.point(new_v) = (1-r)*mesh.vertices.point(v0) + r* mesh.vertices.point(v1);
                    mesh.vertices.point(new_v+1) = (1-r)*mesh.vertices.point(v2) + r* mesh.vertices.point(v3);
                }
                else {
                    mesh.vertices.point<2>(new_v) = (1-r)*mesh.vertices.point<2>(v0) + r* mesh.vertices.point<2>(v1);
                    mesh.vertices.point<2>(new_v+1) = (1-r)*mesh.vertices.point<2>(v2) + r* mesh.vertices.point<2>(v3);
                }

                mesh.edges.set_vertex(new_e, 0, new_v);
                mesh.edges.set_vertex(new_e, 1, new_v+1);
                mesh_e_type[new_e] = type;
                new_v += 2;
                ++new_e;
            };
            for (const auto& [f, type] : sheet_quads) {
                if (type & 0b0001) {
                    v0 = mesh.facets.vertex(f, 0);
                    v1 = mesh.facets.vertex(f, 1);
                    v2 = mesh.facets.vertex(f, 3);
                    v3 = mesh.facets.vertex(f, 2);
                    create_edge(0);
                }
                if (type & 0b0010) {
                    v0 = mesh.facets.vertex(f, 1);
                    v1 = mesh.facets.vertex(f, 0);
                    v2 = mesh.facets.vertex(f, 2);
                    v3 = mesh.facets.vertex(f, 3);
                    create_edge(1);
                }
                if (type & 0b0100) {
                    v0 = mesh.facets.vertex(f, 0);
                    v1 = mesh.facets.vertex(f, 3);
                    v2 = mesh.facets.vertex(f, 1);
                    v3 = mesh.facets.vertex(f, 2);
                    create_edge(2);
                }
                if (type & 0b1000) {
                    v0 = mesh.facets.vertex(f, 3);
                    v1 = mesh.facets.vertex(f, 0);
                    v2 = mesh.facets.vertex(f, 2);
                    v3 = mesh.facets.vertex(f, 1);
                    create_edge(3);
                }
            }

            EXPECT_TRUE(mesh.save(get_current_test_name()+".geogram"));
        }

        GEO::Mesh mesh;
        const double r = 0.3;
    };

    class MobiusLoopQuadsOperationTest : public LoopQuadsOperationTest {
    protected:
        void SetUp() override {
            ASSERT_TRUE(mesh.load(std::string(TEST_DATA_PATH)+"twice_mobius_quads.geogram"));
        }
    };

    TEST_F(MobiusLoopQuadsOperationTest, find_sheet_quads) {
        std::vector<std::pair<GEO::index_t, GEO::Numeric::uint8>> sheet_quads;
        find_sheet_quads(mesh, 0, 0, sheet_quads);

        EXPECT_EQ(sheet_quads.size(), 65);
        for (const auto&type: sheet_quads | std::views::values)
            EXPECT_EQ(std::popcount(type), 1);

        save_sheet(sheet_quads);
    }

    class EightLoopQuadsOperationTest : public LoopQuadsOperationTest {
    protected:
        void SetUp() override {
            ASSERT_TRUE(mesh.load(std::string(TEST_DATA_PATH)+"eight_quads.geogram"));
        }
    };

    TEST_F(EightLoopQuadsOperationTest, find_sheet_quads) {
        std::vector<std::pair<GEO::index_t, GEO::Numeric::uint8>> sheet_quads;
        find_sheet_quads(mesh, 0, 0, sheet_quads);

        EXPECT_EQ(sheet_quads.size(), mesh.facets.nb());
        for (const auto& [f, type] : sheet_quads) {
            if (f == 0) {
                EXPECT_TRUE(type & 0b1100);
                EXPECT_TRUE(type & 0b0011);
            }
            else
                EXPECT_EQ(std::popcount(type), 1);
        }

        save_sheet(sheet_quads);
    }
}