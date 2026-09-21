//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/5/2.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <unordered_set>
#include <gtest/gtest.h>
#include <geogram/mesh/mesh.h>
#include <geolio/mesh/hex_operations.h>
#include <bit>
#include <geogram/mesh/mesh_repair.h>

#include "../utils.h"

namespace geolio::test
{
    class SingleHexOperationsTest : public ::testing::Test {
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

        GEO::Mesh mesh;
        const GEO::index_t c = 0;
    };

    TEST_F(SingleHexOperationsTest, find_hex_vertex) {
        for (GEO::index_t lv = 0; lv < mesh.cells.nb_vertices(c); ++lv)
            EXPECT_EQ(find_hex_vertex(mesh, c, mesh.cells.vertex(c, lv)), lv);
    }

    TEST_F(SingleHexOperationsTest, find_hex_edge_from_local_vertices) {
        for (GEO::index_t le = 0; le < mesh.cells.nb_edges(c); ++le) {
            const auto& lv0 = HEX_LE_INCIDENT_LV[le][0];
            const auto& lv1 = HEX_LE_INCIDENT_LV[le][1];
            EXPECT_EQ(find_hex_edge_from_local_vertices(lv0, lv1), le);
            EXPECT_EQ(find_hex_edge_from_local_vertices(lv1, lv0), le);
        }
    }

    TEST_F(SingleHexOperationsTest, find_hex_edge) {
        for (GEO::index_t le = 0; le < mesh.cells.nb_edges(c); ++le) {
            const auto& ev0 = mesh.cells.edge_vertex(c, le, 0);
            const auto& ev1 = mesh.cells.edge_vertex(c, le, 1);
            EXPECT_EQ(find_hex_edge(mesh, c, ev0, ev1), le);
            EXPECT_EQ(find_hex_edge(mesh, c, ev1, ev0), le);
        }
    }

    TEST_F(SingleHexOperationsTest, find_hex_facet_from_local_vertices) {
        for (GEO::index_t i = 0; i < mesh.cells.nb_vertices(c); ++i) {
            const auto& vi = mesh.cells.vertex(c, i);
            for (GEO::index_t j = 0; j < mesh.cells.nb_vertices(c); ++j) {
                const auto& vj = mesh.cells.vertex(c, j);
                for (GEO::index_t k = 0; k < mesh.cells.nb_vertices(c); ++k) {
                    const auto& vk = mesh.cells.vertex(c, k);
                    for (GEO::index_t l = 0; l < mesh.cells.nb_vertices(c); ++l) {
                        const auto& vl = mesh.cells.vertex(c, l);

                        for (GEO::index_t lf = 0; lf < mesh.cells.nb_facets(c); ++lf) {
                            bool found_vi = false;
                            bool found_vj = false;
                            bool found_vk = false;
                            bool found_vl = false;
                            for (GEO::index_t lv = 0; lv < mesh.cells.facet_nb_vertices(c, lf); ++lv) {
                                if (const auto& v = mesh.cells.facet_vertex(c, lf, lv);
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

    TEST_F(SingleHexOperationsTest, find_hex_facet) {
        for (GEO::index_t i = 0; i < mesh.cells.nb_vertices(c); ++i) {
            const auto& vi = mesh.cells.vertex(c, i);
            for (GEO::index_t j = 0; j < mesh.cells.nb_vertices(c); ++j) {
                const auto& vj = mesh.cells.vertex(c, j);
                for (GEO::index_t k = 0; k < mesh.cells.nb_vertices(c); ++k) {
                    const auto& vk = mesh.cells.vertex(c, k);

                    GEO::index_t found_lf = GEO::NO_INDEX;
                    for (GEO::index_t lf = 0; lf < mesh.cells.nb_facets(c); ++lf) {
                        bool found_vi = false;
                        bool found_vj = false;
                        bool found_vk = false;
                        for (GEO::index_t lv = 0; lv < mesh.cells.facet_nb_vertices(c, lf); ++lv) {
                            if (const auto& v = mesh.cells.facet_vertex(c, lf, lv);
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

                    EXPECT_EQ(find_hex_facet(mesh, c, vi, vj, vk), found_lf);
                }
            }
        }
    }

    class LoopHexesOperationTest : public ::testing::Test {
    protected:
        void SetUp() {
            ASSERT_TRUE(mesh.load(std::string(TEST_DATA_PATH)+"mobius_hexes.geogram"));
        }

        void save_sheet(const std::vector<std::pair<GEO::index_t, GEO::Numeric::uint8>>& sheet_hexes) {
            GEO::Attribute<bool> mesh_c_sheet(mesh.cells.attributes(), "sheet");
            mesh_c_sheet.fill(false);

            GEO::index_t sheet_facets_nb = 0;
            for (const auto& [c, type] : sheet_hexes) {
                mesh_c_sheet[c] = true;

                EXPECT_FALSE(type & 0b11000000); // only first six bits
                sheet_facets_nb += std::popcount(type);
            }

            GEO::Attribute<GEO::index_t> mesh_f_type(mesh.facets.attributes(), "type");
            GEO::index_t new_v = mesh.vertices.create_vertices(4*sheet_facets_nb);
            GEO::index_t new_f = mesh.facets.create_quads(sheet_facets_nb);
            for (const auto& [c, type] : sheet_hexes) {
                constexpr std::array<GEO::index_t, 3> start_les = {0, 1, 8};
                for (GEO::index_t i = 0; i < 3; ++i) {
                    if (type & (1<<(2*i))) {
                        for (GEO::index_t j = 0; j < 4; ++j) {
                            const auto& le = HEX_LE_LOOP_LE[start_les[i]][j];
                            GEO::index_t ev0, ev1;
                            if (HEX_LE_LOOP_LE_ORIENT[start_les[i]][j]) {
                                ev0 = mesh.cells.edge_vertex(c, le, 0);
                                ev1 = mesh.cells.edge_vertex(c, le, 1);
                            }
                            else {
                                ev0 = mesh.cells.edge_vertex(c, le, 1);
                                ev1 = mesh.cells.edge_vertex(c, le, 0);
                            }
                            mesh.vertices.point(new_v+j) = (1-r)*mesh.vertices.point(ev0) + r* mesh.vertices.point(ev1);
                            mesh.facets.set_vertex(new_f, j, new_v+j);
                        }

                        mesh_f_type[new_f] = 2*i;

                        new_v += 4;
                        ++new_f;
                    }
                    if (type & (1<<(2*i+1))) {
                        for (GEO::index_t j = 0; j < 4; ++j) {
                            const auto& le = HEX_LE_LOOP_LE[start_les[i]][j];
                            GEO::index_t ev0, ev1;
                            if (!HEX_LE_LOOP_LE_ORIENT[start_les[i]][j]) {
                                ev0 = mesh.cells.edge_vertex(c, le, 0);
                                ev1 = mesh.cells.edge_vertex(c, le, 1);
                            }
                            else {
                                ev0 = mesh.cells.edge_vertex(c, le, 1);
                                ev1 = mesh.cells.edge_vertex(c, le, 0);
                            }
                            mesh.vertices.point(new_v+j) = (1-r)*mesh.vertices.point(ev0) + r* mesh.vertices.point(ev1);
                            mesh.facets.set_vertex(new_f, j, new_v+j);
                        }

                        mesh_f_type[new_f] = 2*i+1;

                        new_v += 4;
                        ++new_f;
                    }
                }
            }

            EXPECT_TRUE(mesh.save(get_current_test_name()+".geogram"));
        }

        GEO::Mesh mesh;
        const double r = 0.3;
    };

    TEST_F(LoopHexesOperationTest, find_sheet_hexes) {
        std::vector<std::pair<GEO::index_t, GEO::Numeric::uint8>> sheet_hexes;
        find_sheet_hexes(mesh, 0, 0, sheet_hexes);
        save_sheet(sheet_hexes);
    }
}
