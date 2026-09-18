//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/3/23.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <gtest/gtest.h>
#include <numeric>
#include <ranges>
#include <unordered_set>
#include <vector>
#include <geogram/mesh/mesh_io.h>
#include "../utils.h"
#include <geolio/mesh/mesh_operations.h>
#include <geolio/mesh/tet_operations.h>

#include "geolio/common/log.h"

namespace geolio::test
{
    class SingleTetOperationsTest : public ::testing::Test {
    protected:
        void SetUp() override {
            M.vertices.create_vertices(4);
            M.vertices.point(0) = GEO::vec3(0,0,0);
            M.vertices.point(1) = GEO::vec3(1,0,0);
            M.vertices.point(2) = GEO::vec3(0,1,0);
            M.vertices.point(3) = GEO::vec3(0,0,1);
            M.cells.create_tet(0,1,2,3);

            ASSERT_GT(GEO::Geom::tetra_signed_volume(
                M.vertices.point(0),
                M.vertices.point(1),
                M.vertices.point(2),
                M.vertices.point(3)), 0);
        }

    public:
        GEO::Mesh M;
        const GEO::index_t c = 0;
    };

    TEST_F(SingleTetOperationsTest, find_tet_edge_from_local_vertices) {
        for (GEO::index_t le = 0; le < M.cells.nb_edges(c); ++le) {
            const auto& lv0 = TET_LE_INCIDENT_LV[le][0];
            const auto& lv1 = TET_LE_INCIDENT_LV[le][1];
            EXPECT_EQ(find_tet_edge_from_local_vertices(lv0, lv1), le);
            EXPECT_EQ(find_tet_edge_from_local_vertices(lv1, lv0), le);
        }
    }

    TEST_F(SingleTetOperationsTest, find_tet_edge) {
        for (GEO::index_t le = 0; le < M.cells.nb_edges(c); ++le) {
            const auto& ev0 = M.cells.edge_vertex(c, le, 0);
            const auto& ev1 = M.cells.edge_vertex(c, le, 1);
            EXPECT_EQ(find_tet_edge(M, c, ev0, ev1), le);
            EXPECT_EQ(find_tet_edge(M, c, ev1, ev0), le);
        }
    }

    TEST_F(SingleTetOperationsTest, get_tet_facet_another_vertex) {
        for (GEO::index_t lf = 0; lf < M.cells.nb_facets(c); ++lf) {
            for (GEO::index_t lv = 0; lv < 3; ++lv) {
                const auto& v0 = M.cells.facet_vertex(c, lf, lv);
                const auto& v1 = M.cells.facet_vertex(c, lf, (lv+1)%3);
                const auto& v2 = M.cells.facet_vertex(c, lf, (lv+2)%3);
                EXPECT_EQ(get_tet_facet_another_vertex(M, c, lf, v0, v1), v2);
            }
        }
    }

    /* ============================================================================================================= */

    class TetOperationsTest : public ::testing::Test {
    protected:
        void SetUp() override {
            generate_random_delaunay3d_mesh(mesh, 20, 10);

            original_mesh.copy(mesh);
        }

        void for_each_c() {
            for (const auto& c : original_mesh.cells) {
                mesh.copy(original_mesh);

                /* Compute */
                perform_operation(c);

                /* Eval */
                check_connections();
            }
        }
        virtual void perform_operation(GEO::index_t c) {}

        void for_each_c_lf() {
            for (const auto& c : original_mesh.cells) {
                for (GEO::index_t lf = 0, lf_end = original_mesh.cells.nb_facets(c); lf < lf_end; ++lf) {
                    mesh.copy(original_mesh);

                    /* Compute */
                    perform_operation(c, lf);

                    /* Eval */
                    check_connections();
                }
            }
        }
        virtual void perform_operation(GEO::index_t c, GEO::index_t lf_or_le) {}

        void for_each_c_le() {
            for (const auto& c : original_mesh.cells) {
                for (GEO::index_t le = 0, le_end = original_mesh.cells.nb_edges(c); le < le_end; ++le) {
                    mesh.copy(original_mesh);

                    /* Compute */
                    perform_operation(c, le);

                    /* Eval */
                    check_connections();
                }
            }
        }

        /**
         * Number of operations the test actually performed.
         *
         * The operations guarded by a validity predicate may be rejected; the tests assert that at
         * least one of them went through, so that a predicate that becomes too strict cannot turn
         * them into empty loops.
         */
        GEO::index_t performed_operations_nb = 0;

        /**
         * Verify that reconnecting the mesh preserves the current cell adjacency layout.
         */
        void check_connections() {
            std::vector<GEO::index_t> current_connections(4*mesh.cells.nb(), GEO::NO_CELL);
            for (const auto& c : mesh.cells) {
                for (GEO::index_t lf = 0; lf < 4; ++lf)
                    current_connections[4*c+lf] = mesh.cells.adjacent(c, lf);
            }

            mesh.cells.connect();
            for (const auto& c : mesh.cells) {
                for (GEO::index_t lf = 0; lf < 4; ++lf)
                    EXPECT_EQ(current_connections[4*c+lf], mesh.cells.adjacent(c, lf));
            }

            /* Rollback adjacency */
            for (const auto& c : mesh.cells) {
                for (GEO::index_t lf = 0; lf < 4; ++lf)
                    mesh.cells.set_adjacent(c, lf, current_connections[4*c+lf]);
            }
        }

        GEO::Mesh mesh;
        GEO::Mesh original_mesh;
    };

    class TetOperationsAttributeTest : public ::testing::Test {
    protected:
        void SetUp() override {
            mesh_c_idx.bind(mesh.cells.attributes(), "idx");
            mesh_cc_idx.bind(mesh.cell_corners.attributes(), "idx");
            mesh_cf_idx.bind(mesh.cell_facets.attributes(), "idx");
        }

        void create_mesh(
            const std::vector<GEO::vec3>& vertices,
            const std::vector<GEO::index_t>& cells
            ) {
            mesh.vertices.create_vertices(vertices.size());
            for (const auto& v : mesh.vertices)
                mesh.vertices.point(v) = vertices[v];

            mesh.cells.create_tets(cells.size()/4);
            for (const auto& c : mesh.cells) {
                for (GEO::index_t lv = 0; lv < 4; ++lv)
                    mesh.cells.set_vertex(c, lv, cells[4*c+lv]);
            }
            mesh.cells.connect();

            create_attributes();
        }

        void random_attributes(
            const GEO::index_t c
            ) {
            mesh_c_idx[c] = GEO::Numeric::random_int32();
            for (GEO::index_t i = 0; i < 4; ++i) {
                mesh_cc_idx[mesh.cells.corner(c, i)] = GEO::Numeric::random_int32();
                mesh_cf_idx[mesh.cells.facet(c, i)] = GEO::Numeric::random_int32();
            }
        }

        GEO::Mesh mesh;
        GEO::Attribute<GEO::index_t> mesh_c_idx;
        GEO::Attribute<GEO::index_t> mesh_cc_idx;
        GEO::Attribute<GEO::index_t> mesh_cf_idx;
        GEO::vector<GEO::index_t> mesh_c_original_idx;
        GEO::vector<GEO::index_t> mesh_cc_original_idx;
        GEO::vector<GEO::index_t> mesh_cf_original_idx;

        const GEO::index_t DEFAULT_IDX = 0;

    private:
        void create_attributes(
            ) {
            auto& mesh_c_idx_vector = mesh_c_idx.get_vector();
            std::iota(mesh_c_idx_vector.begin(), mesh_c_idx_vector.end(), 1);

            auto& mesh_cc_idx_vector = mesh_cc_idx.get_vector();
            std::iota(mesh_cc_idx_vector.begin(), mesh_cc_idx_vector.end(), 1);

            auto& mesh_cf_idx_vector = mesh_cf_idx.get_vector();
            std::iota(mesh_cf_idx_vector.begin(), mesh_cf_idx_vector.end(), 1);

            mesh_c_original_idx = mesh_c_idx.get_vector();
            mesh_cc_original_idx = mesh_cc_idx.get_vector();
            mesh_cf_original_idx = mesh_cf_idx.get_vector();
        }
    };

    /* ============================================================================================================= */

    class TetSplitTest : public TetOperationsTest {
    protected:
        void perform_operation(const GEO::index_t c) override {
            const GEO::index_t new_v = mesh.vertices.create_vertices(1);
            const GEO::index_t new_c = mesh.cells.create_tets(3);
            tet_split(mesh, c, new_v, new_c, new_c+1, new_c+2);
        }
    };

    TEST_F(TetSplitTest, tet_split) {
        for_each_c();
    }

    class TetSplitSimpleTest : public TetOperationsAttributeTest {};

    TEST_F(TetSplitSimpleTest, manage_attributes) {
        const std::vector<GEO::vec3> vertices = {
            GEO::vec3(0, 0, 2),
            GEO::vec3(0, 2, 0), GEO::vec3(-1.732, -1, 0), GEO::vec3(1.732, -1, 0),
        };
        const std::vector<GEO::index_t> cells = {
            1, 2, 3, 0
        };
        create_mesh(vertices, cells);
        GEO::mesh_save(mesh, get_current_test_name()+"_0.geogram");

        constexpr GEO::index_t c0 = 0;
        const auto cv0 = mesh.cells.vertex(c0, 0);
        const auto cv1 = mesh.cells.vertex(c0, 1);
        const auto cv2 = mesh.cells.vertex(c0, 2);
        const auto cv3 = mesh.cells.vertex(c0, 3);
        const auto cf0_v0 = mesh.cells.facet_vertex(c0, 0, 0);
        const auto cf0_v1 = mesh.cells.facet_vertex(c0, 0, 1);
        const auto cf0_v2 = mesh.cells.facet_vertex(c0, 0, 2);
        const auto cf1_v0 = mesh.cells.facet_vertex(c0, 1, 0);
        const auto cf1_v1 = mesh.cells.facet_vertex(c0, 1, 1);
        const auto cf1_v2 = mesh.cells.facet_vertex(c0, 1, 2);
        const auto cf2_v0 = mesh.cells.facet_vertex(c0, 2, 0);
        const auto cf2_v1 = mesh.cells.facet_vertex(c0, 2, 1);
        const auto cf2_v2 = mesh.cells.facet_vertex(c0, 2, 2);
        const auto cf3_v0 = mesh.cells.facet_vertex(c0, 3, 0);
        const auto cf3_v1 = mesh.cells.facet_vertex(c0, 3, 1);
        const auto cf3_v2 = mesh.cells.facet_vertex(c0, 3, 2);

        /* Split */
        const GEO::index_t new_v = mesh.vertices.create_vertices(1);
        const GEO::index_t new_c0 = mesh.cells.create_tets(3);
        const GEO::index_t new_c1 = new_c0+1;
        const GEO::index_t new_c2 = new_c1+1;
        random_attributes(new_c0);
        random_attributes(new_c1);
        random_attributes(new_c2);
        tet_split(mesh, c0, new_v, new_c0, new_c1, new_c2);
        GEO::mesh_save(mesh, get_current_test_name()+"_1.geogram");

        /* Check */
        {
            EXPECT_EQ(mesh_c_idx[c0],       mesh_c_original_idx[c0]);
            EXPECT_EQ(mesh_c_idx[new_c0],   mesh_c_original_idx[c0]);
            EXPECT_EQ(mesh_c_idx[new_c1],   mesh_c_original_idx[c0]);
            EXPECT_EQ(mesh_c_idx[new_c2],   mesh_c_original_idx[c0]);
        }
        {
            for (const auto& c : {c0, new_c0, new_c1, new_c2}) {
                for (GEO::index_t lv = 0; lv < 4; ++lv) {
                    if (const auto& v = mesh.cells.vertex(c, lv);
                        v == cv0)
                        EXPECT_EQ(mesh_cc_idx[mesh.cells.corner(c, lv)], mesh_cc_original_idx[mesh.cells.corner(c0, 0)]);
                    else if (v == cv1)
                        EXPECT_EQ(mesh_cc_idx[mesh.cells.corner(c, lv)], mesh_cc_original_idx[mesh.cells.corner(c0, 1)]);
                    else if (v == cv2)
                        EXPECT_EQ(mesh_cc_idx[mesh.cells.corner(c, lv)], mesh_cc_original_idx[mesh.cells.corner(c0, 2)]);
                    else if (v == cv3)
                        EXPECT_EQ(mesh_cc_idx[mesh.cells.corner(c, lv)], mesh_cc_original_idx[mesh.cells.corner(c0, 3)]);
                    else
                        EXPECT_EQ(mesh_cc_idx[mesh.cells.corner(c, lv)], DEFAULT_IDX);
                }
            }
        }
        {
            for (const auto& c : {c0, new_c0, new_c1, new_c2}) {
                std::vector<bool> found_lf(4, false);

                if (const auto lf = mesh.cells.find_tet_facet(c, cf0_v0, cf0_v1, cf0_v2);
                    lf != GEO::NO_INDEX) {
                    found_lf[lf] = true;
                    EXPECT_EQ(mesh_cf_idx[mesh.cells.facet(c, lf)], mesh_cf_original_idx[mesh.cells.facet(c0, 0)]);
                }
                if (const auto lf = mesh.cells.find_tet_facet(c, cf1_v0, cf1_v1, cf1_v2);
                    lf != GEO::NO_INDEX) {
                    found_lf[lf] = true;
                    EXPECT_EQ(mesh_cf_idx[mesh.cells.facet(c, lf)], mesh_cf_original_idx[mesh.cells.facet(c0, 1)]);
                }
                if (const auto lf = mesh.cells.find_tet_facet(c, cf2_v0, cf2_v1, cf2_v2);
                    lf != GEO::NO_INDEX) {
                    found_lf[lf] = true;
                    EXPECT_EQ(mesh_cf_idx[mesh.cells.facet(c, lf)], mesh_cf_original_idx[mesh.cells.facet(c0, 2)]);
                }
                if (const auto lf = mesh.cells.find_tet_facet(c, cf3_v0, cf3_v1, cf3_v2);
                    lf != GEO::NO_INDEX) {
                    found_lf[lf] = true;
                    EXPECT_EQ(mesh_cf_idx[mesh.cells.facet(c, lf)], mesh_cf_original_idx[mesh.cells.facet(c0, 3)]);
                }

                for (GEO::index_t lf = 0; lf < 4; ++lf) {
                    if (!found_lf[lf])
                        EXPECT_EQ(mesh_cf_idx[mesh.cells.facet(c, lf)], DEFAULT_IDX);
                }
            }
        }
    }

    /* ============================================================================================================= */

    class TetFacetSplitTest : public TetOperationsTest {
    protected:
        void perform_operation(const GEO::index_t c, const GEO::index_t lf) override {
            const auto CELL_FACET_ON_BORDER = original_mesh.cells.adjacent(c, lf) == GEO::NO_CELL;

            const GEO::index_t new_v = mesh.vertices.create_vertices(1);
            GEO::index_t new_c0 = GEO::NO_CELL;
            GEO::index_t new_c1 = GEO::NO_CELL;
            GEO::index_t new_c2 = GEO::NO_CELL;
            GEO::index_t new_c3 = GEO::NO_CELL;
            if (CELL_FACET_ON_BORDER) {
                new_c0 = mesh.cells.create_tets(2);
                new_c1 = new_c0+1;
            }
            else {
                new_c0 = mesh.cells.create_tets(4);
                new_c1 = new_c0+1;
                new_c2 = new_c0+2;
                new_c3 = new_c0+3;
            }
            tet_facet_split(mesh, c, lf, new_v, new_c0, new_c1, new_c2, new_c3);
        }
    };

    TEST_F(TetFacetSplitTest, tet_facet_split) {
        for_each_c_lf();
    }

    class TetFacetSplitSimpleTest : public TetOperationsAttributeTest {};

    TEST_F(TetFacetSplitSimpleTest, manage_attributes) {
        const std::vector<GEO::vec3> vertices = {
            GEO::vec3(0, 0, 2),
            GEO::vec3(0, 2, 0), GEO::vec3(-1.732, -1, 0), GEO::vec3(1.732, -1, 0),
            GEO::vec3(0, 0, -2)
        };
        const std::vector<GEO::index_t> cells = {
            1, 2, 3, 0,
            3, 2, 1, 4
        };
        create_mesh(vertices, cells);
        GEO::mesh_save(mesh, get_current_test_name()+"_0.geogram");

        constexpr GEO::index_t c0 = 0;
        constexpr GEO::index_t lf0 = 0;
        constexpr GEO::index_t lf1 = 1;
        constexpr GEO::index_t lf2 = 2;
        constexpr GEO::index_t lf3 = 3;
        const auto c0_v0 = mesh.cells.vertex(c0, 0);
        const auto c0_v1 = mesh.cells.vertex(c0, 1);
        const auto c0_v2 = mesh.cells.vertex(c0, 2);
        const auto c0_v3 = mesh.cells.vertex(c0, 3);
        const auto c0_f0_v0 = mesh.cells.facet_vertex(c0, lf0, 0);
        const auto c0_f0_v1 = mesh.cells.facet_vertex(c0, lf0, 1);
        const auto c0_f0_v2 = mesh.cells.facet_vertex(c0, lf0, 2);
        const auto c0_f1_v0 = mesh.cells.facet_vertex(c0, lf1, 0);
        const auto c0_f1_v1 = mesh.cells.facet_vertex(c0, lf1, 1);
        const auto c0_f1_v2 = mesh.cells.facet_vertex(c0, lf1, 2);
        const auto c0_f2_v0 = mesh.cells.facet_vertex(c0, lf2, 0);
        const auto c0_f2_v1 = mesh.cells.facet_vertex(c0, lf2, 1);
        const auto c0_f2_v2 = mesh.cells.facet_vertex(c0, lf2, 2);
        const auto c0_f3_v0 = mesh.cells.facet_vertex(c0, lf3, 0);
        const auto c0_f3_v1 = mesh.cells.facet_vertex(c0, lf3, 1);
        const auto c0_f3_v2 = mesh.cells.facet_vertex(c0, lf3, 2);
        const GEO::index_t c1 = mesh.cells.adjacent(c0, lf3);
        ASSERT_NE(c1, GEO::NO_CELL);
        const auto c1_v0 = mesh.cells.vertex(c1, 0);
        const auto c1_v1 = mesh.cells.vertex(c1, 1);
        const auto c1_v2 = mesh.cells.vertex(c1, 2);
        const auto c1_v3 = mesh.cells.vertex(c1, 3);
        const auto nlf3 = mesh.cells.find_tet_facet(c1, c0_f3_v2, c0_f3_v1, c0_f3_v0);
        const auto nlf0 = (nlf3+1)%4;
        const auto nlf1 = (nlf3+2)%4;
        const auto nlf2 = (nlf3+3)%4;
        ASSERT_NE(nlf3, GEO::NO_INDEX);
        const auto c1_f0_v0 = mesh.cells.facet_vertex(c1, nlf0, 0);
        const auto c1_f0_v1 = mesh.cells.facet_vertex(c1, nlf0, 1);
        const auto c1_f0_v2 = mesh.cells.facet_vertex(c1, nlf0, 2);
        const auto c1_f1_v0 = mesh.cells.facet_vertex(c1, nlf1, 0);
        const auto c1_f1_v1 = mesh.cells.facet_vertex(c1, nlf1, 1);
        const auto c1_f1_v2 = mesh.cells.facet_vertex(c1, nlf1, 2);
        const auto c1_f2_v0 = mesh.cells.facet_vertex(c1, nlf2, 0);
        const auto c1_f2_v1 = mesh.cells.facet_vertex(c1, nlf2, 1);
        const auto c1_f2_v2 = mesh.cells.facet_vertex(c1, nlf2, 2);
        const auto c1_f3_v0 = mesh.cells.facet_vertex(c1, nlf3, 0);
        const auto c1_f3_v1 = mesh.cells.facet_vertex(c1, nlf3, 1);
        const auto c1_f3_v2 = mesh.cells.facet_vertex(c1, nlf3, 2);

        /* Split */
        const GEO::index_t new_v = mesh.vertices.create_vertices(1);
        const GEO::index_t new_c0 = mesh.cells.create_tets(4);
        const GEO::index_t new_c1 = new_c0+1;
        const GEO::index_t new_c2 = new_c1+1;
        const GEO::index_t new_c3 = new_c2+1;
        random_attributes(new_c0);
        random_attributes(new_c1);
        random_attributes(new_c2);
        random_attributes(new_c3);
        tet_facet_split(mesh, c0, lf3, new_v, new_c0, new_c1, new_c2, new_c3);
        GEO::mesh_save(mesh, get_current_test_name()+"_1.geogram");

        /* Check */
        {
            EXPECT_EQ(mesh_c_idx[c0],       mesh_c_original_idx[c0]);
            EXPECT_EQ(mesh_c_idx[new_c0],   mesh_c_original_idx[c0]);
            EXPECT_EQ(mesh_c_idx[new_c1],   mesh_c_original_idx[c0]);
            EXPECT_EQ(mesh_c_idx[c1],       mesh_c_original_idx[c1]);
            EXPECT_EQ(mesh_c_idx[new_c2],   mesh_c_original_idx[c1]);
            EXPECT_EQ(mesh_c_idx[new_c3],   mesh_c_original_idx[c1]);
        }
        {
            for (const auto& c : {c0, new_c0, new_c1}) {
                for (GEO::index_t lv = 0; lv < 4; ++lv) {
                    if (const auto& v = mesh.cells.vertex(c, lv);
                        v == c0_v0)
                        EXPECT_EQ(mesh_cc_idx[mesh.cells.corner(c, lv)], mesh_cc_original_idx[mesh.cells.corner(c0, 0)]);
                    else if (v == c0_v1)
                        EXPECT_EQ(mesh_cc_idx[mesh.cells.corner(c, lv)], mesh_cc_original_idx[mesh.cells.corner(c0, 1)]);
                    else if (v == c0_v2)
                        EXPECT_EQ(mesh_cc_idx[mesh.cells.corner(c, lv)], mesh_cc_original_idx[mesh.cells.corner(c0, 2)]);
                    else if (v == c0_v3)
                        EXPECT_EQ(mesh_cc_idx[mesh.cells.corner(c, lv)], mesh_cc_original_idx[mesh.cells.corner(c0, 3)]);
                    else
                        EXPECT_EQ(mesh_cc_idx[mesh.cells.corner(c, lv)], DEFAULT_IDX);
                }
            }
            for (const auto& c : {c1, new_c2, new_c3}) {
                for (GEO::index_t lv = 0; lv < 4; ++lv) {
                    if (const auto& v = mesh.cells.vertex(c, lv);
                        v == c1_v0)
                        EXPECT_EQ(mesh_cc_idx[mesh.cells.corner(c, lv)], mesh_cc_original_idx[mesh.cells.corner(c1, 0)]);
                    else if (v == c1_v1)
                        EXPECT_EQ(mesh_cc_idx[mesh.cells.corner(c, lv)], mesh_cc_original_idx[mesh.cells.corner(c1, 1)]);
                    else if (v == c1_v2)
                        EXPECT_EQ(mesh_cc_idx[mesh.cells.corner(c, lv)], mesh_cc_original_idx[mesh.cells.corner(c1, 2)]);
                    else if (v == c1_v3)
                        EXPECT_EQ(mesh_cc_idx[mesh.cells.corner(c, lv)], mesh_cc_original_idx[mesh.cells.corner(c1, 3)]);
                    else
                        EXPECT_EQ(mesh_cc_idx[mesh.cells.corner(c, lv)], DEFAULT_IDX);
                }
            }
        }
        {
            for (const auto& c : {c0, new_c0, new_c1}) {
                std::vector<bool> found_lf(4, false);

                if (const auto lf = mesh.cells.find_tet_facet(c, c0_f0_v0, c0_f0_v1, c0_f0_v2);
                    lf != GEO::NO_INDEX) {
                    found_lf[lf] = true;
                    EXPECT_EQ(mesh_cf_idx[mesh.cells.facet(c, lf)], mesh_cf_original_idx[mesh.cells.facet(c0, lf0)]);
                }
                if (const auto lf = mesh.cells.find_tet_facet(c, c0_f1_v0, c0_f1_v1, c0_f1_v2);
                    lf != GEO::NO_INDEX) {
                    found_lf[lf] = true;
                    EXPECT_EQ(mesh_cf_idx[mesh.cells.facet(c, lf)], mesh_cf_original_idx[mesh.cells.facet(c0, lf1)]);
                }
                if (const auto lf = mesh.cells.find_tet_facet(c, c0_f2_v0, c0_f2_v1, c0_f2_v2);
                    lf != GEO::NO_INDEX) {
                    found_lf[lf] = true;
                    EXPECT_EQ(mesh_cf_idx[mesh.cells.facet(c, lf)], mesh_cf_original_idx[mesh.cells.facet(c0, lf2)]);
                }

                for (GEO::index_t lf = 0; lf < 4; ++lf) {
                    GEO::index_t cnt = 0;
                    for (GEO::index_t lv = 0; lv < 3; ++lv) {
                        if (const auto& v = mesh.cells.facet_vertex(c, lf, lv);
                            v == c0_f3_v0 || v == c0_f3_v1 || v == c0_f3_v2 || v == new_v)
                            ++cnt;
                    }
                    if (cnt == 3) {
                        found_lf[lf] = true;
                        EXPECT_EQ(mesh_cf_idx[mesh.cells.facet(c, lf)], mesh_cf_original_idx[mesh.cells.facet(c0, lf3)]);
                    }
                }

                for (GEO::index_t lf = 0; lf < 4; ++lf) {
                    if (!found_lf[lf])
                        EXPECT_EQ(mesh_cf_idx[mesh.cells.facet(c, lf)], DEFAULT_IDX);
                }
            }
            
            for (const auto& c : {c1, new_c2, new_c3}) {
                std::vector<bool> found_lf(4, false);

                if (const auto lf = mesh.cells.find_tet_facet(c, c1_f0_v0, c1_f0_v1, c1_f0_v2);
                    lf != GEO::NO_INDEX) {
                    found_lf[lf] = true;
                    EXPECT_EQ(mesh_cf_idx[mesh.cells.facet(c, lf)], mesh_cf_original_idx[mesh.cells.facet(c1, lf0)]);
                }
                if (const auto lf = mesh.cells.find_tet_facet(c, c1_f1_v0, c1_f1_v1, c1_f1_v2);
                    lf != GEO::NO_INDEX) {
                    found_lf[lf] = true;
                    EXPECT_EQ(mesh_cf_idx[mesh.cells.facet(c, lf)], mesh_cf_original_idx[mesh.cells.facet(c1, lf1)]);
                }
                if (const auto lf = mesh.cells.find_tet_facet(c, c1_f2_v0, c1_f2_v1, c1_f2_v2);
                    lf != GEO::NO_INDEX) {
                    found_lf[lf] = true;
                    EXPECT_EQ(mesh_cf_idx[mesh.cells.facet(c, lf)], mesh_cf_original_idx[mesh.cells.facet(c1, lf2)]);
                }

                for (GEO::index_t lf = 0; lf < 4; ++lf) {
                    GEO::index_t cnt = 0;
                    for (GEO::index_t lv = 0; lv < 3; ++lv) {
                        if (const auto& v = mesh.cells.facet_vertex(c, lf, lv);
                            v == c1_f3_v0 || v == c1_f3_v1 || v == c1_f3_v2 || v == new_v)
                            ++cnt;
                    }
                    if (cnt == 3) {
                        found_lf[lf] = true;
                        EXPECT_EQ(mesh_cf_idx[mesh.cells.facet(c, lf)], mesh_cf_original_idx[mesh.cells.facet(c1, lf3)]);
                    }
                }

                for (GEO::index_t lf = 0; lf < 4; ++lf) {
                    if (!found_lf[lf])
                        EXPECT_EQ(mesh_cf_idx[mesh.cells.facet(c, lf)], DEFAULT_IDX);
                }
            }
        }
    }

    /* ============================================================================================================= */

    class TetEdgeSplitTest : public TetOperationsTest {
    protected:
        void perform_operation(const GEO::index_t c, const GEO::index_t le) override {
            std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>> ordered_c_le_lf;
            get_edge_incident_cells(mesh, c, le, ordered_c_le_lf);

            const GEO::index_t EDGE_INCIDENT_TETS_NB = ordered_c_le_lf.size();

            const GEO::index_t new_v = mesh.vertices.create_vertices(1);
            const GEO::index_t new_c = mesh.cells.create_tets(EDGE_INCIDENT_TETS_NB);
            std::vector<GEO::index_t> new_cells(EDGE_INCIDENT_TETS_NB);
            std::iota(new_cells.begin(), new_cells.end(), new_c);

            tet_edge_split(mesh, ordered_c_le_lf, new_v, new_cells);
        }
    };

    TEST_F(TetEdgeSplitTest, tet_edge_split) {
        for_each_c_le();
    }

    class TetEdgeSplitSimpleTest : public TetOperationsAttributeTest {};

    TEST_F(TetEdgeSplitSimpleTest, manage_attributes) {
        const std::vector<GEO::vec3> vertices = {
            GEO::vec3(0, 2, 0), GEO::vec3(-1.732, -1, 0), GEO::vec3(1.732, -1, 0),
            GEO::vec3(0, 0, 2), GEO::vec3(0, 0, -2)
        };
        const std::vector<GEO::index_t> cells = {
            4, 0, 1, 3,
            4, 1, 2, 3,
            4, 2, 0, 3
        };
        create_mesh(vertices, cells);
        GEO::mesh_save(mesh, get_current_test_name()+"_0.geogram");

        constexpr GEO::index_t c0 = 0;
        constexpr GEO::index_t le0 = 5;
        std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>> ordered_c_le_lf;
        {
            get_edge_incident_cells(mesh, c0, le0, ordered_c_le_lf);
        }
        std::vector<GEO::index_t> cell_vertices;
        {
            cell_vertices.reserve(ordered_c_le_lf.size()*4);
            for (const auto& [c, _, __] : ordered_c_le_lf) {
                for (GEO::index_t lv = 0; lv < 4; ++lv)
                    cell_vertices.push_back(mesh.cells.vertex(c, lv));
            }
        }
        std::vector<GEO::index_t> cell_facet_vertices;
        std::vector<GEO::vec3> cell_facet_normals;
        {
            cell_facet_vertices.reserve(ordered_c_le_lf.size()*12);
            cell_facet_normals.reserve(ordered_c_le_lf.size()*4);
            for (const auto& [c, _, __] : ordered_c_le_lf) {
                for (GEO::index_t lf = 0; lf < 4; ++lf) {
                    for (GEO::index_t lv = 0; lv < 3; ++lv)
                        cell_facet_vertices.push_back(mesh.cells.facet_vertex(c, lf, lv));
                    cell_facet_normals.push_back(GEO::triangle_normal<GEO::vec3>(
                        mesh.vertices.point(mesh.cells.facet_vertex(c, lf, 2)),
                        mesh.vertices.point(mesh.cells.facet_vertex(c, lf, 1)),
                        mesh.vertices.point(mesh.cells.facet_vertex(c, lf, 0))
                        ));
                }
            }
        }

        /* Split */
        const GEO::index_t new_v = mesh.vertices.create_vertices(1);
        std::vector<GEO::index_t> new_cs(ordered_c_le_lf.size());
        {
            const GEO::index_t new_c = mesh.cells.create_tets(ordered_c_le_lf.size());
            std::iota(new_cs.begin(), new_cs.end(), new_c);
        }
        for (const auto& c : new_cs)
            random_attributes(c);

        tet_edge_split(mesh, ordered_c_le_lf, new_v, new_cs);
        GEO::mesh_save(mesh, get_current_test_name()+"_1.geogram");

        /* Check */
        for (GEO::index_t i = 0, i_end = ordered_c_le_lf.size(); i < i_end; ++i) {
            const auto& cur_c = get<0>(ordered_c_le_lf[i]);
            const auto& new_c = new_cs[i];
            {
                EXPECT_EQ(mesh_c_idx[cur_c],        mesh_c_original_idx[cur_c]);
                EXPECT_EQ(mesh_c_idx[new_c],    mesh_c_original_idx[cur_c]);
            }
            {
                for (const auto& c : {cur_c, new_c}) {
                    for (GEO::index_t lv = 0; lv < 4; ++lv) {
                        if (const auto& v = mesh.cells.vertex(c, lv);
                            v == cell_vertices[4*cur_c+0])
                            EXPECT_EQ(mesh_cc_idx[mesh.cells.corner(c, lv)], mesh_cc_original_idx[mesh.cells.corner(cur_c, 0)]);
                        else if (v == cell_vertices[4*cur_c+1])
                            EXPECT_EQ(mesh_cc_idx[mesh.cells.corner(c, lv)], mesh_cc_original_idx[mesh.cells.corner(cur_c, 1)]);
                        else if (v == cell_vertices[4*cur_c+2])
                            EXPECT_EQ(mesh_cc_idx[mesh.cells.corner(c, lv)], mesh_cc_original_idx[mesh.cells.corner(cur_c, 2)]);
                        else if (v == cell_vertices[4*cur_c+3])
                            EXPECT_EQ(mesh_cc_idx[mesh.cells.corner(c, lv)], mesh_cc_original_idx[mesh.cells.corner(cur_c, 3)]);
                        else
                            EXPECT_EQ(mesh_cc_idx[mesh.cells.corner(c, lv)], DEFAULT_IDX);
                    }
                }
            }
            {
                for (const auto& c : {cur_c, new_c}) {
                    std::vector<bool> found_lf(4, false);

                    for (GEO::index_t lf = 0; lf < 4; ++lf) {
                        const auto lf_nor = cell_facet_normals[4*cur_c+lf];

                        for (GEO::index_t flf = 0; flf < 4; ++flf) {
                            const auto flf_nor = GEO::triangle_normal<GEO::vec3>(
                                mesh.vertices.point(mesh.cells.facet_vertex(c, flf, 2)),
                                mesh.vertices.point(mesh.cells.facet_vertex(c, flf, 1)),
                                mesh.vertices.point(mesh.cells.facet_vertex(c, flf, 0)));

                            if (GEO::Geom::angle(lf_nor, flf_nor) > 0.01)
                                continue;

                            GEO::index_t cnt = 0;
                            for (GEO::index_t lv = 0; lv < 3; ++lv) {
                                if (const auto& v = mesh.cells.facet_vertex(c, flf, lv);
                                    v == cell_facet_vertices[12*cur_c+lf*3+0] ||
                                    v == cell_facet_vertices[12*cur_c+lf*3+1] ||
                                    v == cell_facet_vertices[12*cur_c+lf*3+2] ||
                                    v == new_v)
                                    ++cnt;
                            }
                            if (cnt == 3) {
                                found_lf[flf] = true;
                                EXPECT_EQ(mesh_cf_idx[mesh.cells.facet(c, flf)], mesh_cf_original_idx[mesh.cells.facet(cur_c, lf)]);
                            }
                        }
                    }

                    for (GEO::index_t lf = 0; lf < 4; ++lf) {
                        if (!found_lf[lf])
                            EXPECT_EQ(mesh_cf_idx[mesh.cells.facet(c, lf)], DEFAULT_IDX);
                    }
                }
            }
        }
    }

    /* ============================================================================================================= */

    class TetEdgeCollapseTest : public TetOperationsTest {
    protected:
        void perform_operation(const GEO::index_t c, const GEO::index_t le) override {
            LOG::TRACE("{}(c: {}, le: {})", __FUNCTION__, c, le);

            if (!is_tet_edge_collapse_valid(mesh, c, le))
                return;

            ++performed_operations_nb;
            GEO::index_t disuse_v;
            std::vector<GEO::index_t> disuse_cs;
            tet_edge_collapse(mesh, c, le, disuse_v, disuse_cs);

            /* Clean disuse vertices and cells */
            GEO::vector<GEO::index_t> cells_to_delete(mesh.cells.nb(), 0);
            for (const auto& cc : disuse_cs)
                cells_to_delete[cc] = 1;
            mesh.cells.delete_elements(cells_to_delete);
        }
    };

    TEST_F(TetEdgeCollapseTest, tet_edge_collapse) {
        for_each_c_le();
        EXPECT_GT(performed_operations_nb, 0) << "the collapse validity check rejected every edge";
    }

    /* ============================================================================================================= */

    class TetEdgeSwap23Test : public TetOperationsTest {
    protected:
        void perform_operation(const GEO::index_t c, const GEO::index_t lf) override {
            LOG::TRACE("{}(c: {}, lf: {})", __FUNCTION__, c, lf);

            if (!is_tet_edge_swap_2_3_valid(mesh, c, lf))
                return;

            ++performed_operations_nb;
            const GEO::index_t new_c = mesh.cells.create_tets(1);
            tet_edge_swap_2_3(mesh, c, lf, new_c);
        }
    };

    TEST_F(TetEdgeSwap23Test, tet_edge_swap) {
        for_each_c_lf();
        EXPECT_GT(performed_operations_nb, 0) << "the swap validity check rejected every facet";
    }

    class TetEdgeSwap23SimpleTest : public TetOperationsAttributeTest {};

    TEST_F(TetEdgeSwap23SimpleTest, manage_attributes) {
        const std::vector<GEO::vec3> vertices = {
            GEO::vec3(0, 2, 0), GEO::vec3(-1.732, -1, 0), GEO::vec3(1.732, -1, 0),
            GEO::vec3(0, 0, 2), GEO::vec3(0, 0, -2)
        };
        const std::vector<GEO::index_t> cells = {
            0, 1, 2, 3,
            2, 1, 0, 4
        };
        create_mesh(vertices, cells);
        GEO::mesh_save(mesh, get_current_test_name()+"_0.geogram");

        constexpr GEO::index_t c0 = 0;
        constexpr GEO::index_t lf0 = 0;
        constexpr GEO::index_t lf1 = 1;
        constexpr GEO::index_t lf2 = 2;
        constexpr GEO::index_t lf3 = 3;
        const auto c0v = mesh.cells.vertex(c0, lf3);
        const auto c0_f0_v0 = mesh.cells.facet_vertex(c0, lf0, 0);
        const auto c0_f0_v1 = mesh.cells.facet_vertex(c0, lf0, 1);
        const auto c0_f0_v2 = mesh.cells.facet_vertex(c0, lf0, 2);
        const auto c0_f1_v0 = mesh.cells.facet_vertex(c0, lf1, 0);
        const auto c0_f1_v1 = mesh.cells.facet_vertex(c0, lf1, 1);
        const auto c0_f1_v2 = mesh.cells.facet_vertex(c0, lf1, 2);
        const auto c0_f2_v0 = mesh.cells.facet_vertex(c0, lf2, 0);
        const auto c0_f2_v1 = mesh.cells.facet_vertex(c0, lf2, 1);
        const auto c0_f2_v2 = mesh.cells.facet_vertex(c0, lf2, 2);
        const auto c1 = mesh.cells.adjacent(c0, lf3);
        ASSERT_NE(c1, GEO::NO_CELL);
        const auto nlf3 = mesh.cells.find_tet_facet(c1, mesh.cells.facet_vertex(c0, lf3, 2), mesh.cells.facet_vertex(c0, lf3, 1), mesh.cells.facet_vertex(c0, lf3, 0));
        const auto nlf0 = (nlf3+1)%4;
        const auto nlf1 = (nlf3+2)%4;
        const auto nlf2 = (nlf3+3)%4;
        const auto c1v = mesh.cells.vertex(c1, nlf3);
        const auto c1_f0_v0 = mesh.cells.facet_vertex(c1, nlf0, 0);
        const auto c1_f0_v1 = mesh.cells.facet_vertex(c1, nlf0, 1);
        const auto c1_f0_v2 = mesh.cells.facet_vertex(c1, nlf0, 2);
        const auto c1_f1_v0 = mesh.cells.facet_vertex(c1, nlf1, 0);
        const auto c1_f1_v1 = mesh.cells.facet_vertex(c1, nlf1, 1);
        const auto c1_f1_v2 = mesh.cells.facet_vertex(c1, nlf1, 2);
        const auto c1_f2_v0 = mesh.cells.facet_vertex(c1, nlf2, 0);
        const auto c1_f2_v1 = mesh.cells.facet_vertex(c1, nlf2, 1);
        const auto c1_f2_v2 = mesh.cells.facet_vertex(c1, nlf2, 2);

        /* Swap */
        const GEO::index_t new_c = mesh.cells.create_tets(1);
        random_attributes(new_c);
        tet_edge_swap_2_3(mesh, c0, lf3, new_c);
        GEO::mesh_save(mesh, get_current_test_name()+"_1.geogram");

        /* Check */
        {
            EXPECT_EQ(mesh_c_idx[c0], DEFAULT_IDX);
            EXPECT_EQ(mesh_c_idx[c1], DEFAULT_IDX);
            EXPECT_EQ(mesh_c_idx[new_c], DEFAULT_IDX);
        }
        {
            for (const auto& c : {c0, c1, new_c}) {
                std::vector<bool> found_lv(4, false);

                for (GEO::index_t lv = 0; lv < 4; ++lv) {
                    if (const auto& v = mesh.cells.vertex(c, lv);
                        v == c0v) {
                        found_lv[lv] = true;
                        EXPECT_EQ(mesh_cc_idx[mesh.cells.corner(c, lv)], mesh_cc_original_idx[mesh.cells.corner(c0, lf3)]);
                    }
                    else if (v == c1v) {
                        found_lv[lv] = true;
                        EXPECT_EQ(mesh_cc_idx[mesh.cells.corner(c, lv)], mesh_cc_original_idx[mesh.cells.corner(c1, nlf3)]);
                    }
                }

                for (GEO::index_t lv = 0; lv < 4; ++lv) {
                    if (!found_lv[lv])
                        EXPECT_EQ(mesh_cf_idx[mesh.cells.corner(c, lv)], DEFAULT_IDX);
                }
            }
        }
        {
            for (const auto& c : {c0, c1, new_c}) {
                std::vector<bool> found_lf(4, false);

                if (const auto lf = mesh.cells.find_tet_facet(c, c0_f0_v0, c0_f0_v1, c0_f0_v2);
                    lf != GEO::NO_INDEX) {
                    found_lf[lf] = true;
                    EXPECT_EQ(mesh_cf_idx[mesh.cells.facet(c, lf)], mesh_cf_original_idx[mesh.cells.facet(c0, lf0)]);
                }
                if (const auto lf = mesh.cells.find_tet_facet(c, c0_f1_v0, c0_f1_v1, c0_f1_v2);
                    lf != GEO::NO_INDEX) {
                    found_lf[lf] = true;
                    EXPECT_EQ(mesh_cf_idx[mesh.cells.facet(c, lf)], mesh_cf_original_idx[mesh.cells.facet(c0, lf1)]);
                }
                if (const auto lf = mesh.cells.find_tet_facet(c, c0_f2_v0, c0_f2_v1, c0_f2_v2);
                    lf != GEO::NO_INDEX) {
                    found_lf[lf] = true;
                    EXPECT_EQ(mesh_cf_idx[mesh.cells.facet(c, lf)], mesh_cf_original_idx[mesh.cells.facet(c0, lf2)]);
                }
                if (const auto lf = mesh.cells.find_tet_facet(c, c1_f0_v0, c1_f0_v1, c1_f0_v2);
                    lf != GEO::NO_INDEX) {
                    found_lf[lf] = true;
                    EXPECT_EQ(mesh_cf_idx[mesh.cells.facet(c, lf)], mesh_cf_original_idx[mesh.cells.facet(c1, lf0)]);
                }
                if (const auto lf = mesh.cells.find_tet_facet(c, c1_f1_v0, c1_f1_v1, c1_f1_v2);
                    lf != GEO::NO_INDEX) {
                    found_lf[lf] = true;
                    EXPECT_EQ(mesh_cf_idx[mesh.cells.facet(c, lf)], mesh_cf_original_idx[mesh.cells.facet(c1, lf1)]);
                }
                if (const auto lf = mesh.cells.find_tet_facet(c, c1_f2_v0, c1_f2_v1, c1_f2_v2);
                    lf != GEO::NO_INDEX) {
                    found_lf[lf] = true;
                    EXPECT_EQ(mesh_cf_idx[mesh.cells.facet(c, lf)], mesh_cf_original_idx[mesh.cells.facet(c1, lf2)]);
                }

                for (GEO::index_t lf = 0; lf < 4; ++lf) {
                    if (!found_lf[lf])
                        EXPECT_EQ(mesh_cf_idx[mesh.cells.facet(c, lf)], DEFAULT_IDX);
                }
            }
        }
    }

    /* ============================================================================================================= */

    class TetEdgeSwap32Test : public TetOperationsTest {
    protected:
        void perform_operation(const GEO::index_t c, const GEO::index_t le) override {
            LOG::TRACE("{}(c: {}, le: {})", __FUNCTION__, c, le);

            std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>> ordered_c_le_lf;
            get_edge_incident_cells(mesh, c, le, ordered_c_le_lf);

            GEO::index_t disuse_c = GEO::NO_CELL;
            if (!tet_edge_swap_3_2(mesh, ordered_c_le_lf, disuse_c))
                return;
            ++performed_operations_nb;

            /* Clean disuse vertices and cells */
            ASSERT_LT(disuse_c, mesh.cells.nb());
            GEO::vector<GEO::index_t> cells_to_delete(mesh.cells.nb(), 0);
            cells_to_delete[disuse_c] = 1;
            mesh.cells.delete_elements(cells_to_delete);
        }
    };

    TEST_F(TetEdgeSwap32Test, tet_edge_swap) {
        for_each_c_le();
        EXPECT_GT(performed_operations_nb, 0) << "the 3-2 swap was rejected for every edge";
    }

    class TetEdgeSwap32AttributeTest : public TetOperationsAttributeTest {};

    TEST_F(TetEdgeSwap32AttributeTest, manage_attributes) {
        const std::vector<GEO::vec3> vertices = {
            GEO::vec3(0, 2, 0), GEO::vec3(-1.732, -1, 0), GEO::vec3(1.732, -1, 0),
            GEO::vec3(0, 0, 2), GEO::vec3(0, 0, -2)
        };
        const std::vector<GEO::index_t> cells = {
            0, 4, 3, 1,
            1, 4, 3, 2,
            2, 4, 3, 0
        };
        create_mesh(vertices, cells);
        GEO::mesh_save(mesh, get_current_test_name()+"_0.geogram");

        constexpr GEO::index_t c0 = 0;
        constexpr GEO::index_t le0 = 0;

        std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>> ordered_c_le_lf;
        const bool ON_BORDER = get_edge_incident_cells(mesh, c0, le0, ordered_c_le_lf);
        ASSERT_FALSE(ON_BORDER);
        ASSERT_EQ(ordered_c_le_lf.size(), 3);

        const auto c1 = get<0>(ordered_c_le_lf[1]);

        std::vector<GEO::index_t> cell_facet_vertices;
        {
            cell_facet_vertices.reserve(12*ordered_c_le_lf.size());
            for (const auto& [c, _, __] : ordered_c_le_lf) {
                for (GEO::index_t lf = 0; lf < 4; ++lf) {
                    for (GEO::index_t lv = 0; lv < 3; ++lv)
                        cell_facet_vertices.push_back(mesh.cells.facet_vertex(c, lf, lv));
                }
            }
        }

        /* Split */
        GEO::index_t disuse_c;
        tet_edge_swap_3_2(mesh, ordered_c_le_lf, disuse_c);

        /* Check */
        {
            EXPECT_EQ(mesh_c_idx[c0], DEFAULT_IDX);
            EXPECT_EQ(mesh_c_idx[c1], DEFAULT_IDX);
        }
        {
            for (const auto& c : {c0, c1}) {
                for (GEO::index_t lv = 0; lv < 4; ++lv)
                    EXPECT_EQ(mesh_cc_idx[mesh.cells.corner(c, lv)], DEFAULT_IDX);
            }
        }
        {
            for (const auto& c : {c0, c1}) {
                std::vector<bool> found_lf(4, false);

                for (const auto& [cc, _, __] : ordered_c_le_lf) {
                    for (GEO::index_t lf = 0; lf < 4; ++lf) {
                        if (const auto flf = mesh.cells.find_tet_facet(c, cell_facet_vertices[12*cc+3*lf+0], cell_facet_vertices[12*cc+3*lf+1], cell_facet_vertices[12*cc+3*lf+2]);
                            flf != GEO::NO_INDEX) {
                            found_lf[flf] = true;
                            EXPECT_EQ(mesh_cf_idx[mesh.cells.facet(c, flf)], mesh_cf_original_idx[mesh.cells.facet(cc, lf)]);
                        }
                    }
                }

                for (GEO::index_t lf = 0; lf < 4; ++lf) {
                    if (!found_lf[lf])
                        EXPECT_EQ(mesh_cf_idx[mesh.cells.facet(c, lf)], DEFAULT_IDX);
                }
            }
        }


        GEO::vector<GEO::index_t> cells_to_delete(mesh.cells.nb(), 0);
        cells_to_delete[disuse_c] = 1;
        mesh.cells.delete_elements(cells_to_delete);

        GEO::mesh_save(mesh, get_current_test_name()+"_1.geogram");
    }
}
