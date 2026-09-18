//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/3/13.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <array>
#include <numbers>
#include <random>
#include <ranges>
#include <vector>
#include <geogram/mesh/mesh_io.h>
#include <gtest/gtest.h>
#include "../utils.h"
#include <geolio/mesh/tri_operations.h>
#include <geolio/common/log.h>

namespace geolio::test
{
    class TriOperationsTest : public ::testing::Test {
    protected:
        void SetUp() override {
            generate_random_CDT2d_mesh(mesh, 20, 10);

            original_mesh.copy(mesh);
        }

        void for_each_f_lv() {
            for (const auto& f : original_mesh.facets) {
                for (GEO::index_t lv = 0, lv_end = original_mesh.facets.nb_vertices(f); lv < lv_end; ++lv) {
                    LOG::TRACE("f: {}/{}, lv: {}/{}", f, original_mesh.facets.nb(), lv, lv_end);

                    mesh.copy(original_mesh);

                    /* Compute */
                    perform_operation(f, lv);

                    /* Eval */
                    check_connections();
                }
            }
        }

        virtual void perform_operation(
            GEO::index_t f,
            GEO::index_t lv) = 0;

        /**
         * Verify that reconnecting the mesh preserves the current adjacency layout.
         */
        void check_connections() {
            std::vector<GEO::index_t> current_connections(3*mesh.facets.nb(), GEO::NO_FACET);
            for (const auto& f : mesh.facets) {
                for (GEO::index_t lv = 0; lv < 3; ++lv)
                    current_connections[3*f+lv] = mesh.facets.adjacent(f, lv);
            }

            mesh.facets.connect();
            GEO::Attribute<bool> mesh_fc_adj_error(mesh.facet_corners.attributes(), "adj_error");
            mesh_fc_adj_error.fill(false);
            for (const auto& f : mesh.facets) {
                for (GEO::index_t lv = 0; lv < 3; ++lv) {
                    EXPECT_EQ(current_connections[3*f+lv], mesh.facets.adjacent(f, lv));
                    mesh_fc_adj_error[mesh.facets.corner(f, lv)] = current_connections[3*f+lv] != mesh.facets.adjacent(f, lv);
                }
            }

            /* Rollback adjacency */
            for (const auto& f : mesh.facets) {
                for (GEO::index_t lv = 0; lv < 3; ++lv)
                    mesh.facets.set_adjacent(f, lv, current_connections[3*f+lv]);
            }
        }

        GEO::Mesh mesh;
        GEO::Mesh original_mesh;
    };

    class TriOperationsSimpleTest : public ::testing::Test {
    protected:
        void SetUp() override {
            mesh_f_idx.bind(mesh.facets.attributes(), "idx");
            mesh_fc_idx.bind(mesh.facet_corners.attributes(), "idx");
        }

        void create_mesh(
            const std::vector<GEO::vec3>& vertices,
            const std::vector<GEO::index_t>& facets
            ) {
            mesh.vertices.create_vertices(vertices.size());
            for (const auto& v : mesh.vertices)
                mesh.vertices.point(v) = vertices[v];

            mesh.facets.create_triangles(facets.size()/3);
            for (const auto& f : mesh.facets) {
                for (GEO::index_t lv = 0; lv < 3; ++lv)
                    mesh.facets.set_vertex(f, lv, facets[3*f+lv]);
            }
            mesh.facets.connect();

            create_attributes();
        }

        void random_attributes(
            const GEO::index_t f
            ) {
            mesh_f_idx[f] = GEO::Numeric::random_int32();
            for (GEO::index_t i = 0; i < 3; ++i)
                mesh_fc_idx[mesh.facets.corner(f, i)] = GEO::Numeric::random_int32();
        }

        /**
         * Verify that the adjacency stored in the mesh is the one a fresh connectivity pass builds.
         *
         * @details The operations maintain the facet-to-facet adjacency incrementally; this check
         *          states that the incremental result is the one mesh.facets.connect() recomputes.
         */
        void expect_adjacency_consistent() {
            std::vector<GEO::index_t> stored_adjacency(3*mesh.facets.nb(), GEO::NO_FACET);
            for (const auto& f : mesh.facets)
                for (GEO::index_t lv = 0; lv < 3; ++lv)
                    stored_adjacency[3*f+lv] = mesh.facets.adjacent(f, lv);

            mesh.facets.connect();
            for (const auto& f : mesh.facets) {
                for (GEO::index_t lv = 0; lv < 3; ++lv) {
                    EXPECT_EQ(stored_adjacency[3*f+lv], mesh.facets.adjacent(f, lv))
                        << "facet " << f << ", local edge " << lv;
                }
            }
        }

        GEO::Mesh mesh;
        GEO::Attribute<GEO::index_t> mesh_f_idx;
        GEO::Attribute<GEO::index_t> mesh_fc_idx;
        GEO::vector<GEO::index_t> mesh_f_original_idx;
        GEO::vector<GEO::index_t> mesh_fc_original_idx;

        const GEO::index_t DEFAULT_IDX = 0;

    private:
        void create_attributes(
            ) {
            auto& mesh_f_idx_vector = mesh_f_idx.get_vector();
            std::iota(mesh_f_idx_vector.begin(), mesh_f_idx_vector.end(), 1);

            auto& mesh_fc_idx_vector = mesh_fc_idx.get_vector();
            std::iota(mesh_fc_idx_vector.begin(), mesh_fc_idx_vector.end(), 1);

            mesh_f_original_idx = mesh_f_idx.get_vector();
            mesh_fc_original_idx = mesh_fc_idx.get_vector();
        }
    };

    /* ============================================================================================================= */

    class TriEdgeSplitTest : public TriOperationsTest {
    protected:
        void perform_operation(
            const GEO::index_t f,
            const GEO::index_t lv
            ) override {
            const bool EDGE_ON_BORDER = original_mesh.facets.adjacent(f, lv) == GEO::NO_FACET;

            const GEO::index_t new_v = mesh.vertices.create_vertices(1);
            GEO::index_t new_f0 = GEO::NO_FACET;
            GEO::index_t new_f1 = GEO::NO_FACET;
            if (EDGE_ON_BORDER)
                new_f0 = mesh.facets.create_triangles(1);
            else {
                new_f0 = mesh.facets.create_triangles(2);
                new_f1 = new_f0+1;
            }
            tri_edge_split<3>(mesh, f, lv, new_v, new_f0, new_f1);
        }
    };

    TEST_F(TriEdgeSplitTest, tri_edge_split) {
        for_each_f_lv();
    }

    class TriEdgeSplitSimpleTest : public TriOperationsSimpleTest {};

    TEST_F(TriEdgeSplitSimpleTest, manage_attributes) {
        const std::vector<GEO::vec3> vertices = {
            GEO::vec3(0, 2, 0), GEO::vec3(1, 2, 0), GEO::vec3(2, 2, 0),
            GEO::vec3(0, 1, 0), GEO::vec3(2, 1, 0),
            GEO::vec3(0, 0, 0), GEO::vec3(1, 0, 0), GEO::vec3(2, 0, 0),
        };
        const std::vector<GEO::index_t> facets = {
            0, 3, 1,
            1, 4, 2, 1, 3, 6, 1, 6, 4,
            3, 5, 6, 4, 6, 7
        };
        create_mesh(vertices, facets);
        GEO::mesh_save(mesh, get_current_test_name()+"_0.geogram");

        constexpr GEO::index_t f0 = 3;
        constexpr GEO::index_t lv0 = 0;
        const auto fv0 = mesh.facets.vertex(f0, 0);
        const auto fv1 = mesh.facets.vertex(f0, 1);
        const auto fv2 = mesh.facets.vertex(f0, 2);
        const auto f1 = mesh.facets.adjacent(f0, lv0);
        const auto nfv0 = mesh.facets.vertex(f1, 0);
        const auto nfv1 = mesh.facets.vertex(f1, 1);
        const auto nfv2 = mesh.facets.vertex(f1, 2);

        /* Split */
        const GEO::index_t new_v = mesh.vertices.create_vertices(1);
        const GEO::index_t new_f0 = mesh.facets.create_triangles(2);
        const GEO::index_t new_f1 = new_f0+1;
        random_attributes(new_f0);
        random_attributes(new_f1);
        tri_edge_split<3>(mesh, f0, lv0, new_v, new_f0, new_f1);
        GEO::mesh_save(mesh, get_current_test_name()+"_1.geogram");

        /* Check */
        {
            EXPECT_EQ(mesh_f_idx[f0],  mesh_f_original_idx[f0]);
            EXPECT_EQ(mesh_f_idx[new_f0],   mesh_f_original_idx[f0]);
            EXPECT_EQ(mesh_f_idx[f1], mesh_f_original_idx[f1]);
            EXPECT_EQ(mesh_f_idx[new_f1],   mesh_f_original_idx[f1]);
        }
        {
            for (const auto& f : {f0, new_f0}) {
                for (GEO::index_t lv = 0; lv < 3; ++lv) {
                    if (const auto& v = mesh.facets.vertex(f, lv);
                        v == fv0)
                        EXPECT_EQ(mesh_fc_idx[mesh.facets.corner(f, lv)], mesh_fc_original_idx[mesh.facets.corner(f0, 0)]);
                    else if (v == fv1)
                        EXPECT_EQ(mesh_fc_idx[mesh.facets.corner(f, lv)], mesh_fc_original_idx[mesh.facets.corner(f0, 1)]);
                    else if (v == fv2)
                        EXPECT_EQ(mesh_fc_idx[mesh.facets.corner(f, lv)], mesh_fc_original_idx[mesh.facets.corner(f0, 2)]);
                    else
                        EXPECT_EQ(mesh_fc_idx[mesh.facets.corner(f, lv)], DEFAULT_IDX);
                }
            }
            for (const auto& f : {f1, new_f1}) {
                for (GEO::index_t lv = 0; lv < 3; ++lv) {
                    if (const auto& v = mesh.facets.vertex(f, lv);
                        v == nfv0)
                        EXPECT_EQ(mesh_fc_idx[mesh.facets.corner(f, lv)], mesh_fc_original_idx[mesh.facets.corner(f1, 0)]);
                    else if (v == nfv1)
                        EXPECT_EQ(mesh_fc_idx[mesh.facets.corner(f, lv)], mesh_fc_original_idx[mesh.facets.corner(f1, 1)]);
                    else if (v == nfv2)
                        EXPECT_EQ(mesh_fc_idx[mesh.facets.corner(f, lv)], mesh_fc_original_idx[mesh.facets.corner(f1, 2)]);
                    else
                        EXPECT_EQ(mesh_fc_idx[mesh.facets.corner(f, lv)], DEFAULT_IDX);
                }
            }
        }
    }

    /* ============================================================================================================= */

    class TriEdgeCollapseTest : public TriOperationsTest {
    protected:
        void perform_operation(
            const GEO::index_t f,
            const GEO::index_t lv
            ) override {
            if (!is_tri_edge_collapse_valid(mesh, f, lv))
                return;

            const bool EDGE_ON_BORDER = original_mesh.facets.adjacent(f, lv) == GEO::NO_FACET;

            GEO::index_t disuse_v, disuse_f0, disuse_f1;
            tri_edge_collapse<3>(mesh, f, lv, disuse_v, disuse_f0, disuse_f1);

            /* Clean disuse vertices and facets */
            GEO::vector<GEO::index_t> facets_to_delete(mesh.facets.nb(), 0);
            facets_to_delete[disuse_f0] = 1;
            if (EDGE_ON_BORDER)
                EXPECT_EQ(disuse_f1, GEO::NO_FACET);
            else {
                EXPECT_NE(disuse_f1, GEO::NO_FACET);
                facets_to_delete[disuse_f1] = 1;
            }
            mesh.facets.delete_elements(facets_to_delete);
        }
    };

    TEST_F(TriEdgeCollapseTest, tri_edge_collapse) {
        for_each_f_lv();
    }

    TEST_F(TriEdgeCollapseTest, degenerate_2d) {
        const std::array<GEO::vec2, 13> vertices = {
            GEO::vec2(0, 0), GEO::vec2(1, 0), GEO::vec2(2, 0),
            GEO::vec2(0, 1), GEO::vec2(1, 1), GEO::vec2(2, 1),
            GEO::vec2(0, 2), GEO::vec2(1, 2), GEO::vec2(2, 2),
            GEO::vec2(0, 3), GEO::vec2(1, 3), GEO::vec2(2, 3),
            GEO::vec2(1.2, 1.8)
        };
        const std::array<GEO::index_t, 14*3> facets = {
            0, 1, 4, 0, 4, 3, 1, 2, 5, 1, 5, 4,
            3, 4, 7, 3, 7, 6, 4, 12, 7, 4, 5, 12, 5, 8, 12, 7, 12, 8,
            6, 7, 10, 6, 10, 9, 7, 8, 11, 7, 11, 10
        };
        mesh.clear();
        mesh.vertices.set_dimension(2);
        mesh.vertices.create_vertices(vertices.size());
        for (const auto& v : mesh.vertices)
            mesh.vertices.point<2>(v) = vertices[v];
        mesh.facets.create_triangles(facets.size()/3);
        for (const auto& f : mesh.facets) {
            for (GEO::index_t lv = 0; lv < 3; ++lv)
                mesh.facets.set_vertex(f, lv, facets[3*f+lv]);
        }
        mesh.facets.connect();
        GEO::mesh_save(mesh, get_current_test_name()+"_0.geogram");

        /* Collapse */
        constexpr GEO::index_t f = 4;
        constexpr GEO::index_t lv = 1;

        GEO::index_t disuse_v, disuse_f0, disuse_f1;
        tri_edge_collapse<2>(mesh, f, lv, disuse_v, disuse_f0, disuse_f1);
        { // clean up
            GEO::vector<GEO::index_t> facets_to_delete(mesh.facets.nb(), 0);
            ASSERT_NE(disuse_f0, GEO::NO_FACET);
            facets_to_delete[disuse_f0] = 1;
            if (disuse_f1 != GEO::NO_FACET)
                facets_to_delete[disuse_f1] = 1;
            mesh.facets.delete_elements(facets_to_delete);
        }
        check_connections();

        GEO::mesh_save(mesh, get_current_test_name()+"_1.geogram");
    }

    TEST_F(TriEdgeCollapseTest, degenerate_2d_progressive) {
        const std::array<GEO::vec2, 13> vertices = {
            GEO::vec2(0, 0), GEO::vec2(1, 0), GEO::vec2(2, 0),
            GEO::vec2(0, 1), GEO::vec2(1, 1), GEO::vec2(2, 1),
            GEO::vec2(0, 2), GEO::vec2(1, 2), GEO::vec2(2, 2),
            GEO::vec2(0, 3), GEO::vec2(1, 3), GEO::vec2(2, 3),
            GEO::vec2(1.2, 1.8)
        };
        const std::array<GEO::index_t, 14*3> facets = {
            0, 1, 4, 0, 4, 3, 1, 2, 5, 1, 5, 4,
            3, 4, 7, 3, 7, 6, 4, 12, 7, 4, 5, 12, 5, 8, 12, 7, 12, 8,
            6, 7, 10, 6, 10, 9, 7, 8, 11, 7, 11, 10
        };
        mesh.clear();
        mesh.vertices.set_dimension(2);
        mesh.vertices.create_vertices(vertices.size());
        for (const auto& v : mesh.vertices)
            mesh.vertices.point<2>(v) = vertices[v];
        mesh.facets.create_triangles(facets.size()/3);
        for (const auto& f : mesh.facets) {
            for (GEO::index_t lv = 0; lv < 3; ++lv)
                mesh.facets.set_vertex(f, lv, facets[3*f+lv]);
        }
        mesh.facets.connect();
        // GEO::mesh_save(mesh, get_current_test_name()+"_0.geogram");

        /* Collapse */
        GEO::index_t cnt = 1;
        while (mesh.facets.nb() > 0) {
            bool collapse = false;
            for (const auto& f : mesh.facets) {
                for (GEO::index_t lv = 0; lv < 3; ++lv) {
                    if (!is_tri_edge_collapse_valid(mesh, f, lv))
                        continue;

                    GEO::index_t disuse_v, disuse_f0, disuse_f1;
                    tri_edge_collapse<2>(mesh, f, lv, disuse_v, disuse_f0, disuse_f1);
                    { // clean up
                        GEO::vector<GEO::index_t> facets_to_delete(mesh.facets.nb(), 0);
                        ASSERT_NE(disuse_f0, GEO::NO_FACET);
                        facets_to_delete[disuse_f0] = 1;
                        if (disuse_f1 != GEO::NO_FACET)
                            facets_to_delete[disuse_f1] = 1;
                        mesh.facets.delete_elements(facets_to_delete);
                    }
                    check_connections();

                    // GEO::mesh_save(mesh, get_current_test_name()+"_"+std::to_string(cnt++)+".geogram");

                    collapse = true;
                    break;
                }
                if (collapse)
                    break;
            }
        }
        EXPECT_EQ(mesh.facets.nb(), 0);
    }

    TEST_F(TriEdgeCollapseTest, degenerate_3d) {
        const std::array<GEO::vec3, 13> vertices = {
            GEO::vec3(0, 0, GEO::Numeric::random_float32()), GEO::vec3(1, 0, GEO::Numeric::random_float32()), GEO::vec3(2, 0, GEO::Numeric::random_float32()),
            GEO::vec3(0, 1, GEO::Numeric::random_float32()), GEO::vec3(1, 1, GEO::Numeric::random_float32()), GEO::vec3(2, 1, GEO::Numeric::random_float32()),
            GEO::vec3(0, 2, GEO::Numeric::random_float32()), GEO::vec3(1, 2, GEO::Numeric::random_float32()), GEO::vec3(2, 2, GEO::Numeric::random_float32()),
            GEO::vec3(0, 3, GEO::Numeric::random_float32()), GEO::vec3(1, 3, GEO::Numeric::random_float32()), GEO::vec3(2, 3, GEO::Numeric::random_float32()),
            GEO::vec3(1.2, 1.8, GEO::Numeric::random_float32())
        };
        const std::array<GEO::index_t, 14*3> facets = {
            0, 1, 4, 0, 4, 3, 1, 2, 5, 1, 5, 4,
            3, 4, 7, 3, 7, 6, 4, 12, 7, 4, 5, 12, 5, 8, 12, 7, 12, 8,
            6, 7, 10, 6, 10, 9, 7, 8, 11, 7, 11, 10
        };
        mesh.clear();
        mesh.vertices.create_vertices(vertices.size());
        for (const auto& v : mesh.vertices)
            mesh.vertices.point(v) = vertices[v];
        mesh.facets.create_triangles(facets.size()/3);
        for (const auto& f : mesh.facets) {
            for (GEO::index_t lv = 0; lv < 3; ++lv)
                mesh.facets.set_vertex(f, lv, facets[3*f+lv]);
        }
        mesh.facets.connect();
        GEO::mesh_save(mesh, get_current_test_name()+"_0.geogram");

        /* Collapse */
        constexpr GEO::index_t f = 4;
        constexpr GEO::index_t lv = 1;

        GEO::index_t disuse_v, disuse_f0, disuse_f1;
        tri_edge_collapse<3>(mesh, f, lv, disuse_v, disuse_f0, disuse_f1);
        { // clean up
            GEO::vector<GEO::index_t> facets_to_delete(mesh.facets.nb(), 0);
            ASSERT_NE(disuse_f0, GEO::NO_FACET);
            facets_to_delete[disuse_f0] = 1;
            if (disuse_f1 != GEO::NO_FACET)
                facets_to_delete[disuse_f1] = 1;
            mesh.facets.delete_elements(facets_to_delete);
        }
        check_connections();

        GEO::mesh_save(mesh, get_current_test_name()+"_1.geogram");
    }

    class TriEdgeCollapseSimpleTest : public TriOperationsSimpleTest {};

    /**
     * @brief Vertices of the collapsed edge neighbourhood used by the two tests below.
     *
     * The collapsed edge is (v0, v1), in the facet (v0, v1, v2); (v3, v1, v0) is the facet on the
     * other side of that edge. Vertex 5 is joined to both endpoints by mesh edges, which is what
     * the two tests below turn on.
     */
    const std::vector<GEO::vec3> COLLAPSED_EDGE_NEIGHBOURHOOD_VERTICES = {
        GEO::vec3(-10.0, -10.0, 0.0),                             // v1
        GEO::vec3(10.0, -10.0, 0.0),                              // v2
        GEO::vec3(-10.0, 10.0, 0.0),
        GEO::vec3(-2.075718879699707, -7.9290009811520576, 0.0),  // v0
        GEO::vec3(-7.1111459136009216, -5.9281086921691895, 0.0),
        GEO::vec3(-6.3337912559509277, -1.3703341484069824, 0.0), // neighbour of both v0 and v1
        GEO::vec3(-6.1758983135223389, -6.9848983287811279, 0.0), // v3
        GEO::vec3(1.2940673828125, -4.9864871501922607, 0.0),
        GEO::vec3(3.8541994094848633, -7.1933146119117737, 0.0),
    };

    /** The facet whose edge 0 is collapsed, and the facet on the other side of that edge. */
    const std::vector<GEO::index_t> COLLAPSED_EDGE_FACETS = {
        3, 0, 1, // the collapsed facet (v0, v1, v2)
        6, 0, 3, // its neighbour across the collapsed edge
    };

    /** The facets joining the neighbour shared by v0 and v1 to v0. */
    const std::vector<GEO::index_t> COLLAPSED_EDGE_SHARED_NEIGHBOUR_V0_FACETS = {
        6, 3, 5,
        7, 5, 3,
    };

    /** The facets joining the neighbour shared by v0 and v1 to v1. */
    const std::vector<GEO::index_t> COLLAPSED_EDGE_SHARED_NEIGHBOUR_V1_FACETS = {
        5, 2, 0,
        5, 0, 4,
    };

    /** The remaining facets of the two one-rings. */
    const std::vector<GEO::index_t> COLLAPSED_EDGE_RING_FACETS = {
        6, 4, 0,
        8, 3, 1,
        8, 7, 3,
    };

    /** Local edge of the collapsed facet: the collapsed edge joins local vertices 0 and 1. */
    constexpr GEO::index_t COLLAPSED_EDGE_LV = 0;

    /**
     * @brief A collapse that would duplicate an edge must be rejected.
     *
     * @details Merging v1 into v0 turns the two edges (v0, u) and (v1, u) into the same edge whenever
     *          u is a neighbour of both endpoints. Here vertex 5 is such a neighbour, which is not one
     *          of the vertices opposite to the collapsed edge, so the collapse would leave the mesh
     *          with two copies of the edge (v0, 5): it would stop being 2-manifold and the adjacency
     *          maintained by the operation would no longer be the one mesh.facets.connect()
     *          recomputes. This configuration comes from a random CDT mesh.
     */
    TEST_F(TriEdgeCollapseSimpleTest, rejects_collapse_creating_duplicate_edge) {
        std::vector<GEO::index_t> facets = COLLAPSED_EDGE_FACETS;
        facets.insert(facets.end(), COLLAPSED_EDGE_SHARED_NEIGHBOUR_V0_FACETS.begin(), COLLAPSED_EDGE_SHARED_NEIGHBOUR_V0_FACETS.end());
        facets.insert(facets.end(), COLLAPSED_EDGE_SHARED_NEIGHBOUR_V1_FACETS.begin(), COLLAPSED_EDGE_SHARED_NEIGHBOUR_V1_FACETS.end());
        facets.insert(facets.end(), COLLAPSED_EDGE_RING_FACETS.begin(), COLLAPSED_EDGE_RING_FACETS.end());
        create_mesh(COLLAPSED_EDGE_NEIGHBOURHOOD_VERTICES, facets);

        EXPECT_FALSE(is_tri_edge_collapse_valid(mesh, 0, COLLAPSED_EDGE_LV));

        /* The rejected collapse leaves the mesh untouched and consistent. */
        EXPECT_EQ(mesh.facets.vertex(0, 0), 3);
        EXPECT_EQ(mesh.facets.vertex(0, 1), 0);
        EXPECT_EQ(mesh.facets.vertex(0, 2), 1);
        expect_adjacency_consistent();
    }

    /**
     * @brief The same neighbourhood without the facets joining the shared neighbour to v1 collapses.
     *
     * @details This is the counterpart of the test above: vertex 5 is then only joined to v0, the two
     *          endpoints no longer have any neighbour in common besides the vertices opposite to the
     *          collapsed edge, and the collapse is valid.
     */
    TEST_F(TriEdgeCollapseSimpleTest, accepts_collapse_when_link_condition_holds) {
        std::vector<GEO::index_t> facets = COLLAPSED_EDGE_FACETS;
        facets.insert(facets.end(), COLLAPSED_EDGE_SHARED_NEIGHBOUR_V0_FACETS.begin(), COLLAPSED_EDGE_SHARED_NEIGHBOUR_V0_FACETS.end());
        facets.insert(facets.end(), COLLAPSED_EDGE_RING_FACETS.begin(), COLLAPSED_EDGE_RING_FACETS.end());
        create_mesh(COLLAPSED_EDGE_NEIGHBOURHOOD_VERTICES, facets);

        ASSERT_TRUE(is_tri_edge_collapse_valid(mesh, 0, COLLAPSED_EDGE_LV));

        GEO::index_t disuse_v, disuse_f0, disuse_f1;
        tri_edge_collapse<3>(mesh, 0, COLLAPSED_EDGE_LV, disuse_v, disuse_f0, disuse_f1);

        /* The two facets of the collapsed edge became degenerate and are not used any more. */
        EXPECT_EQ(disuse_v, 0);
        EXPECT_EQ(disuse_f0, 0);
        EXPECT_EQ(disuse_f1, 1);
        GEO::vector<GEO::index_t> facets_to_delete(mesh.facets.nb(), 0);
        facets_to_delete[disuse_f0] = 1;
        facets_to_delete[disuse_f1] = 1;
        mesh.facets.delete_elements(facets_to_delete);
        EXPECT_EQ(mesh.facets.nb(), 5);

        expect_adjacency_consistent();
    }

    /* ============================================================================================================= */

    class TriEdgeSwapTest : public TriOperationsTest {
    protected:
        void perform_operation(
            const GEO::index_t f,
            const GEO::index_t lv
            ) override {
            if (is_tri_edge_swap_valid(mesh, f, lv))
                tri_edge_swap(mesh, f, lv);
        }
    };

    TEST_F(TriEdgeSwapTest, tri_edge_swap) {
        for_each_f_lv();
    }

    TEST_F(TriEdgeSwapTest, degenerate_2d) {
        const std::array<GEO::vec2, 7> vertices = {
            GEO::vec2(0, 0), GEO::vec2(1, 0), GEO::vec2(2, 0),
            GEO::vec2(1, 1),
            GEO::vec2(0, 2), GEO::vec2(1, 2), GEO::vec2(2, 2),
        };
        const std::array<GEO::index_t, 6*3> facets = {
            0, 3, 5, 5, 3, 2,
            // 0, 2, 3,
            0, 1, 3, 1, 2, 3,
            4, 0, 5, 5, 2, 6
        };
        mesh.clear();
        mesh.vertices.set_dimension(2);
        mesh.vertices.create_vertices(vertices.size());
        for (const auto& v : mesh.vertices)
            mesh.vertices.point<2>(v) = vertices[v];
        mesh.facets.create_triangles(facets.size()/3);
        for (const auto& f : mesh.facets) {
            for (GEO::index_t lv = 0; lv < 3; ++lv)
                mesh.facets.set_vertex(f, lv, facets[3*f+lv]);
        }
        mesh.facets.connect();
        GEO::mesh_save(mesh, get_current_test_name()+"_0.geogram");

        /* Collapse */
        constexpr GEO::index_t f = 0;
        constexpr GEO::index_t lv = 1;

        tri_edge_swap(mesh, f, lv);
        check_connections();

        GEO::mesh_save(mesh, get_current_test_name()+"_1.geogram");
    }

    class TriEdgeSwapSimpleTest : public TriOperationsSimpleTest {};

    TEST_F(TriEdgeSwapSimpleTest, manage_attributes) {
        const std::vector<GEO::vec3> vertices = {
            GEO::vec3(0, 2, 0), GEO::vec3(1, 2, 0), GEO::vec3(2, 2, 0),
            GEO::vec3(0, 1, 0), GEO::vec3(2, 1, 0),
            GEO::vec3(0, 0, 0), GEO::vec3(1, 0, 0), GEO::vec3(2, 0, 0),
        };
        const std::vector<GEO::index_t> facets = {
            0, 3, 1,
            1, 4, 2, 1, 3, 6, 1, 6, 4,
            3, 5, 6, 4, 6, 7
        };
        create_mesh(vertices, facets);
        GEO::mesh_save(mesh, get_current_test_name()+"_0.geogram");

        const GEO::index_t f0 = 3;
        const GEO::index_t lv0 = 0;
        const auto fv0 = mesh.facets.vertex(f0, lv0);
        const auto fv = mesh.facets.vertex(f0, (lv0+2)%3);
        const auto f1 = mesh.facets.adjacent(f0, lv0);
        const auto nlv = (mesh.facets.find_vertex(f1, fv0)+1)%3;
        const auto nfv = mesh.facets.vertex(f1, nlv);

        /* Swap */
        tri_edge_swap(mesh, f0, lv0);
        GEO::mesh_save(mesh, get_current_test_name()+"_1.geogram");

        /* Check */
        {
            EXPECT_EQ(mesh_f_idx[f0], DEFAULT_IDX);
            EXPECT_EQ(mesh_f_idx[f1], DEFAULT_IDX);
        }
        {
            for (const auto& f : {f0, f1}) {
                for (GEO::index_t lv = 0; lv < 3; ++lv) {
                    if (const auto& v = mesh.facets.vertex(f, lv);
                        v == fv)
                        EXPECT_EQ(mesh_fc_idx[mesh.facets.corner(f, lv)], mesh_fc_original_idx[mesh.facets.corner(f0, (lv0+2)%3)]);
                    else if (v == nfv)
                        EXPECT_EQ(mesh_fc_idx[mesh.facets.corner(f, lv)], mesh_fc_original_idx[mesh.facets.corner(f1, nlv)]);
                    else
                        EXPECT_EQ(mesh_fc_idx[mesh.facets.corner(f, lv)], DEFAULT_IDX);
                }
            }
        }
    }

    /**
     * @brief Vertices of the quad used by the two tests below, and of its neighbourhood.
     *
     * The quad is (v1, v2, v0, v3), flipped along the diagonal (v0, v1) opposite to its local
     * vertex 2. It is non-convex and its other diagonal (v2, v3) is the segment the flip would
     * introduce. The last two vertices complete the one-rings of the endpoints of that diagonal.
     */
    const std::vector<GEO::vec3> QUAD_NEIGHBOURHOOD_VERTICES = {
        GEO::vec3(-10.0, -10.0, 0.0),                             // v2
        GEO::vec3(10.0, -10.0, 0.0),
        GEO::vec3(-10.0, 10.0, 0.0),
        GEO::vec3(-2.075718879699707, -7.9290009811520576, 0.0),  // v0
        GEO::vec3(-7.1111459136009216, -5.9281086921691895, 0.0),
        GEO::vec3(-6.3337912559509277, -1.3703341484069824, 0.0), // v3
        GEO::vec3(-6.1758983135223389, -6.9848983287811279, 0.0), // v1
    };

    /** The two facets of the quad: the flipped one first, then its neighbour across the diagonal. */
    const std::vector<GEO::index_t> QUAD_FACETS = {
        6, 0, 3, // the flipped facet (v1, v2, v0)
        6, 3, 5, // its neighbour across (v0, v1)
    };

    /** The two facets owning the other diagonal (v2, v3) of the quad. */
    const std::vector<GEO::index_t> QUAD_OTHER_DIAGONAL_FACETS = {
        5, 2, 0, // owns the directed edge (v2 -> v3)
        5, 0, 4, // owns the directed edge (v3 -> v2)
    };

    /** The neighbours of the quad along its two edges (v1, v2) and (v2, v0). */
    const std::vector<GEO::index_t> QUAD_SIDE_FACETS = {
        6, 4, 0,
        3, 0, 1,
    };

    /** Local index of the flipped facet whose edge (lv -> lv+1) is the diagonal (v0 -> v1). */
    constexpr GEO::index_t QUAD_FLIPPED_EDGE_LV = 2;

    /**
     * @brief A flip whose other diagonal is already an edge of the mesh must be rejected.
     *
     * @details Flipping it would make the two flipped facets share that edge with the two facets
     *          that already own it: the mesh would stop being 2-manifold and the adjacency
     *          maintained by the operation would no longer be the one mesh.facets.connect()
     *          recomputes. This configuration comes from a random CDT mesh; note that the facets
     *          owning the other diagonal are not neighbours of the quad, so inspecting only the
     *          neighbours of the quad (as the previous implementation did) misses it.
     */
    TEST_F(TriEdgeSwapSimpleTest, rejects_flip_creating_duplicate_edge) {
        std::vector<GEO::index_t> facets = QUAD_FACETS;
        facets.insert(facets.end(), QUAD_OTHER_DIAGONAL_FACETS.begin(), QUAD_OTHER_DIAGONAL_FACETS.end());
        facets.insert(facets.end(), QUAD_SIDE_FACETS.begin(), QUAD_SIDE_FACETS.end());
        create_mesh(QUAD_NEIGHBOURHOOD_VERTICES, facets);

        EXPECT_FALSE(is_tri_edge_swap_valid(mesh, 0, QUAD_FLIPPED_EDGE_LV));
        EXPECT_FALSE(tri_edge_swap(mesh, 0, QUAD_FLIPPED_EDGE_LV));

        /* The rejected flip must leave the facet and the connectivity untouched. */
        EXPECT_EQ(mesh.facets.vertex(0, 0), QUAD_FACETS[0]);
        EXPECT_EQ(mesh.facets.vertex(0, 1), QUAD_FACETS[1]);
        EXPECT_EQ(mesh.facets.vertex(0, 2), QUAD_FACETS[2]);
        expect_adjacency_consistent();
    }

    /**
     * @brief The same quad, without the facets owning its other diagonal, can be flipped.
     *
     * @details This is the counterpart of the test above: it states that the check rejects only the
     *          flips that would duplicate an existing edge.
     */
    TEST_F(TriEdgeSwapSimpleTest, accepts_flip_when_other_diagonal_is_free) {
        std::vector<GEO::index_t> facets = QUAD_FACETS;
        facets.insert(facets.end(), QUAD_SIDE_FACETS.begin(), QUAD_SIDE_FACETS.end());
        create_mesh(QUAD_NEIGHBOURHOOD_VERTICES, facets);

        EXPECT_TRUE(is_tri_edge_swap_valid(mesh, 0, QUAD_FLIPPED_EDGE_LV));
        EXPECT_TRUE(tri_edge_swap(mesh, 0, QUAD_FLIPPED_EDGE_LV));

        /* The flipped facet (v1, v2, v0) now uses the other diagonal: v3 takes the place of v1. */
        EXPECT_EQ(mesh.facets.vertex(0, 0), 5);
        EXPECT_EQ(mesh.facets.vertex(0, 1), 0);
        EXPECT_EQ(mesh.facets.vertex(0, 2), 3);
        expect_adjacency_consistent();
    }
}
