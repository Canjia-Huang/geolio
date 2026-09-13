//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/13.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <geogram/mesh/mesh_io.h>
#include <gtest/gtest.h>
#include <geolio/common/mesh_edges_AABB3d.h>
#include "../utils.h"

namespace geolio::test
{
    class MeshEdgesAABB3DTest : public ::testing::Test {
    public:
        static void generate_random_points(
            const GEO::index_t points_nb,
            std::vector<GEO::vec3>& points
            ) {
            points.resize(points_nb);
            for (GEO::index_t i = 0; i < points_nb; ++i) {
                GEO::vec3 p(GEO::Numeric::random_float32(), GEO::Numeric::random_float32(), GEO::Numeric::random_float32());
                p *= 2;
                p.x -= 1;
                p.y -= 1;
                p.z -= 1;

                points[i] = p;
            }
        }

        void append_to_mesh(
            const std::vector<GEO::vec3>& points,
            const std::vector<GEO::vec3>& nearest_points,
            const std::vector<double>& nearest_sq_dist,
            GEO::Mesh& M_out
            ) const {
            M_out.copy(mesh);

            const GEO::index_t POINTS_NB = points.size();
            ASSERT_EQ(nearest_points.size(), POINTS_NB);
            ASSERT_EQ(nearest_sq_dist.size(), POINTS_NB);

            GEO::Attribute<GEO::index_t> M_out_v_start_end(M_out.vertices.attributes(), "start_end");
            GEO::Attribute<double> M_out_e_sq_dist(M_out.edges.attributes(), "sq_dist");
            GEO::index_t new_v = M_out.vertices.create_vertices(2*POINTS_NB);
            GEO::index_t new_e = M_out.edges.create_edges(POINTS_NB);
            for (GEO::index_t i = 0; i < POINTS_NB; ++i) {
                M_out.vertices.point(new_v) = points[i];
                M_out.vertices.point(new_v+1) = nearest_points[i];
                M_out.edges.set_vertex(new_e, 0, new_v);
                M_out.edges.set_vertex(new_e, 1, new_v+1);
                M_out_v_start_end[new_v] = 1;
                M_out_v_start_end[new_v+1] = 2;
                M_out_e_sq_dist[new_e] = nearest_sq_dist[i];

                new_v += 2;
                ++new_e;
            }
        }

        MeshEdgesAABB3d AABB;
        GEO::Mesh mesh;
    };

    TEST_F(MeshEdgesAABB3DTest, simple) {
        mesh.vertices.create_vertices(4);
        mesh.vertices.point(0) = GEO::vec3(0, 0, 0);
        mesh.vertices.point(1) = GEO::vec3(1, 0, 0);
        mesh.vertices.point(2) = GEO::vec3(0, 1, 0);
        mesh.vertices.point(3) = GEO::vec3(0, 0, 1);
        mesh.edges.create_edges(3);
        mesh.edges.set_vertex(0, 0, 0); mesh.edges.set_vertex(0, 1, 1);
        mesh.edges.set_vertex(1, 0, 0); mesh.edges.set_vertex(1, 1, 2);
        mesh.edges.set_vertex(2, 0, 0); mesh.edges.set_vertex(2, 1, 3);

        AABB.initialize(mesh);

        EXPECT_EQ(AABB.nearest_edge(GEO::vec3(0.6, 0, 0.5)), 0);
        EXPECT_EQ(AABB.nearest_edge(GEO::vec3(0, 0.8, 0.7)), 1);
        EXPECT_EQ(AABB.nearest_edge(GEO::vec3(0, 0.2, 0.5)), 2);
        EXPECT_EQ(AABB.nearest_edge(GEO::vec3(0.3, 0, 0.5)), 2);
        EXPECT_NEAR(AABB.squared_distance(GEO::vec3(0.1, 0, 0.2)), 0.1*0.1, 1e-10);
        EXPECT_NEAR(AABB.squared_distance(GEO::vec3(0.7, 0, 0.2)), 0.2*0.2, 1e-10);
        EXPECT_NEAR(AABB.squared_distance(GEO::vec3(0, 0.3, 0.8)), 0.3*0.3, 1e-10);
        EXPECT_NEAR(AABB.squared_distance(GEO::vec3(0, 1.5, 0)), 0.5*0.5, 1e-10);
    }

    TEST_F(MeshEdgesAABB3DTest, simple_random) {
        mesh.vertices.create_vertices(4);
        mesh.vertices.point(0) = GEO::vec3(0, 0, 0);
        mesh.vertices.point(1) = GEO::vec3(1, 0, 0);
        mesh.vertices.point(2) = GEO::vec3(0, 1, 0);
        mesh.vertices.point(3) = GEO::vec3(0, 0, 1);
        mesh.edges.create_edges(3);
        mesh.edges.set_vertex(0, 0, 0); mesh.edges.set_vertex(0, 1, 1);
        mesh.edges.set_vertex(1, 0, 0); mesh.edges.set_vertex(1, 1, 2);
        mesh.edges.set_vertex(2, 0, 0); mesh.edges.set_vertex(2, 1, 3);

        AABB.initialize(mesh);

        constexpr GEO::index_t POINTS_NB = 1000;
        std::vector<GEO::vec3> points;
        generate_random_points(POINTS_NB, points);

        std::vector<GEO::vec3> nearest_points(POINTS_NB);
        std::vector<double> nearest_sq_dist(POINTS_NB);
        for (GEO::index_t i = 0; i < POINTS_NB; ++i) {
            AABB.nearest_edge(
                points[i],
                nearest_points[i],
                nearest_sq_dist[i]);
        }

        GEO::Mesh mesh_out;
        append_to_mesh(points, nearest_points, nearest_sq_dist, mesh_out);
        EXPECT_TRUE(GEO::mesh_save(mesh_out, get_current_test_name()+".geogram"));
    }

    TEST_F(MeshEdgesAABB3DTest, stress) {
        constexpr GEO::index_t EDGES_NB = 1000;
        std::vector<GEO::vec3> start_points;
        std::vector<GEO::vec3> end_points;
        generate_random_points(EDGES_NB, start_points);
        generate_random_points(EDGES_NB, end_points);

        mesh.vertices.create_vertices(2*EDGES_NB);
        mesh.edges.create_edges(EDGES_NB);
        for (GEO::index_t i = 0; i < EDGES_NB; ++i) {
            mesh.vertices.point(2*i) = start_points[i];
            mesh.vertices.point(2*i+1) = end_points[i];
            mesh.edges.set_vertex(i, 0, 2*i);
            mesh.edges.set_vertex(i, 1, 2*i+1);
        }

        AABB.initialize(mesh);

        constexpr GEO::index_t POINTS_NB = 1000;
        std::vector<GEO::vec3> points;
        generate_random_points(POINTS_NB, points);

        std::vector<GEO::vec3> nearest_points(POINTS_NB);
        std::vector<double> nearest_sq_dist(POINTS_NB);
        for (GEO::index_t i = 0; i < POINTS_NB; ++i) {
            AABB.nearest_edge(
                points[i],
                nearest_points[i],
                nearest_sq_dist[i]);
        }

        GEO::Mesh mesh_out;
        append_to_mesh(points, nearest_points, nearest_sq_dist, mesh_out);
        EXPECT_TRUE(GEO::mesh_save(mesh_out, get_current_test_name()+".geogram"));
    }
}