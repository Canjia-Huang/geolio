//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/13.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <geogram/mesh/mesh_io.h>
#include <gtest/gtest.h>
#include <geolio/common/mesh_edges_AABB.h>
#include "../utils.h"

namespace geolio::test
{
    template <GEO::index_t DIM>
    class MeshEdgesAABBTest : public ::testing::Test {
    protected:
        void SetUp() override {
            mesh.vertices.set_dimension(DIM);
        }

        void initialize() {
            AABB.initialize(mesh);
        }

        void querying() {
            ASSERT_FALSE(AABB.mesh() == nullptr);

            const GEO::index_t query_points_nb = query_points.size();
            nearest_edges.assign(query_points_nb, GEO::NO_EDGE);
            nearest_points.resize(query_points_nb);
            nearest_sq_dist.assign(query_points_nb, -1.0);
            for (GEO::index_t i = 0; i < query_points_nb; ++i)
                nearest_edges[i] = AABB.nearest_edge(query_points[i], nearest_points[i], nearest_sq_dist[i]);
        }

        void checking() {
            const GEO::index_t query_points_nb = query_points.size();
            ASSERT_EQ(nearest_edges.size(), query_points_nb);
            ASSERT_EQ(nearest_points.size(), query_points_nb);
            ASSERT_EQ(nearest_sq_dist.size(), query_points_nb);

            for (GEO::index_t i = 0; i < query_points_nb; ++i) {
                GEO::index_t gt_nearest_edge = GEO::NO_EDGE;
                GEO::vecng<DIM, double> gt_nearest_point;
                double gt_nearest_sq_dist = std::numeric_limits<double>::max();

                for (const auto& e : mesh.edges) {
                    GEO::vecng<DIM, double> nearest_point;
                    double lambda0, lambda1;
                    const double sq_dist = GEO::Geom::point_segment_squared_distance(
                        query_points[i],
                        mesh.vertices.point<DIM>(mesh.edges.vertex(e, 0)),
                        mesh.vertices.point<DIM>(mesh.edges.vertex(e, 1)),
                        nearest_point,
                        lambda0, lambda1);

                    if (sq_dist < gt_nearest_sq_dist) {
                        gt_nearest_edge = e;
                        gt_nearest_point = nearest_point;
                        gt_nearest_sq_dist = sq_dist;
                    }
                }

                EXPECT_EQ(gt_nearest_edge, nearest_edges[i]);
                EXPECT_NEAR(GEO::distance2(gt_nearest_point, nearest_points[i]), 0, 1e-10);
                EXPECT_NEAR(gt_nearest_sq_dist, nearest_sq_dist[i], 1e-10);
            }
        }

        void generate_random_query_points(
            const GEO::index_t points_nb
            ) {
            query_points.resize(points_nb);
            for (GEO::index_t i = 0; i < points_nb; ++i) {
                auto& p = query_points[i];
                for (GEO::index_t d = 0; d < DIM; ++d)
                    p[d] = GEO::Numeric::random_float32();

                p *= 2;
                p.x -= 1;
                p.y -= 1;
                p.z -= 1;
            }
        }

        void append_to_mesh(
            GEO::Mesh& mesh_out
            ) const {
            mesh_out.copy(mesh);
            ASSERT_EQ(mesh_out.vertices.dimension(), mesh.vertices.dimension());

            const GEO::index_t query_points_nb = query_points.size();
            ASSERT_EQ(nearest_edges.size(), query_points_nb);
            ASSERT_EQ(nearest_points.size(), query_points_nb);
            ASSERT_EQ(nearest_sq_dist.size(), query_points_nb);

            GEO::Attribute<GEO::index_t> mesh_out_v_start_end(mesh_out.vertices.attributes(), "start_end");
            GEO::Attribute<double> mesh_out_e_sq_dist(mesh_out.edges.attributes(), "sq_dist");
            GEO::index_t new_v = mesh_out.vertices.create_vertices(2*query_points_nb);
            GEO::index_t new_e = mesh_out.edges.create_edges(query_points_nb);
            for (GEO::index_t i = 0; i < query_points_nb; ++i) {
                mesh_out.vertices.point<DIM>(new_v) = query_points[i];
                mesh_out.vertices.point<DIM>(new_v+1) = nearest_points[i];
                mesh_out.edges.set_vertex(new_e, 0, new_v);
                mesh_out.edges.set_vertex(new_e, 1, new_v+1);
                mesh_out_v_start_end[new_v] = 1;
                mesh_out_v_start_end[new_v+1] = 2;
                mesh_out_e_sq_dist[new_e] = nearest_sq_dist[i];

                new_v += 2;
                ++new_e;
            }
        }

        GEO::Mesh mesh;
        MeshEdgesAABB<DIM> AABB;

        std::vector<GEO::vecng<DIM, double>> query_points;
        std::vector<GEO::index_t>            nearest_edges;
        std::vector<GEO::vecng<DIM, double>> nearest_points;
        std::vector<double>                  nearest_sq_dist;
    };

    template<GEO::index_t DIM>
    struct DimWrapper {
        static constexpr GEO::index_t value = DIM;
    };

    using Dim2 = std::integral_constant<GEO::index_t, 2>;
    using Dim3 = std::integral_constant<GEO::index_t, 3>;
    using DimTypes = ::testing::Types<Dim2, Dim3>;

    template <typename DimType>
    class MeshEdgesAABBDimTest : public MeshEdgesAABBTest<DimType::value> {};

    TYPED_TEST_SUITE(MeshEdgesAABBDimTest, DimTypes);

    TYPED_TEST(MeshEdgesAABBDimTest, simple) {
        constexpr GEO::index_t DIM = TypeParam::value;

        this->mesh.vertices.create_vertices(4);
        if constexpr (DIM == 2) {
            this->mesh.vertices.template point<2>(0) = GEO::vec2(0, 0);
            this->mesh.vertices.template point<2>(1) = GEO::vec2(1, 0);
            this->mesh.vertices.template point<2>(2) = GEO::vec2(0, 1);
            this->mesh.vertices.template point<2>(3) = GEO::vec2(1, 1);
        }
        else if constexpr (DIM == 3) {
            this->mesh.vertices.point(0) = GEO::vec3(0, 0, 0);
            this->mesh.vertices.point(1) = GEO::vec3(1, 0, 0);
            this->mesh.vertices.point(2) = GEO::vec3(0, 1, 0);
            this->mesh.vertices.point(3) = GEO::vec3(0, 0, 1);
        }
        else
            FAIL();
        this->mesh.edges.create_edges(3);
        this->mesh.edges.set_vertex(0, 0, 0); this->mesh.edges.set_vertex(0, 1, 1);
        this->mesh.edges.set_vertex(1, 0, 0); this->mesh.edges.set_vertex(1, 1, 2);
        this->mesh.edges.set_vertex(2, 0, 0); this->mesh.edges.set_vertex(2, 1, 3);

        this->initialize();

        if constexpr (DIM == 2) {
            this->query_points.push_back(GEO::vec2(0.6, 0));
            this->query_points.push_back(GEO::vec2(0, 0.8));
            this->query_points.push_back(GEO::vec2(0, 0.2));
            this->query_points.push_back(GEO::vec2(0.3, 0));
            this->query_points.push_back(GEO::vec2(0.1, 0));
            this->query_points.push_back(GEO::vec2(0.7, 0));
            this->query_points.push_back(GEO::vec2(0, 0.3));
            this->query_points.push_back(GEO::vec2(0, 1.5));
        }
        else if constexpr (DIM == 3) {
            this->query_points.push_back(GEO::vec3(0.6, 0, 0.5));
            this->query_points.push_back(GEO::vec3(0, 0.8, 0.7));
            this->query_points.push_back(GEO::vec3(0, 0.2, 0.5));
            this->query_points.push_back(GEO::vec3(0.3, 0, 0.5));
            this->query_points.push_back(GEO::vec3(0.1, 0, 0.2));
            this->query_points.push_back(GEO::vec3(0.7, 0, 0.2));
            this->query_points.push_back(GEO::vec3(0, 0.3, 0.8));
            this->query_points.push_back(GEO::vec3(0, 1.5, 0));
        }
        else
            FAIL();

        this->querying();
        this->checking();
        {
            GEO::Mesh mesh_out;
            this->append_to_mesh(mesh_out);
            mesh_out.save(get_current_test_name()+".geogram");
        }
    }

    // TEST_F(MeshEdgesAABB3DTest, simple_random) {
    //     mesh.vertices.create_vertices(4);
    //     mesh.vertices.point(0) = GEO::vec3(0, 0, 0);
    //     mesh.vertices.point(1) = GEO::vec3(1, 0, 0);
    //     mesh.vertices.point(2) = GEO::vec3(0, 1, 0);
    //     mesh.vertices.point(3) = GEO::vec3(0, 0, 1);
    //     mesh.edges.create_edges(3);
    //     mesh.edges.set_vertex(0, 0, 0); mesh.edges.set_vertex(0, 1, 1);
    //     mesh.edges.set_vertex(1, 0, 0); mesh.edges.set_vertex(1, 1, 2);
    //     mesh.edges.set_vertex(2, 0, 0); mesh.edges.set_vertex(2, 1, 3);
    //
    //     AABB.initialize(mesh);
    //
    //     constexpr GEO::index_t POINTS_NB = 1000;
    //     std::vector<GEO::vec3> points;
    //     generate_random_points(POINTS_NB, points);
    //
    //     std::vector<GEO::vec3> nearest_points(POINTS_NB);
    //     std::vector<double> nearest_sq_dist(POINTS_NB);
    //     for (GEO::index_t i = 0; i < POINTS_NB; ++i) {
    //         AABB.nearest_edge(
    //             points[i],
    //             nearest_points[i],
    //             nearest_sq_dist[i]);
    //     }
    //
    //     GEO::Mesh mesh_out;
    //     append_to_mesh(points, nearest_points, nearest_sq_dist, mesh_out);
    //     EXPECT_TRUE(GEO::mesh_save(mesh_out, get_current_test_name()+".geogram"));
    // }
    //
    // TEST_F(MeshEdgesAABB3DTest, stress) {
    //     constexpr GEO::index_t EDGES_NB = 1000;
    //     std::vector<GEO::vec3> start_points;
    //     std::vector<GEO::vec3> end_points;
    //     generate_random_points(EDGES_NB, start_points);
    //     generate_random_points(EDGES_NB, end_points);
    //
    //     mesh.vertices.create_vertices(2*EDGES_NB);
    //     mesh.edges.create_edges(EDGES_NB);
    //     for (GEO::index_t i = 0; i < EDGES_NB; ++i) {
    //         mesh.vertices.point(2*i) = start_points[i];
    //         mesh.vertices.point(2*i+1) = end_points[i];
    //         mesh.edges.set_vertex(i, 0, 2*i);
    //         mesh.edges.set_vertex(i, 1, 2*i+1);
    //     }
    //
    //     AABB.initialize(mesh);
    //
    //     constexpr GEO::index_t POINTS_NB = 1000;
    //     std::vector<GEO::vec3> points;
    //     generate_random_points(POINTS_NB, points);
    //
    //     std::vector<GEO::vec3> nearest_points(POINTS_NB);
    //     std::vector<double> nearest_sq_dist(POINTS_NB);
    //     for (GEO::index_t i = 0; i < POINTS_NB; ++i) {
    //         AABB.nearest_edge(
    //             points[i],
    //             nearest_points[i],
    //             nearest_sq_dist[i]);
    //     }
    //
    //     GEO::Mesh mesh_out;
    //     append_to_mesh(points, nearest_points, nearest_sq_dist, mesh_out);
    //     EXPECT_TRUE(GEO::mesh_save(mesh_out, get_current_test_name()+".geogram"));
    // }
}