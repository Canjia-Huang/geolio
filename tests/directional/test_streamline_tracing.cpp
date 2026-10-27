//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/27.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <geogram/voronoi/CVT.h>
#include <gtest/gtest.h>
#include <geolio/directional/streamline_tracing.h>
#include <geolio/directional/frame_field.h>

#include "../utils.h"

namespace geolio::test
{
    class StreamlineSurficialTracingTest : public ::testing::Test {
    protected:
        void SetUp() override {
            ASSERT_TRUE(mesh_.load(std::string(TEST_DATA_PATH)+"three_holes.geogram"));

            /* Samples */
            GEO::CentroidalVoronoiTesselation CVT(&mesh_);
            CVT.compute_initial_sampling(points_nb_);
            CVT.Newton_iterations(30);

            points_.reserve(3*points_nb_);
            for (GEO::index_t v = 0; v < points_nb_; ++v) {
                const auto& p = CVT.embedding(v);
                points_.push_back(p[0]);
                points_.push_back(p[1]);
                points_.push_back(p[2]);
            }

            /* Directional */
            FrameField frame_field;
            frame_field.create_curvature_directions(mesh_, 45.0);
            const auto& frames = frame_field.frames();
            const GEO::index_t frames_size = 3*N_*mesh_.facets.nb();
            ASSERT_EQ(frames.size(), frames_size);
            directions_.resize(frames_size);
            std::ranges::copy_n(frame_field.frames().data(), frames_size, directions_.data());

            /* Tracer */
            tracer = std::make_unique<StreamlineSurficialTracing>(mesh_);
            tracer->set_initial_points(points_nb_, points_.data());
            tracer->set_per_facet_directions(N_, directions_.data());
        }

        void save_result() {
            GEO::Attribute<GEO::index_t> mesh_v_tracing_times(mesh_.vertices.attributes(), "tracing_times");
            GEO::Attribute<GEO::index_t> mesh_e_point_idx(mesh_.edges.attributes(), "point_idx");
            tracer->append_tracing_points_to_mesh(mesh_, &mesh_v_tracing_times, &mesh_e_point_idx);
            EXPECT_TRUE(mesh_.save(get_current_test_name()+".geogram"));
        }

        GEO::Mesh mesh_;

        const GEO::index_t points_nb_ = 1e3;
        std::vector<double> points_;

        const GEO::index_t N_ = 3;
        std::vector<double> directions_;

        std::unique_ptr<StreamlineSurficialTracing> tracer;
    };

    TEST_F(StreamlineSurficialTracingTest, constant_steps) {
        tracer->tracing(30);
        save_result();
    }
}
