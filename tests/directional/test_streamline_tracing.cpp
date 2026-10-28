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
    class StreamlineTracerTest : public ::testing::Test {
    protected:
        virtual void initialize_points() = 0;

        virtual void initialize_directions() = 0;

        virtual void initialize_tracer() = 0;

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

        std::unique_ptr<StreamlineTracer> tracer;
    };

    class StreamlineSurficialTracerTest : public StreamlineTracerTest {
    protected:
        void initialize_points() override {
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
        }

        void initialize_directions() override {
            FrameField frame_field;
            frame_field.create_curvature_directions(mesh_, false, 45.0);
            const auto& frames = frame_field.frames();
            const GEO::index_t frames_size = 3*N_*mesh_.facets.nb();
            ASSERT_EQ(frames.size(), frames_size);
            directions_.resize(frames_size);
            std::ranges::copy_n(frame_field.frames().data(), frames_size, directions_.data());
        }

        void initialize_tracer() override {
            tracer = std::make_unique<StreamlineSurficialTracer>(mesh_);
            const auto surf_tracer = dynamic_cast<StreamlineSurficialTracer*>(tracer.get());
            surf_tracer->set_initial_points(points_nb_, points_.data());
            surf_tracer->set_per_facet_directions(N_, directions_.data());
        }

        void SetUp() override {
            ASSERT_TRUE(mesh_.load(std::string(TEST_DATA_PATH)+"three_holes.geogram"));

            initialize_points();
            initialize_directions();
            initialize_tracer();
        }
    };

    TEST_F(StreamlineSurficialTracerTest, constant_steps) {
        tracer->trace(30);
        save_result();
    }

    class StreamlineVolumetricTracerTest : public StreamlineTracerTest {
    protected:
        void initialize_points() override {
            GEO::CentroidalVoronoiTesselation CVT(&mesh_);
            CVT.set_volumetric(true);
            CVT.compute_initial_sampling(points_nb_);
            CVT.Newton_iterations(30);

            points_.reserve(3*points_nb_);
            for (GEO::index_t v = 0; v < points_nb_; ++v) {
                const auto& p = CVT.embedding(v);
                points_.push_back(p[0]);
                points_.push_back(p[1]);
                points_.push_back(p[2]);
            }
        }

        void initialize_directions() override {
            mesh_.cells.compute_borders();

            FrameField frame_field;
            frame_field.create_curvature_directions(mesh_, true, 45.0);
            const auto& frames = frame_field.frames();
            const GEO::index_t frames_size = 3*N_*mesh_.cells.nb();
            ASSERT_EQ(frames.size(), frames_size);
            directions_.resize(frames_size);
            std::ranges::copy_n(frame_field.frames().data(), frames_size, directions_.data());
        }

        void initialize_tracer() override {
            tracer = std::make_unique<StreamlineVolumetricTracer>(mesh_);
            const auto surf_tracer = dynamic_cast<StreamlineVolumetricTracer*>(tracer.get());
            surf_tracer->set_initial_points(points_nb_, points_.data());
            surf_tracer->set_per_cell_directions(N_, directions_.data());
        }

        void SetUp() override {
            ASSERT_TRUE(mesh_.load(std::string(TEST_DATA_PATH)+"three_holes_tets.geogram"));

            initialize_points();
            initialize_directions();
            initialize_tracer();
        }
    };

    TEST_F(StreamlineVolumetricTracerTest, constant_steps) {
        tracer->trace(30);
        save_result();
    }
}
