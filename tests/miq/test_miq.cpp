//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/19.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifdef GEOLIO_ENABLE_MIQ
#include <gtest/gtest.h>
#include <geolio/miq/miq_interface.h>
#include <geogram/mesh/mesh_frame_field.h>
#include "../utils.h"

namespace geolio::test
{
    class MIQTest : public ::testing::Test {
    protected:
        void SetUp() override {
            ASSERT_TRUE(mesh.load(std::string(TEST_DATA_PATH)+"fandisk.geogram"));
            mesh_fc_uv.bind(mesh.facet_corners.attributes(), "uv");
        }

        void save_results() {
            GEO::Mesh mesh_out(2);
            mesh_out.vertices.create_vertices(3*mesh.facets.nb());
            mesh_out.facets.create_triangles(mesh.facets.nb());
            for (const auto& f : mesh.facets) {
                mesh_out.vertices.point<2>(3*f)     = mesh_fc_uv[mesh.facets.corner(f, 0)];
                mesh_out.vertices.point<2>(3*f+1)   = mesh_fc_uv[mesh.facets.corner(f, 1)];
                mesh_out.vertices.point<2>(3*f+2)   = mesh_fc_uv[mesh.facets.corner(f, 2)];
                mesh_out.facets.set_vertex(f, 0, 3*f);
                mesh_out.facets.set_vertex(f, 1, 3*f+1);
                mesh_out.facets.set_vertex(f, 2, 3*f+2);
            }
            mesh_out.save(get_current_test_name()+".geogram");
        }

        GEO::Mesh mesh;
        GEO::Attribute<GEO::vec2> mesh_fc_uv;
    };

    TEST_F(MIQTest, nrosy) {
        miq<3>(mesh, mesh_fc_uv);
        save_results();
    }

    TEST_F(MIQTest, pre_compute_cross) {
        GEO::FrameField cross_field;
        cross_field.create_from_surface_mesh(mesh, false);

        MIQParameters params;
        params.cross = cross_field.frames().data();
        params.cross_dim = 9;

        miq<3>(mesh, mesh_fc_uv, params);
        save_results();
    }
}

#endif