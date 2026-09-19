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
            mesh_fc_uv.create_vector_attribute(mesh.facet_corners.attributes(), "tex_coord", 2);
        }

        void save_results() {
            GEO::Mesh mesh_out(2);
            mesh_out.vertices.create_vertices(3*mesh.facets.nb());
            mesh_out.facets.create_triangles(mesh.facets.nb());
            for (const auto& f : mesh.facets) {
                for (GEO::index_t lv = 0; lv < 3; ++lv) {
                    const auto& fc = mesh.facets.corner(f, lv);
                    mesh_out.vertices.point<2>(3*f) = GEO::vec2(mesh_fc_uv[2*fc], mesh_fc_uv[2*fc+1]);
                }
                mesh_out.facets.set_vertex(f, 0, 3*f);
                mesh_out.facets.set_vertex(f, 1, 3*f+1);
                mesh_out.facets.set_vertex(f, 2, 3*f+2);
            }
            mesh_out.save(get_current_test_name()+".geogram");
        }

        GEO::Mesh mesh;
        GEO::Attribute<double> mesh_fc_uv;
        MIQParameters params;
    };

    TEST_F(MIQTest, nrosy) {
        miq<3>(mesh, mesh_fc_uv, params);
        save_results();
    }

    TEST_F(MIQTest, pre_compute_cross) {
        GEO::FrameField cross_field;
        cross_field.create_from_surface_mesh(mesh, false);

        params.cross = cross_field.frames().data();
        params.vectors_nb_per_facet = 3;

        miq<3>(mesh, mesh_fc_uv, params);
        save_results();
    }
}

#endif