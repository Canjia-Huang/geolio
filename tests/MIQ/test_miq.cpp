//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/19.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifdef GEOLIO_ENABLE_MIQ
#include <gtest/gtest.h>
#include <geolio/MIQ/miq_interface.h>
#include <geogram/mesh/mesh_frame_field.h>
#include "../utils.h"

namespace geolio::test
{
    class MIQTest : public ::testing::Test {
    protected:
        void save_results() {
            mesh.save(get_current_test_name()+".geogram");

            {
                GEO::Mesh mesh_out(2);
                mesh_out.vertices.create_vertices(3*mesh.facets.nb());
                mesh_out.facets.create_triangles(mesh.facets.nb());
                for (const auto& f : mesh.facets) {
                    for (GEO::index_t lv = 0; lv < 3; ++lv) {
                        const auto& fc = mesh.facets.corner(f, lv);
                        mesh_out.vertices.point<2>(3*f+lv) = GEO::vec2(mesh_fc_uv[2*fc], mesh_fc_uv[2*fc+1]);
                    }
                    mesh_out.facets.set_vertex(f, 0, 3*f);
                    mesh_out.facets.set_vertex(f, 1, 3*f+1);
                    mesh_out.facets.set_vertex(f, 2, 3*f+2);
                }
                mesh_out.save(get_current_test_name()+"_uv.geogram");
            }
        }

        GEO::Mesh mesh;
        GEO::Attribute<double> mesh_fc_uv;
        MIQParameters params;
    };

    class MIQExampleTest : public MIQTest {
    protected:
        void SetUp() override {
            ASSERT_TRUE(mesh.load(std::string(TEST_DATA_PATH)+"three_holes.geogram"));
            mesh_fc_uv.create_vector_attribute(mesh.facet_corners.attributes(), "tex_coord", 2);
        }
    };

    TEST_F(MIQExampleTest, nrosy) {
        miq<3>(mesh, mesh_fc_uv, params);
        save_results();
    }

    TEST_F(MIQExampleTest, pre_compute_cross) {
        GEO::FrameField cross_field;
        cross_field.create_from_surface_mesh(mesh, false);

        params.cross = cross_field.frames().data();
        params.vectors_nb_per_facet = 3;

        miq<3>(mesh, mesh_fc_uv, params);
        save_results();
    }

    class MIQOpenExampleTest : public MIQTest {
    protected:
        void SetUp() override {
            ASSERT_TRUE(mesh.load(std::string(TEST_DATA_PATH)+"cut_cylinder.geogram"));
            mesh_fc_uv.create_vector_attribute(mesh.facet_corners.attributes(), "tex_coord", 2);
        }
    };

    /**
     * @brief Open boundaries must end up on integer grid lines.
     *
     * MIQ cuts the mesh along seams derived from the singularities, but a mesh with an
     * open boundary is already cut by that boundary. Since no seam constraint exists
     * along a boundary, the boundary is only driven by the Poisson energy and lands at
     * arbitrary real coordinates unless it is explicitly snapped. `snap_boundary`
     * (default true) turns every boundary edge into a hard feature, i.e. forces it onto
     * an integer grid line. This test verifies exactly that property, and that turning
     * the option off restores the old, unsnapped behaviour.
     */
    TEST_F(MIQOpenExampleTest, boundary_on_integer_grid) {
        auto check_boundary = [&](const bool expect_snapped) {
            GEO::index_t nb_boundary_edges = 0;
            GEO::index_t nb_grid_aligned = 0;
            for (const auto& f : mesh.facets) {
                for (GEO::index_t lv = 0; lv < 3; ++lv) {
                    if (mesh.facets.adjacent(f, lv) != GEO::NO_FACET) continue;
                    ++nb_boundary_edges;
                    const GEO::index_t c0 = mesh.facets.corner(f, lv);
                    const GEO::index_t c1 = mesh.facets.corner(f, (lv + 1) % 3);
                    const double du = mesh_fc_uv[2*c1] - mesh_fc_uv[2*c0];
                    const double dv = mesh_fc_uv[2*c1+1] - mesh_fc_uv[2*c0+1];
                    const bool u_is_const = std::fabs(du) <= 1e-6;
                    const bool v_is_const = std::fabs(dv) <= 1e-6;
                    if (!u_is_const && !v_is_const) continue;
                    const double line = u_is_const ? mesh_fc_uv[2*c0] : mesh_fc_uv[2*c0+1];
                    if (std::fabs(line - std::round(line)) > 1e-6) continue;
                    ++nb_grid_aligned;
                }
            }
            EXPECT_GT(nb_boundary_edges, 0);
            if (expect_snapped) {
                EXPECT_EQ(nb_grid_aligned, nb_boundary_edges)
                    << "every boundary edge should lie on an integer grid line";
            } else {
                EXPECT_LT(nb_grid_aligned, nb_boundary_edges)
                    << "without snap_boundary the boundary is expected to miss the grid";
            }
        };

        params.snap_boundary = true;
        miq<3>(mesh, mesh_fc_uv, params);
        check_boundary(true);
        save_results();

        params.snap_boundary = false;
        miq<3>(mesh, mesh_fc_uv, params);
        check_boundary(false);
    }
}
#endif
