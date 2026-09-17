//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/17.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <gtest/gtest.h>
#include <geolio/common/axis_aligned_simplex_clipper.h>
#include <geogram/mesh/mesh.h>
#include "../utils.h"
#include <geolio/mesh/tet_descriptor.h>

namespace
{
    const std::array<std::array<GEO::index_t, 4>, 12> TET_VERTICES_ORDER = {
        {
            {0, 1, 2, 3}, {0, 3, 1, 2}, {0, 2, 3, 1},
            {1, 0, 3, 2}, {1, 2, 0, 3}, {1, 3, 2, 0},
            {2, 0, 1, 3}, {2, 1, 3, 0}, {2, 3, 0, 1},
            {3, 0, 2, 1}, {3, 1, 0, 2}, {3, 2, 1, 0}
        }
    };

    const std::array<GEO::vec3, 4> TET_VERTICES = {
        {
                GEO::vec3(-std::sqrt(3), -1, 0),
                GEO::vec3(std::sqrt(3), -1, 0),
                GEO::vec3(0, 2, 0),
                GEO::vec3(-1, 0, 2)
        }
    };
}

namespace geolio::test
{
    class AxisAlignedTetClipperTest : public ::testing::Test {
    public:
        void init(
            const GEO::vec3& p0,
            const GEO::vec3& p1,
            const GEO::vec3& p2,
            const GEO::vec3& p3
            ) {
            cut_planes.clear();

            origin_p0 = p0; origin_p1 = p1; origin_p2 = p2; origin_p3 = p3;
            tet_clipper = std::make_unique<AxisAlignedTetClipper>(p0, p1, p2, p3);
            eval_signed_volume();
        }

        void clip() {
            for (const auto& [d, t] : cut_planes)
                tet_clipper->clip(d, t);
        }

        void eval_volume_computation() const {
            ASSERT_EQ(cut_planes.size(), 1);
            const auto& [d, t] = cut_planes[0];

            double V, V0, V1;
            axis_aligned_tet_clipped_volumes(
                origin_p0, origin_p1, origin_p2, origin_p3,
                d, t,
                V, V0, V1);

            ASSERT_FALSE(tet_clipper == nullptr);
            const auto& tet_coords = tet_clipper->tet_coords();
            const auto& partitions = tet_clipper->partitions();
            EXPECT_EQ(tet_coords.size()%4, 0);
            EXPECT_EQ(tet_coords.size()/4, partitions.size());

            double exact_V{0}, exact_V0{0}, exact_V1{0};
            for (GEO::index_t c = 0, c_end = tet_coords.size()/4; c < c_end; ++c) {
                const auto v = GEO::Geom::tetra_signed_volume(
                    tet_coords[4*c], tet_coords[4*c+1], tet_coords[4*c+2], tet_coords[4*c+3]);
                exact_V += v;
                if (partitions[c] & 1)
                    exact_V1 += v;
                else
                    exact_V0 += v;
            }

            EXPECT_NEAR(V, exact_V, 1e-10);
            EXPECT_NEAR(V0, exact_V0, 1e-10);
            EXPECT_NEAR(V1, exact_V1, 1e-10);
        }

        void eval_signed_volume() const {
            ASSERT_FALSE(tet_clipper == nullptr);
            const auto& tet_coords = tet_clipper->tet_coords();
            EXPECT_EQ(tet_coords.size()%4, 0);

            for (GEO::index_t c = 0, c_end = tet_coords.size()/4; c < c_end; ++c) {
                const auto& p0 = tet_coords[4*c];
                const auto& p1 = tet_coords[4*c+1];
                const auto& p2 = tet_coords[4*c+2];
                const auto& p3 = tet_coords[4*c+3];
                EXPECT_GE(GEO::Geom::tetra_signed_volume(p0, p1, p2, p3), -1e-10);
            }
        }

        void eval_barycentric_coords() const {
            ASSERT_FALSE(tet_clipper == nullptr);
            const auto& tet_coords = tet_clipper->tet_coords();
            const auto& bary_coords = tet_clipper->bary_coords();
            EXPECT_EQ(tet_coords.size()%4, 0);
            EXPECT_EQ(bary_coords.size()%4, 0);
            EXPECT_EQ(tet_coords.size(), bary_coords.size());

            for (GEO::index_t c = 0, c_end = tet_coords.size()/4; c < c_end; ++c) {
                for (GEO::index_t lv = 0; lv < 4; ++lv) {
                    const auto& p = tet_coords[4*c+lv];
                    const auto& bp = bary_coords[4*c+lv];
                    EXPECT_NEAR(GEO::distance(p, origin_p0*bp[0]+origin_p1*bp[1]+origin_p2*bp[2]+origin_p3*bp[3]), 0, 1e-10);
                }
            }
        }

        void eval_partitions() const {
            ASSERT_FALSE(tet_clipper == nullptr);
            const auto& tet_coords = tet_clipper->tet_coords();
            const auto& partitions = tet_clipper->partitions();
            EXPECT_EQ(tet_coords.size()%4, 0);
            EXPECT_EQ(tet_coords.size()/4, partitions.size());

            for (GEO::index_t c = 0, c_end = tet_coords.size()/4; c < c_end; ++c) {
                const auto center = 0.25 * (tet_coords[4*c]+tet_coords[4*c+1]+tet_coords[4*c+2]+tet_coords[4*c+3]);

                for (GEO::index_t i = 0, i_end = cut_planes.size(); i < i_end; ++i) {
                    if (const auto& [d, t] = cut_planes[i];
                        center[d] > t)
                        EXPECT_TRUE(partitions[c] & (1<<i));
                    else
                        EXPECT_FALSE(partitions[c] & (1<<i));
                }
            }
        }

        void output(
            const GEO::index_t i
            ) const {
            ASSERT_FALSE(tet_clipper == nullptr);
            const auto& tet_coords = tet_clipper->tet_coords();
            const auto& partitions = tet_clipper->partitions();
            const auto& tet_facet_cut_plane = tet_clipper->facet_cut_plane();
            EXPECT_EQ(tet_coords.size()%4, 0);
            EXPECT_EQ(partitions.size(), tet_coords.size()/4);
            EXPECT_EQ(tet_facet_cut_plane.size(), tet_coords.size());

            GEO::Mesh M_out;
            {
                GEO::Attribute<GEO::index_t> M_out_c_partitions(M_out.cells.attributes(), "partitions");
                GEO::index_t new_v = M_out.vertices.create_vertices(tet_coords.size());
                M_out.cells.create_tets(tet_coords.size()/4);
                for (const auto& c : M_out.cells) {
                    M_out_c_partitions[c] = partitions[c];
                    for (GEO::index_t lv = 0; lv < 4; ++lv) {
                        M_out.vertices.point(new_v+lv) = tet_coords[4*c+lv];
                        M_out.cells.set_vertex(c, lv, new_v+lv);
                    }
                    new_v += 4;
                }
            }
            {
                GEO::Attribute<GEO::index_t> M_out_f_cut_plane(M_out.facets.attributes(), "cut_plane");
                GEO::index_t new_v = M_out.vertices.create_vertices(tet_coords.size());
                M_out.facets.create_triangles(tet_coords.size());
                for (const auto& c : M_out.cells) {
                    const auto center = 0.25 * (
                        M_out.cells.point(c, 0) + M_out.cells.point(c, 1) + M_out.cells.point(c, 2) + M_out.cells.point(c, 3));
                    for (GEO::index_t lv = 0; lv < 4; ++lv) {
                        M_out.vertices.point(new_v+lv) = 0.9 * M_out.cells.point(c, lv) + 0.1 * center;
                        // M_out.vertices.point(new_v+lv) = M_out.cells.point(c, lv);
                    }
                    for (GEO::index_t lf = 0; lf < 4; ++lf) {
                        for (GEO::index_t lv = 0; lv < 3; ++lv)
                            M_out.facets.set_vertex(4*c+lf, lv, new_v+TET_LF_INCIDENT_LV[lf][lv]);
                        M_out_f_cut_plane[4*c+lf] = tet_facet_cut_plane[4*c+lf];
                    }
                    new_v += 4;
                }

                // delete other facets
                GEO::vector<GEO::index_t> facets_to_delete(M_out.facets.nb(), 0);
                for (const auto& f : M_out.facets) {
                    if (M_out_f_cut_plane[f] == GEO::NO_INDEX)
                        facets_to_delete[f] = 1;
                }
                M_out.facets.delete_elements(facets_to_delete);
            }

            M_out.save("test_AxisAlignedTetClipperTest/" + get_current_test_name() + "_" + std::to_string(i) + ".geogram");
        }

        GEO::vec3 origin_p0, origin_p1, origin_p2, origin_p3;
        std::vector<std::pair<GEO::index_t, double>> cut_planes;
        std::unique_ptr<AxisAlignedTetClipper> tet_clipper;
    };

    TEST_F(AxisAlignedTetClipperTest, cut_1_3_x) {
        for (GEO::index_t i = 0; i < TET_VERTICES_ORDER.size(); ++i) {
            const auto& lvs = TET_VERTICES_ORDER[i];
            init(TET_VERTICES[lvs[0]], TET_VERTICES[lvs[1]], TET_VERTICES[lvs[2]], TET_VERTICES[lvs[3]]);

            cut_planes.emplace_back(0, 1);
            clip();

            eval_volume_computation();
            eval_signed_volume();
            eval_barycentric_coords();
            eval_partitions();
            output(i);
        }
    }

    TEST_F(AxisAlignedTetClipperTest, cut_1_3_y) {
        for (GEO::index_t i = 0; i < TET_VERTICES_ORDER.size(); ++i) {
            const auto& lvs = TET_VERTICES_ORDER[i];
            init(TET_VERTICES[lvs[0]], TET_VERTICES[lvs[1]], TET_VERTICES[lvs[2]], TET_VERTICES[lvs[3]]);

            cut_planes.emplace_back(1, 0.8);
            clip();

            eval_volume_computation();
            eval_signed_volume();
            eval_barycentric_coords();
            eval_partitions();
            output(i);
        }
    }

    TEST_F(AxisAlignedTetClipperTest, cut_1_3_z) {
        for (GEO::index_t i = 0; i < TET_VERTICES_ORDER.size(); ++i) {
            const auto& lvs = TET_VERTICES_ORDER[i];
            init(TET_VERTICES[lvs[0]], TET_VERTICES[lvs[1]], TET_VERTICES[lvs[2]], TET_VERTICES[lvs[3]]);

            cut_planes.emplace_back(2, 1.8);
            clip();

            eval_volume_computation();
            eval_signed_volume();
            eval_barycentric_coords();
            eval_partitions();
            output(i);
        }
    }

    TEST_F(AxisAlignedTetClipperTest, cut_3_1_x) {
        for (GEO::index_t i = 0; i < TET_VERTICES_ORDER.size(); ++i) {
            const auto& lvs = TET_VERTICES_ORDER[i];
            init(TET_VERTICES[lvs[0]], TET_VERTICES[lvs[1]], TET_VERTICES[lvs[2]], TET_VERTICES[lvs[3]]);

            cut_planes.emplace_back(0, -1.5);
            clip();

            eval_volume_computation();
            eval_signed_volume();
            eval_barycentric_coords();
            eval_partitions();
            output(i);
        }
    }

    TEST_F(AxisAlignedTetClipperTest, cut_2_2_x) {
        for (GEO::index_t i = 0; i < TET_VERTICES_ORDER.size(); ++i) {
            const auto& lvs = TET_VERTICES_ORDER[i];
            init(TET_VERTICES[lvs[0]], TET_VERTICES[lvs[1]], TET_VERTICES[lvs[2]], TET_VERTICES[lvs[3]]);

            cut_planes.emplace_back(0, -0.5);
            clip();

            eval_volume_computation();
            eval_signed_volume();
            eval_barycentric_coords();
            eval_partitions();
            output(i);
        }
    }

    TEST_F(AxisAlignedTetClipperTest, cut_2_planes) {
        for (GEO::index_t i = 0; i < TET_VERTICES_ORDER.size(); ++i) {
            const auto& lvs = TET_VERTICES_ORDER[i];
            init(TET_VERTICES[lvs[0]], TET_VERTICES[lvs[1]], TET_VERTICES[lvs[2]], TET_VERTICES[lvs[3]]);

            cut_planes.emplace_back(0, -0.6);
            cut_planes.emplace_back(1, 0.5);
            clip();

            eval_signed_volume();
            eval_barycentric_coords();
            eval_partitions();
            output(i);
        }
    }

    TEST_F(AxisAlignedTetClipperTest, cut_3_planes) {
        for (GEO::index_t i = 0; i < TET_VERTICES_ORDER.size(); ++i) {
            const auto& lvs = TET_VERTICES_ORDER[i];
            init(TET_VERTICES[lvs[0]], TET_VERTICES[lvs[1]], TET_VERTICES[lvs[2]], TET_VERTICES[lvs[3]]);

            cut_planes.emplace_back(0, -0.6);
            cut_planes.emplace_back(1, 0.5);
            cut_planes.emplace_back(2, 0.2);
            clip();

            eval_signed_volume();
            eval_barycentric_coords();
            eval_partitions();
            output(i);
        }
    }

    TEST_F(AxisAlignedTetClipperTest, cut_multi_planes) {
        for (GEO::index_t i = 0; i < TET_VERTICES_ORDER.size(); ++i) {
            const auto& lvs = TET_VERTICES_ORDER[i];
            init(TET_VERTICES[lvs[0]], TET_VERTICES[lvs[1]], TET_VERTICES[lvs[2]], TET_VERTICES[lvs[3]]);

            cut_planes.emplace_back(0, -0.7);
            cut_planes.emplace_back(2, 0.9);
            cut_planes.emplace_back(2, 0.1);
            cut_planes.emplace_back(1, 0.3);
            cut_planes.emplace_back(0, 1.2);
            cut_planes.emplace_back(1, 0.5);
            clip();

            eval_signed_volume();
            eval_barycentric_coords();
            eval_partitions();
            output(i);
        }
    }
}
