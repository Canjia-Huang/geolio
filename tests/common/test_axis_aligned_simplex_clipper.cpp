//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/17.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <geogram/basic/geometry_nd.h>
#include <geogram/mesh/mesh.h>
#include <geolio/common/axis_aligned_simplex_clipper.h>
#include <geolio/common/vecg.h>
#include <geolio/mesh/tet_descriptor.h>
#include <gtest/gtest.h>
#include "../utils.h"

namespace
{
    const std::array<std::array<GEO::index_t, 3>, 3> TRI_VERTICES_ORDER = {
        {
            {0, 1, 2}, {1, 2, 0}, {2, 0, 1}
        }
    };

    const std::array<GEO::vec2, 3> TRI_VERTICES_2D = {
        {
            GEO::vec2(0, 2),
            GEO::vec2(-std::sqrt(3), -1),
            GEO::vec2(std::sqrt(3), -1)
        }
    };

    const std::array<GEO::vec3, 3> TRI_VERTICES_3D = {
        {
            GEO::vec3(0, 2, 1),
            GEO::vec3(-std::sqrt(3), -1, -1),
            GEO::vec3(std::sqrt(3), -1, 0)
        }
    };

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
    protected:
        void init(
            const GEO::vec3& p0,
            const GEO::vec3& p1,
            const GEO::vec3& p2,
            const GEO::vec3& p3
            ) {
            cut_planes.clear();

            origin_p0 = p0; origin_p1 = p1; origin_p2 = p2; origin_p3 = p3;
            clipper = std::make_unique<AxisAlignedTetClipper>(p0, p1, p2, p3);
            check_signed_volume();
        }

        void clip() {
            for (const auto& [d, t] : cut_planes)
                clipper->clip(d, t);
        }

        void check_volume_computation() const {
            ASSERT_EQ(cut_planes.size(), 1);
            const auto& [d, t] = cut_planes[0];

            double V, V0, V1;
            axis_aligned_tet_clipped_volumes(
                origin_p0, origin_p1, origin_p2, origin_p3,
                d, t,
                V, V0, V1);

            ASSERT_FALSE(clipper == nullptr);
            const auto& tet_coords = clipper->coords();
            const auto& partitions = clipper->partitions();
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

        void check_signed_volume() const {
            ASSERT_FALSE(clipper == nullptr);
            const auto& coords = clipper->coords();
            EXPECT_EQ(coords.size()%4, 0);

            for (GEO::index_t c = 0, c_end = coords.size()/4; c < c_end; ++c) {
                const auto& p0 = coords[4*c];
                const auto& p1 = coords[4*c+1];
                const auto& p2 = coords[4*c+2];
                const auto& p3 = coords[4*c+3];
                EXPECT_GE(GEO::Geom::tetra_signed_volume(p0, p1, p2, p3), -1e-10);
            }
        }

        void check_barycentric_coords() const {
            ASSERT_FALSE(clipper == nullptr);
            const auto& coords = clipper->coords();
            const auto& bary_coords = clipper->bary_coords();
            EXPECT_EQ(coords.size()%4, 0);
            EXPECT_EQ(bary_coords.size()%4, 0);
            EXPECT_EQ(coords.size(), bary_coords.size());

            for (GEO::index_t c = 0, c_end = coords.size()/4; c < c_end; ++c) {
                for (GEO::index_t lv = 0; lv < 4; ++lv) {
                    const auto& p = coords[4*c+lv];
                    const auto& bp = bary_coords[4*c+lv];
                    EXPECT_NEAR(GEO::distance(p, origin_p0*bp[0]+origin_p1*bp[1]+origin_p2*bp[2]+origin_p3*bp[3]), 0, 1e-10);
                }
            }
        }

        void check_partitions() const {
            ASSERT_FALSE(clipper == nullptr);
            const auto& coords = clipper->coords();
            const auto& partitions = clipper->partitions();
            EXPECT_EQ(coords.size()%4, 0);
            EXPECT_EQ(coords.size()/4, partitions.size());

            for (GEO::index_t c = 0, c_end = coords.size()/4; c < c_end; ++c) {
                /* A cell lying entirely in a cut plane has a zero measure, hence no side of its
                   own: `partitions()` reports the partition of the cell it comes from, which the
                   centroid rule below does not define (see the class documentation). */
                if (GEO::PCK::orient_3d(coords[4*c], coords[4*c+1], coords[4*c+2], coords[4*c+3]) == GEO::ZERO)
                    continue;

                const auto center = 0.25 * (coords[4*c]+coords[4*c+1]+coords[4*c+2]+coords[4*c+3]);

                for (GEO::index_t i = 0, i_end = cut_planes.size(); i < i_end; ++i) {
                    if (const auto& [d, t] = cut_planes[i];
                        center[d] > t)
                        EXPECT_TRUE(partitions[c] & (1<<i));
                    else
                        EXPECT_FALSE(partitions[c] & (1<<i));
                }
            }
        }

        /**
         * Checks that the cut plane bookkeeping agrees with the geometry: a facet reported as
         * lying in a cut plane must have all its vertices *exactly* on that plane.
         * @note For tetrahedra, `facet_cut_plane()[4*c+i]` describes the facet *opposite* the
         *  ith vertex, which is geogram's `MeshCells::facet()` convention.
         * @note Exactness matters here: this is what makes the cells sharing such a facet
         *  store bit identical vertices, hence a conforming partition without cracks.
         */
        void check_cut_plane_conformance() const {
            ASSERT_FALSE(clipper == nullptr);
            const auto& coords = clipper->coords();
            const auto& facet_cut_plane = clipper->facet_cut_plane();
            EXPECT_EQ(coords.size()%4, 0);
            EXPECT_EQ(facet_cut_plane.size(), coords.size());

            for (GEO::index_t c = 0, c_end = coords.size()/4; c < c_end; ++c) {
                for (GEO::index_t lv = 0; lv < 4; ++lv) {
                    const auto plane = facet_cut_plane[4*c+lv];
                    if (plane == GEO::NO_INDEX)
                        continue;

                    ASSERT_LT(plane, cut_planes.size());
                    const auto& [d, t] = cut_planes[plane];
                    for (GEO::index_t lv2 = 0; lv2 < 4; ++lv2) {
                        if (lv2 == lv)
                            continue;
                        EXPECT_EQ(coords[4*c+lv2][d], t);
                    }
                }
            }
        }

        /**
         * Checks that the degenerate cells are *exactly* degenerate, i.e. that they have
         * repeated vertices: this is what allows a caller to identify them exactly instead of
         * comparing a computed volume against a threshold.
         */
        void check_degenerate_cells_are_exact() const {
            ASSERT_FALSE(clipper == nullptr);
            const auto& coords = clipper->coords();
            EXPECT_EQ(coords.size()%4, 0);

            for (GEO::index_t c = 0, c_end = coords.size()/4; c < c_end; ++c) {
                if (GEO::PCK::orient_3d(coords[4*c], coords[4*c+1], coords[4*c+2], coords[4*c+3]) != GEO::ZERO)
                    continue;

                bool has_repeated_vertex = false;
                for (GEO::index_t lv = 0; lv < 4; ++lv)
                    for (GEO::index_t lv2 = lv+1; lv2 < 4; ++lv2)
                        if (GEO::distance(coords[4*c+lv], coords[4*c+lv2]) == 0)
                            has_repeated_vertex = true;
                EXPECT_TRUE(has_repeated_vertex);
            }
        }

        void output(
            const GEO::index_t i
            ) const {
            ASSERT_FALSE(clipper == nullptr);
            const auto& coords = clipper->coords();
            const auto& bary_coords = clipper->bary_coords();
            const auto& partitions = clipper->partitions();
            const auto& facet_cut_plane = clipper->facet_cut_plane();
            EXPECT_EQ(coords.size()%4, 0);
            EXPECT_EQ(partitions.size(), coords.size()/4);
            EXPECT_EQ(facet_cut_plane.size(), coords.size());

            GEO::Mesh mesh_out;
            {
                GEO::index_t new_v = mesh_out.vertices.create_vertices(coords.size());
                mesh_out.cells.create_tets(coords.size()/4);
                for (const auto& c : mesh_out.cells) {
                    for (GEO::index_t lv = 0; lv < 4; ++lv) {
                        mesh_out.vertices.point(new_v+lv) = coords[4*c+lv];
                        mesh_out.cells.set_vertex(c, lv, new_v+lv);
                    }
                    new_v += 4;
                }
            }
            {
                GEO::Attribute<double> mesh_out_cc_bary_coord;
                mesh_out_cc_bary_coord.create_vector_attribute(mesh_out.cell_corners.attributes(), "bary_coord", 4);
                for (const auto& c : mesh_out.cells) {
                    for (GEO::index_t lv = 0; lv < 4; ++lv) {
                        const auto& p = bary_coords[4*c+lv];
                        const auto& cc = mesh_out.cells.corner(c, lv);
                        mesh_out_cc_bary_coord[4*cc] = p[0];
                        mesh_out_cc_bary_coord[4*cc+1] = p[1];
                        mesh_out_cc_bary_coord[4*cc+2] = p[2];
                        mesh_out_cc_bary_coord[4*cc+3] = p[3];
                    }
                }
            }
            {
                GEO::Attribute<GEO::index_t> mesh_out_c_partitions(mesh_out.cells.attributes(), "partitions");
                for (const auto& c : mesh_out.cells)
                    mesh_out_c_partitions[c] = partitions[c];
            }
            {
                GEO::Attribute<GEO::index_t> mesh_out_cf_cut_plane(mesh_out.cell_facets.attributes(), "cut_plane");
                for (const auto& c : mesh_out.cells) {
                    for (GEO::index_t lf = 0; lf < 4; ++lf)
                        mesh_out_cf_cut_plane[mesh_out.cells.facet(c, lf)] = facet_cut_plane[4*c+lf];
                }
            }

            mesh_out.save("test_AxisAlignedTetClipperTest/" + get_current_test_name() + "_" + std::to_string(i) + ".geogram");
        }

        GEO::vec3 origin_p0, origin_p1, origin_p2, origin_p3;
        std::vector<std::pair<GEO::index_t, double>> cut_planes;
        std::unique_ptr<AxisAlignedTetClipper> clipper;
    };

    TEST_F(AxisAlignedTetClipperTest, cut_1_3_x) {
        for (GEO::index_t i = 0; i < TET_VERTICES_ORDER.size(); ++i) {
            const auto& lvs = TET_VERTICES_ORDER[i];
            init(TET_VERTICES[lvs[0]], TET_VERTICES[lvs[1]], TET_VERTICES[lvs[2]], TET_VERTICES[lvs[3]]);

            cut_planes.emplace_back(0, 1);
            clip();

            check_volume_computation();
            check_signed_volume();
            check_barycentric_coords();
            check_partitions();
            output(i);
        }
    }

    TEST_F(AxisAlignedTetClipperTest, cut_1_3_y) {
        for (GEO::index_t i = 0; i < TET_VERTICES_ORDER.size(); ++i) {
            const auto& lvs = TET_VERTICES_ORDER[i];
            init(TET_VERTICES[lvs[0]], TET_VERTICES[lvs[1]], TET_VERTICES[lvs[2]], TET_VERTICES[lvs[3]]);

            cut_planes.emplace_back(1, 0.8);
            clip();

            check_volume_computation();
            check_signed_volume();
            check_barycentric_coords();
            check_partitions();
            output(i);
        }
    }

    TEST_F(AxisAlignedTetClipperTest, cut_1_3_z) {
        for (GEO::index_t i = 0; i < TET_VERTICES_ORDER.size(); ++i) {
            const auto& lvs = TET_VERTICES_ORDER[i];
            init(TET_VERTICES[lvs[0]], TET_VERTICES[lvs[1]], TET_VERTICES[lvs[2]], TET_VERTICES[lvs[3]]);

            cut_planes.emplace_back(2, 1.8);
            clip();

            check_volume_computation();
            check_signed_volume();
            check_barycentric_coords();
            check_partitions();
            output(i);
        }
    }

    TEST_F(AxisAlignedTetClipperTest, cut_3_1_x) {
        for (GEO::index_t i = 0; i < TET_VERTICES_ORDER.size(); ++i) {
            const auto& lvs = TET_VERTICES_ORDER[i];
            init(TET_VERTICES[lvs[0]], TET_VERTICES[lvs[1]], TET_VERTICES[lvs[2]], TET_VERTICES[lvs[3]]);

            cut_planes.emplace_back(0, -1.5);
            clip();

            check_volume_computation();
            check_signed_volume();
            check_barycentric_coords();
            check_partitions();
            output(i);
        }
    }

    TEST_F(AxisAlignedTetClipperTest, cut_2_2_x) {
        for (GEO::index_t i = 0; i < TET_VERTICES_ORDER.size(); ++i) {
            const auto& lvs = TET_VERTICES_ORDER[i];
            init(TET_VERTICES[lvs[0]], TET_VERTICES[lvs[1]], TET_VERTICES[lvs[2]], TET_VERTICES[lvs[3]]);

            cut_planes.emplace_back(0, -0.5);
            clip();

            check_volume_computation();
            check_signed_volume();
            check_barycentric_coords();
            check_partitions();
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

            check_signed_volume();
            check_barycentric_coords();
            check_partitions();
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

            check_signed_volume();
            check_barycentric_coords();
            check_partitions();
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

            check_signed_volume();
            check_barycentric_coords();
            check_partitions();
            output(i);
        }
    }

    template <GEO::index_t DIM>
    class AxisAlignedTriClipperTest : public ::testing::Test {
    protected:
        void init(
            const GEO::vecng<DIM, double>& p0,
            const GEO::vecng<DIM, double>& p1,
            const GEO::vecng<DIM, double>& p2
            ) {
            cut_planes.clear();

            origin_p0 = p0; origin_p1 = p1; origin_p2 = p2;
            clipper = std::make_unique<AxisAlignedTriClipper<DIM>>(p0, p1, p2);
            check_signed_area();
        }

        void clip() {
            for (const auto& [d, t] : cut_planes)
                clipper->clip(d, t);
        }

        void check_area_computation() const {
            ASSERT_EQ(cut_planes.size(), 1);
            const auto& [d, t] = cut_planes[0];

            double V, V0, V1;
            axis_aligned_tri_clipped_areas<DIM>(
                origin_p0, origin_p1, origin_p2,
                d, t,
                V, V0, V1);

            ASSERT_FALSE(clipper == nullptr);
            const auto& coords = clipper->coords();
            const auto& partitions = clipper->partitions();
            EXPECT_EQ(coords.size()%3, 0);
            EXPECT_EQ(coords.size()/3, partitions.size());

            double exact_V{0}, exact_V0{0}, exact_V1{0};
            for (GEO::index_t f = 0, f_end = coords.size()/3; f < f_end; ++f) {
                const auto v = GEO::Geom::triangle_area(
                    coords[3*f].data(), coords[3*f+1].data(), coords[3*f+2].data(), DIM);
                exact_V += v;
                if (partitions[f] & 1)
                    exact_V1 += v;
                else
                    exact_V0 += v;
            }

            EXPECT_NEAR(V, exact_V, 1e-10);
            EXPECT_NEAR(V0, exact_V0, 1e-10);
            EXPECT_NEAR(V1, exact_V1, 1e-10);
        }

        void check_signed_area() const {
            ASSERT_FALSE(clipper == nullptr);
            const auto& coords = clipper->coords();
            EXPECT_EQ(coords.size()%3, 0);

            if constexpr (DIM == 2) {
                for (GEO::index_t f = 0, c_end = coords.size()/3; f < c_end; ++f) {
                    const auto& p0 = coords[3*f];
                    const auto& p1 = coords[3*f+1];
                    const auto& p2 = coords[3*f+2];
                    /* A cut plane going through a vertex or an edge produces degenerate
                       triangles: they are kept on purpose, and they are *exactly* degenerate. */
                    if (is_exactly_degenerate(p0, p1, p2))
                        EXPECT_EQ(geolio::cross(p1-p0, p2-p0), 0);
                    else
                        EXPECT_GE(geolio::cross(p1-p0, p2-p0), 1e-10);
                }
            }
        }

        void check_barycentric_coords() const {
            ASSERT_FALSE(clipper == nullptr);
            const auto& coords = clipper->coords();
            const auto& bary_coords = clipper->bary_coords();
            EXPECT_EQ(coords.size()%3, 0);
            EXPECT_EQ(bary_coords.size()%3, 0);
            EXPECT_EQ(coords.size(), bary_coords.size());

            for (GEO::index_t f = 0, f_end = coords.size()/3; f < f_end; ++f) {
                for (GEO::index_t lv = 0; lv < 3; ++lv) {
                    const auto& p = coords[3*f+lv];
                    const auto& bp = bary_coords[3*f+lv];
                    EXPECT_NEAR(GEO::distance(p, origin_p0*bp[0]+origin_p1*bp[1]+origin_p2*bp[2]), 0, 1e-10);
                }
            }
        }

        void check_partitions() const {
            ASSERT_FALSE(clipper == nullptr);
            const auto& coords = clipper->coords();
            const auto& partitions = clipper->partitions();
            EXPECT_EQ(coords.size()%3, 0);
            EXPECT_EQ(coords.size()/3, partitions.size());

            for (GEO::index_t f = 0, f_end = coords.size()/3; f < f_end; ++f) {
                /* @see check_partitions() of AxisAlignedTetClipperTest: a triangle lying
                   entirely in a cut plane has a zero area, hence no side of its own. */
                if (is_exactly_degenerate(coords[3*f], coords[3*f+1], coords[3*f+2]))
                    continue;

                const auto center = (coords[3*f]+coords[3*f+1]+coords[3*f+2]) / 3;

                for (GEO::index_t i = 0, i_end = cut_planes.size(); i < i_end; ++i) {
                    if (const auto& [d, t] = cut_planes[i];
                        center[d] > t)
                        EXPECT_TRUE(partitions[f] & (1<<i));
                    else
                        EXPECT_FALSE(partitions[f] & (1<<i));
                }
            }
        }

        /**
         * Checks that the cut plane bookkeeping agrees with the geometry: the edge reported as
         * lying in a cut plane must have both its vertices *exactly* on that plane.
         * @note For triangles, `facet_cut_plane()[3*f+i]` describes the edge `(i, (i+1)%3)`; for
         *  tetrahedra, entry i describes the facet opposite the ith vertex. The association is
         *  clipper specific, see the base class documentation.
         * @note Exactness matters here: this is what makes the cells sharing such an edge store
         *  bit identical vertices, hence a conforming partition without cracks.
         */
        void check_cut_plane_conformance() const {
            ASSERT_FALSE(clipper == nullptr);
            const auto& coords = clipper->coords();
            const auto& facet_cut_plane = clipper->facet_cut_plane();
            EXPECT_EQ(coords.size()%3, 0);
            EXPECT_EQ(facet_cut_plane.size(), coords.size());

            for (GEO::index_t f = 0, f_end = coords.size()/3; f < f_end; ++f) {
                for (GEO::index_t lv = 0; lv < 3; ++lv) {
                    const auto plane = facet_cut_plane[3*f+lv];
                    if (plane == GEO::NO_INDEX)
                        continue;

                    ASSERT_LT(plane, cut_planes.size());
                    const auto& [d, t] = cut_planes[plane];
                    EXPECT_EQ(coords[3*f+lv][d], t);
                    EXPECT_EQ(coords[3*f+(lv+1)%3][d], t);
                }
            }
        }

        /** @return true if the triangle `(p0, p1, p2)` has an exactly zero area. */
        static bool is_exactly_degenerate(
            const GEO::vecng<DIM, double>& p0,
            const GEO::vecng<DIM, double>& p1,
            const GEO::vecng<DIM, double>& p2
            ) {
            if constexpr (DIM == 2)
                return GEO::PCK::orient_2d(p0, p1, p2) == GEO::ZERO;
            else {
                /* A triangle of the 3d space has a zero area if and only if all of its three
                   coordinate projections are flat. */
                for (GEO::index_t ax = 0; ax < 3; ++ax) {
                    const auto i = (ax+1)%3, j = (ax+2)%3;
                    if (GEO::PCK::orient_2d(
                            GEO::vec2(p0[i], p0[j]), GEO::vec2(p1[i], p1[j]), GEO::vec2(p2[i], p2[j])
                            ) != GEO::ZERO)
                        return false;
                }
                return true;
            }
        }

        /**
         * Checks that the degenerate triangles are *exactly* degenerate, i.e. that they have
         * repeated vertices: this is what allows a caller to identify them exactly instead of
         * comparing a computed area against a threshold.
         */
        void check_degenerate_cells_are_exact() const {
            ASSERT_FALSE(clipper == nullptr);
            const auto& coords = clipper->coords();
            EXPECT_EQ(coords.size()%3, 0);

            for (GEO::index_t f = 0, f_end = coords.size()/3; f < f_end; ++f) {
                if (!is_exactly_degenerate(coords[3*f], coords[3*f+1], coords[3*f+2]))
                    continue;

                bool has_repeated_vertex = false;
                for (GEO::index_t lv = 0; lv < 3; ++lv)
                    for (GEO::index_t lv2 = lv+1; lv2 < 3; ++lv2)
                        if (GEO::distance(coords[3*f+lv], coords[3*f+lv2]) == 0)
                            has_repeated_vertex = true;
                EXPECT_TRUE(has_repeated_vertex);
            }
        }

        void output(
            const GEO::index_t i
            ) const {
            ASSERT_FALSE(clipper == nullptr);
            const auto& coords = clipper->coords();
            const auto& bary_coords = clipper->bary_coords();
            const auto& partitions = clipper->partitions();
            const auto& facet_cut_plane = clipper->facet_cut_plane();
            EXPECT_EQ(coords.size()%3, 0);
            EXPECT_EQ(partitions.size(), coords.size()/3);
            EXPECT_EQ(facet_cut_plane.size(), coords.size());

            GEO::Mesh mesh_out(DIM);
            {
                GEO::index_t new_v = mesh_out.vertices.create_vertices(coords.size());
                mesh_out.facets.create_triangles(coords.size()/3);
                for (const auto& f : mesh_out.facets) {
                    for (GEO::index_t lv = 0; lv < 3; ++lv) {
                        mesh_out.vertices.point<DIM>(new_v+lv) = coords[3*f+lv];
                        mesh_out.facets.set_vertex(f, lv, new_v+lv);
                    }
                    new_v += 3;
                }
            }
            {
                GEO::Attribute<double> mesh_out_fc_bary_coord;
                mesh_out_fc_bary_coord.create_vector_attribute(mesh_out.facet_corners.attributes(), "bary_coord", 3);
                for (const auto& f : mesh_out.facets) {
                    for (GEO::index_t lv = 0; lv < 3; ++lv) {
                        const auto& p = bary_coords[3*f+lv];
                        const auto& fc = mesh_out.facets.corner(f, lv);
                        mesh_out_fc_bary_coord[3*fc] = p[0];
                        mesh_out_fc_bary_coord[3*fc+1] = p[1];
                        mesh_out_fc_bary_coord[3*fc+2] = p[2];
                    }
                }
            }
            {
                GEO::Attribute<GEO::index_t> mesh_out_f_partitions(mesh_out.facets.attributes(), "partitions");
                for (const auto& f : mesh_out.facets)
                    mesh_out_f_partitions[f] = partitions[f];
            }
            {
                GEO::Attribute<GEO::index_t> mesh_out_fc_cut_plane(mesh_out.facet_corners.attributes(), "cut_plane");
                for (const auto& f : mesh_out.facets) {
                    for (GEO::index_t lv = 0; lv < 3; ++lv) {
                        /* `facet_cut_plane()[3*f+lv]` describes the edge `(lv, (lv+1)%3)`, while a
                           geogram facet corner is associated with the edge *opposite* its vertex:
                           the edge `(lv, (lv+1)%3)` is opposite the vertex `(lv+2)%3`. */
                        const GEO::index_t corner = (lv+2)%3;
                        mesh_out_fc_cut_plane[mesh_out.facets.corner(f, corner)] = facet_cut_plane[3*f+lv];
                    }
                }
            }

            mesh_out.save("test_AxisAlignedTriClipperTest/" + get_current_test_name() + "_" + std::to_string(i) + ".geogram");
        }

        GEO::vecng<DIM, double> origin_p0, origin_p1, origin_p2;
        std::vector<std::pair<GEO::index_t, double>> cut_planes;
        std::unique_ptr<AxisAlignedTriClipper<DIM>> clipper;
    };

    template <typename DimType>
    class AxisAlignedTriClipperDimTest : public AxisAlignedTriClipperTest<DimType::value> {};

    TYPED_TEST_SUITE(AxisAlignedTriClipperDimTest, DimTypes);

    TYPED_TEST(AxisAlignedTriClipperDimTest, cut_1_2_x) {
        constexpr GEO::index_t DIM = TypeParam::value;

        for (GEO::index_t i = 0; i < TRI_VERTICES_ORDER.size(); ++i) {
            const auto& lvs = TRI_VERTICES_ORDER[i];
            if constexpr (DIM == 2)
                this->init(TRI_VERTICES_2D[lvs[0]], TRI_VERTICES_2D[lvs[1]], TRI_VERTICES_2D[lvs[2]]);
            else
                this->init(TRI_VERTICES_3D[lvs[0]], TRI_VERTICES_3D[lvs[1]], TRI_VERTICES_3D[lvs[2]]);

            this->cut_planes.emplace_back(0, 1);
            this->clip();

            this->check_area_computation();
            this->check_signed_area();
            this->check_barycentric_coords();
            this->check_partitions();
            this->output(i);
        }
    }

    TYPED_TEST(AxisAlignedTriClipperDimTest, cut_1_2_y) {
        constexpr GEO::index_t DIM = TypeParam::value;

        for (GEO::index_t i = 0; i < TRI_VERTICES_ORDER.size(); ++i) {
            const auto& lvs = TRI_VERTICES_ORDER[i];
            if constexpr (DIM == 2)
                this->init(TRI_VERTICES_2D[lvs[0]], TRI_VERTICES_2D[lvs[1]], TRI_VERTICES_2D[lvs[2]]);
            else
                this->init(TRI_VERTICES_3D[lvs[0]], TRI_VERTICES_3D[lvs[1]], TRI_VERTICES_3D[lvs[2]]);

            this->cut_planes.emplace_back(1, -0.5);
            this->clip();

            this->check_area_computation();
            this->check_signed_area();
            this->check_barycentric_coords();
            this->check_partitions();
            this->output(i);
        }
    }

    TYPED_TEST(AxisAlignedTriClipperDimTest, cut_2_1_x) {
        constexpr GEO::index_t DIM = TypeParam::value;

        for (GEO::index_t i = 0; i < TRI_VERTICES_ORDER.size(); ++i) {
            const auto& lvs = TRI_VERTICES_ORDER[i];
            if constexpr (DIM == 2)
                this->init(TRI_VERTICES_2D[lvs[0]], TRI_VERTICES_2D[lvs[1]], TRI_VERTICES_2D[lvs[2]]);
            else
                this->init(TRI_VERTICES_3D[lvs[0]], TRI_VERTICES_3D[lvs[1]], TRI_VERTICES_3D[lvs[2]]);

            this->cut_planes.emplace_back(0, -1);
            this->clip();

            this->check_area_computation();
            this->check_signed_area();
            this->check_barycentric_coords();
            this->check_partitions();
            this->output(i);
        }
    }

    TYPED_TEST(AxisAlignedTriClipperDimTest, cut_z) {
        constexpr GEO::index_t DIM = TypeParam::value;
        if constexpr (DIM == 2)
            return;

        for (GEO::index_t i = 0; i < TRI_VERTICES_ORDER.size(); ++i) {
            const auto& lvs = TRI_VERTICES_ORDER[i];
            if constexpr (DIM == 2)
                this->init(TRI_VERTICES_2D[lvs[0]], TRI_VERTICES_2D[lvs[1]], TRI_VERTICES_2D[lvs[2]]);
            else
                this->init(TRI_VERTICES_3D[lvs[0]], TRI_VERTICES_3D[lvs[1]], TRI_VERTICES_3D[lvs[2]]);

            this->cut_planes.emplace_back(2, 0.1);
            this->clip();

            this->check_area_computation();
            this->check_signed_area();
            this->check_barycentric_coords();
            this->check_partitions();
            this->output(i);
        }
    }

    TYPED_TEST(AxisAlignedTriClipperDimTest, cut_2_planes) {
        constexpr GEO::index_t DIM = TypeParam::value;

        for (GEO::index_t i = 0; i < TRI_VERTICES_ORDER.size(); ++i) {
            const auto& lvs = TRI_VERTICES_ORDER[i];
            if constexpr (DIM == 2)
                this->init(TRI_VERTICES_2D[lvs[0]], TRI_VERTICES_2D[lvs[1]], TRI_VERTICES_2D[lvs[2]]);
            else
                this->init(TRI_VERTICES_3D[lvs[0]], TRI_VERTICES_3D[lvs[1]], TRI_VERTICES_3D[lvs[2]]);

            this->cut_planes.emplace_back(0, 0.2);
            this->cut_planes.emplace_back(1, -0.3);
            this->clip();

            this->check_signed_area();
            this->check_barycentric_coords();
            this->check_partitions();
            this->output(i);
        }
    }

    TYPED_TEST(AxisAlignedTriClipperDimTest, cut_3_planes) {
        constexpr GEO::index_t DIM = TypeParam::value;

        for (GEO::index_t i = 0; i < TRI_VERTICES_ORDER.size(); ++i) {
            const auto& lvs = TRI_VERTICES_ORDER[i];
            if constexpr (DIM == 2)
                this->init(TRI_VERTICES_2D[lvs[0]], TRI_VERTICES_2D[lvs[1]], TRI_VERTICES_2D[lvs[2]]);
            else
                this->init(TRI_VERTICES_3D[lvs[0]], TRI_VERTICES_3D[lvs[1]], TRI_VERTICES_3D[lvs[2]]);

            this->cut_planes.emplace_back(0, -0.4);
            this->cut_planes.emplace_back(1, 0.5);
            this->cut_planes.emplace_back(0, 0.1);
            this->clip();

            this->check_signed_area();
            this->check_barycentric_coords();
            this->check_partitions();
            this->output(i);
        }
    }

    TYPED_TEST(AxisAlignedTriClipperDimTest, cut_multi_planes) {
        constexpr GEO::index_t DIM = TypeParam::value;

        for (GEO::index_t i = 0; i < TRI_VERTICES_ORDER.size(); ++i) {
            const auto& lvs = TRI_VERTICES_ORDER[i];
            if constexpr (DIM == 2)
                this->init(TRI_VERTICES_2D[lvs[0]], TRI_VERTICES_2D[lvs[1]], TRI_VERTICES_2D[lvs[2]]);
            else
                this->init(TRI_VERTICES_3D[lvs[0]], TRI_VERTICES_3D[lvs[1]], TRI_VERTICES_3D[lvs[2]]);

            this->cut_planes.emplace_back(0, 0.4);
            this->cut_planes.emplace_back(1, 0.2);
            this->cut_planes.emplace_back(0, -0.1);
            this->cut_planes.emplace_back(0, -0.7);
            this->cut_planes.emplace_back(1, 2);
            this->cut_planes.emplace_back(1, -0.3);
            this->cut_planes.emplace_back(1, -2);
            this->cut_planes.emplace_back(0, 0.2);
            if constexpr (DIM == 3) {
                this->cut_planes.emplace_back(2, 0.2);
                this->cut_planes.emplace_back(2, 0.3);
            }
            this->clip();

            this->check_signed_area();
            this->check_barycentric_coords();
            this->check_partitions();
            this->output(i);
        }
    }

    /**
     * A cut plane that contains a whole face of the tetrahedron. Its vertices are then
     * classified as being on the negative side (closed negative half space), so the whole
     * tetrahedron belongs to the positive side and every generated cell is degenerate. Those
     * degenerate cells are kept on purpose: they are *exactly* degenerate (repeated vertices),
     * which lets a caller identify them without a threshold.
     */
    TEST_F(AxisAlignedTetClipperTest, cut_plane_through_a_face) {
        for (GEO::index_t i = 0; i < TET_VERTICES_ORDER.size(); ++i) {
            const auto& lvs = TET_VERTICES_ORDER[i];
            init(TET_VERTICES[lvs[0]], TET_VERTICES[lvs[1]], TET_VERTICES[lvs[2]], TET_VERTICES[lvs[3]]);

            /* TET_VERTICES[0], TET_VERTICES[1] and TET_VERTICES[2] all have z == 0. */
            cut_planes.emplace_back(2, 0.0);
            clip();

            check_signed_volume();
            check_barycentric_coords();
            check_partitions();
            check_cut_plane_conformance();
            check_degenerate_cells_are_exact();
        }
    }

    /**
     * Clipping a second time by a plane that was already used must not move anything: the
     * vertices generated by the first cut lie *exactly* on that plane (they are snapped onto
     * it), so no cell straddles it anymore, and the cells that the second cut generates are
     * exactly degenerate (zero volume) instead of being slivers carrying a wrong partition.
     */
    TEST_F(AxisAlignedTetClipperTest, duplicate_cut_plane_is_a_no_op) {
        for (GEO::index_t i = 0; i < TET_VERTICES_ORDER.size(); ++i) {
            const auto& lvs = TET_VERTICES_ORDER[i];
            init(TET_VERTICES[lvs[0]], TET_VERTICES[lvs[1]], TET_VERTICES[lvs[2]], TET_VERTICES[lvs[3]]);

            cut_planes.emplace_back(1, 0.5);
            clipper->clip(1, 0.5);
            check_signed_volume();
            check_cut_plane_conformance();

            const auto cells_nb = clipper->coords().size()/4;
            double volume_before = 0;
            for (GEO::index_t c = 0; c < cells_nb; ++c)
                volume_before += GEO::Geom::tetra_signed_volume(
                    clipper->coords()[4*c], clipper->coords()[4*c+1],
                    clipper->coords()[4*c+2], clipper->coords()[4*c+3]);

            cut_planes.emplace_back(1, 0.5);
            clipper->clip(1, 0.5);

            const auto& coords = clipper->coords();
            EXPECT_GT(coords.size()/4, cells_nb);
            double volume_after = 0;
            for (GEO::index_t c = 0, c_end = coords.size()/4; c < c_end; ++c) {
                const auto volume = GEO::Geom::tetra_signed_volume(coords[4*c], coords[4*c+1], coords[4*c+2], coords[4*c+3]);
                if (c >= cells_nb) {
                    /* every cell generated by the second cut carries no volume at all */
                    EXPECT_EQ(GEO::PCK::orient_3d(coords[4*c], coords[4*c+1], coords[4*c+2], coords[4*c+3]), GEO::ZERO);
                    EXPECT_EQ(volume, 0);
                }
                volume_after += volume;
            }
            /* and the second cut does not change the volume of the partition */
            EXPECT_NEAR(volume_before, volume_after, 1e-12);

            check_signed_volume();
            check_barycentric_coords();
            check_partitions();
            check_cut_plane_conformance();
            check_degenerate_cells_are_exact();
        }
    }

    /**
     * Regression test for the 2/2 configuration split when the cut plane goes exactly through
     * one or two vertices of the negative side (which then lie on the plane): the interpolated
     * points of the section collapse onto those vertices, and the generic decomposition of the
     * 2/2 branch produced grossly inverted tetrahedra (a whole tetrahedron volume with the
     * wrong sign) instead of degenerate ones, negating the volume of the positive partition.
     *
     * `TET_VERTICES[0]` and `TET_VERTICES[1]` both have y == -1, so the plane y == -1 contains
     * the whole edge that they form; `TET_VERTICES[0]` has x == -1, and the plane x == -1 goes
     * through that single vertex.
     */
    TEST_F(AxisAlignedTetClipperTest, cut_plane_through_simplex_vertices_in_a_2_2_split) {
        for (const auto& [d, t] : std::vector<std::pair<GEO::index_t, double>>{{1, -1.0}, {0, -1.0}}) {
            for (GEO::index_t i = 0; i < TET_VERTICES_ORDER.size(); ++i) {
                const auto& lvs = TET_VERTICES_ORDER[i];
                init(TET_VERTICES[lvs[0]], TET_VERTICES[lvs[1]], TET_VERTICES[lvs[2]], TET_VERTICES[lvs[3]]);

                cut_planes.emplace_back(d, t);
                clip();

                check_signed_volume();
                check_barycentric_coords();
                check_partitions();
                check_cut_plane_conformance();
                check_degenerate_cells_are_exact();
            }
        }
    }

    /**
     * Regression test for the selection of the decomposition inside the 2/2 branch: it is a
     * combinatorial property of the vertex selection (the parity of `(lv0, lv1, lv2, lv3)`),
     * and *not* the sign of the volume of the first generated cell, which decorrelates with it
     * as soon as the section is thin. With a cut plane one ulp away from a vertex, testing the
     * geometry emitted the mirrored decomposition: every cell inverted and the volume of each
     * partition negated (here for `(1,1,0)`, `(2,3,0)` and `(5,7,1)` which all have z == 0).
     */
    TEST_F(AxisAlignedTetClipperTest, cut_plane_one_ulp_away_from_a_vertex) {
        const GEO::vec3 p0(0, 0, 0), p1(1, 1, 0), p2(2, 3, 0), p3(5, 7, 1);
        for (const auto& [d, t] : std::vector<std::pair<GEO::index_t, double>>{
                {0, std::nextafter(1.0, 1e300)}, {1, std::nextafter(1.0, 1e300)}, {2, std::nextafter(1.0, 1e300)}}) {
            for (GEO::index_t i = 0; i < TET_VERTICES_ORDER.size(); ++i) {
                const auto& lvs = TET_VERTICES_ORDER[i];
                const std::array<GEO::vec3, 4> p = {p0, p1, p2, p3};
                init(p[lvs[0]], p[lvs[1]], p[lvs[2]], p[lvs[3]]);

                cut_planes.emplace_back(d, t);
                clip();

                check_signed_volume();
                check_barycentric_coords();
                check_partitions();
                check_cut_plane_conformance();
            }
        }
    }

    /**
     * @see AxisAlignedTetClipperTest::cut_plane_through_a_face() and
     *  AxisAlignedTetClipperTest::duplicate_cut_plane_is_a_no_op(), for triangles: the planes
     *  used here go exactly through a vertex, and through the edge `(v1, v2)` of
     *  `TRI_VERTICES_2D` and `TRI_VERTICES_3D`.
     */
    TYPED_TEST(AxisAlignedTriClipperDimTest, cut_plane_through_vertices) {
        constexpr GEO::index_t DIM = TypeParam::value;

        for (const auto& [d, t] : std::vector<std::pair<GEO::index_t, double>>{{0, 0.0}, {1, -1.0}}) {
            if (d >= DIM)
                continue;

            for (GEO::index_t i = 0; i < TRI_VERTICES_ORDER.size(); ++i) {
                const auto& lvs = TRI_VERTICES_ORDER[i];
                if constexpr (DIM == 2)
                    this->init(TRI_VERTICES_2D[lvs[0]], TRI_VERTICES_2D[lvs[1]], TRI_VERTICES_2D[lvs[2]]);
                else
                    this->init(TRI_VERTICES_3D[lvs[0]], TRI_VERTICES_3D[lvs[1]], TRI_VERTICES_3D[lvs[2]]);

                this->cut_planes.emplace_back(d, t);
                this->clip();

                this->check_area_computation();
                this->check_signed_area();
                this->check_barycentric_coords();
                this->check_partitions();
                this->check_cut_plane_conformance();
                this->check_degenerate_cells_are_exact();
            }
        }
    }

    TYPED_TEST(AxisAlignedTriClipperDimTest, duplicate_cut_plane_is_a_no_op) {
        constexpr GEO::index_t DIM = TypeParam::value;

        for (GEO::index_t i = 0; i < TRI_VERTICES_ORDER.size(); ++i) {
            const auto& lvs = TRI_VERTICES_ORDER[i];
            if constexpr (DIM == 2)
                this->init(TRI_VERTICES_2D[lvs[0]], TRI_VERTICES_2D[lvs[1]], TRI_VERTICES_2D[lvs[2]]);
            else
                this->init(TRI_VERTICES_3D[lvs[0]], TRI_VERTICES_3D[lvs[1]], TRI_VERTICES_3D[lvs[2]]);

            this->cut_planes.emplace_back(1, -0.5);
            this->clipper->clip(1, -0.5);
            this->check_signed_area();
            this->check_cut_plane_conformance();

            const auto cells_nb = this->clipper->coords().size()/3;
            double area_before = 0;
            for (GEO::index_t f = 0; f < cells_nb; ++f)
                area_before += GEO::Geom::triangle_area(
                    this->clipper->coords()[3*f].data(), this->clipper->coords()[3*f+1].data(),
                    this->clipper->coords()[3*f+2].data(), DIM);

            this->cut_planes.emplace_back(1, -0.5);
            this->clipper->clip(1, -0.5);

            const auto& coords = this->clipper->coords();
            EXPECT_GT(coords.size()/3, cells_nb);
            double area_after = 0;
            for (GEO::index_t f = 0, f_end = coords.size()/3; f < f_end; ++f) {
                const auto area = GEO::Geom::triangle_area(coords[3*f].data(), coords[3*f+1].data(), coords[3*f+2].data(), DIM);
                if (f >= cells_nb) {
                    /* every triangle generated by the second cut has a zero area */
                    EXPECT_TRUE(this->is_exactly_degenerate(coords[3*f], coords[3*f+1], coords[3*f+2]));
                    EXPECT_EQ(area, 0);
                }
                area_after += area;
            }
            /* and the second cut does not change the area of the partition */
            EXPECT_NEAR(area_before, area_after, 1e-12);

            this->check_signed_area();
            this->check_barycentric_coords();
            this->check_cut_plane_conformance();
            this->check_degenerate_cells_are_exact();
            /* @note `check_partitions()` is not called here: the triangles generated by the
               second cut lie *exactly* in a plane and inherit the partition of their parent,
               which the centroid rule of `check_partitions()` does not define. */
            this->output(i);
        }
    }
}
