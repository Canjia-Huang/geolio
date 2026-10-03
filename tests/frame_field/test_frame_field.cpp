//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/3.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>
#include <geogram/mesh/mesh_geometry.h>
#include <geogram/mesh/mesh_io.h>
#include <geolio/frame_field/frame_field.h>
#include <gtest/gtest.h>
#include "../utils.h"

// geolio::FrameField adds one entry point to the inherited ones, and the three ways of
// building a field differ in what they put in the frame vectors:
//
//   create_curvature_directions(M, true, autoscale)  curvature directions, sized
//   create_curvature_directions(M, false)            curvature directions, unit (the default)
//   create_from_surface_mesh(M, false)               the inherited cross field, unit
//
// The curvature magnitudes are private, so every size checked below is read back from the
// published frames: with autoscale = 0 the length of U is exactly the magnitude the estimator
// reported for that facet, and the length of V is the second one.

namespace geolio::test
{
    namespace
    {
        /**
         * @brief Extracts one of the three vectors of a frame.
         * @param field the frame field
         * @param f the facet index
         * @param axis 0 for U, 1 for V, 2 for W
         * @return the vector, with the length the field stores.
         */
        GEO::vec3 frame_vector(const GEO::FrameField& field, const GEO::index_t f, const GEO::index_t axis) {
            // GEO::FrameField::frames() returns GEO::vector<double>, a std::vector with a
            // different allocator, so the type has to be deduced rather than spelled out.
            const auto& frames = field.frames();
            return {frames[9 * f + 3 * axis], frames[9 * f + 3 * axis + 1], frames[9 * f + 3 * axis + 2]};
        }

        /**
         * @brief Collects the length of one frame vector, for every facet.
         * @param field the frame field
         * @param M the mesh the field was built from
         * @param axis 0 for U, 1 for V
         * @return the lengths, in facet order.
         */
        std::vector<double> frame_lengths(const GEO::FrameField& field, const GEO::Mesh& M, const GEO::index_t axis) {
            std::vector<double> lengths;
            for (const auto& f : M.facets) {
                lengths.push_back(GEO::length(frame_vector(field, f, axis)));
            }
            return lengths;
        }

        /**
         * @brief Largest value of a vector.
         * @param values the values
         * @return the largest one, or 0 for an empty vector.
         */
        double max_of(const std::vector<double>& values) {
            double result = 0.0;
            for (const double value : values) {
                result = std::max(result, value);
            }
            return result;
        }

        /**
         * @brief Mean of a vector of values.
         * @param values the values
         * @return the arithmetic mean, or 0 for an empty vector.
         */
        double mean_of(const std::vector<double>& values) {
            if (values.empty()) {
                return 0.0;
            }
            double sum = 0.0;
            for (const double value : values) {
                sum += value;
            }
            return sum / static_cast<double>(values.size());
        }

        /**
         * @brief Checks that the two in-plane directions lie in the facet plane.
         * @details This holds for every frame the class publishes: U and V are built by
         *          projecting eigenvectors onto the facet plane, so both are orthogonal to W by
         *          construction. Their mutual angle is another story, see
         *          expect_orthogonal_frame().
         * @param field the frame field
         * @param f the facet index
         */
        void expect_in_facet_plane(const GEO::FrameField& field, const GEO::index_t f) {
            const GEO::vec3 u = frame_vector(field, f, 0);
            const GEO::vec3 v = frame_vector(field, f, 1);
            const GEO::vec3 w = frame_vector(field, f, 2);

            ASSERT_GT(GEO::length(u), 1e-12);
            ASSERT_GT(GEO::length(v), 1e-12);
            ASSERT_GT(GEO::length(w), 1e-12);

            EXPECT_NEAR(GEO::dot(GEO::normalize(u), GEO::normalize(w)), 0.0, 1e-12);
            EXPECT_NEAR(GEO::dot(GEO::normalize(v), GEO::normalize(w)), 0.0, 1e-12);
        }

        /**
         * @brief Checks that U and V are perpendicular.
         * @details Only meaningful where both principal curvatures are well conditioned: where
         *          the minimum curvature vanishes the direction of the second curvature line is
         *          not defined by the data, and the projector can return any direction,
         *          including one parallel to U. See the documentation of geolio::FrameField.
         * @param field the frame field
         * @param f the facet index
         * @param tolerance the bound on |U.V|, i.e. on the cosine of the angle
         */
        void expect_orthogonal_frame(const GEO::FrameField& field, const GEO::index_t f, const double tolerance) {
            expect_in_facet_plane(field, f);
            const GEO::vec3 u = GEO::normalize(frame_vector(field, f, 0));
            const GEO::vec3 v = GEO::normalize(frame_vector(field, f, 1));
            EXPECT_LT(::fabs(GEO::dot(u, v)), tolerance);
        }

        /**
         * @brief Tells whether every frame of a field is orthonormal.
         * @details What the inherited cross field publishes, and what a sized field does not.
         * @param field the frame field
         * @param M the mesh the field was built from
         * @return true if every frame vector has unit length.
         */
        bool all_frames_are_unit(const GEO::FrameField& field, const GEO::Mesh& M) {
            if (field.frames().size() != 9 * M.facets.nb()) {
                return false;
            }
            for (const auto& f : M.facets) {
                for (GEO::index_t axis = 0; axis < 3; ++axis) {
                    if (::fabs(GEO::length(frame_vector(field, f, axis)) - 1.0) > 1e-12) {
                        return false;
                    }
                }
            }
            return true;
        }

        /**
         * @brief Builds a flat grid in the z = 0 plane.
         * @param[out] M the mesh to fill; it is cleared first
         * @param[in] n number of quads along each side
         */
        void make_plane(GEO::Mesh& M, const GEO::index_t n) {
            M.clear();
            M.vertices.create_vertices((n + 1) * (n + 1));
            for (GEO::index_t i = 0; i <= n; ++i) {
                for (GEO::index_t j = 0; j <= n; ++j) {
                    M.vertices.point<3>(i * (n + 1) + j) = GEO::vec3(
                        static_cast<double>(i) / static_cast<double>(n),
                        static_cast<double>(j) / static_cast<double>(n),
                        0.0
                    );
                }
            }

            M.facets.create_triangles(2 * n * n);
            for (GEO::index_t i = 0; i < n; ++i) {
                for (GEO::index_t j = 0; j < n; ++j) {
                    const GEO::index_t v00 = i * (n + 1) + j;
                    const GEO::index_t v01 = i * (n + 1) + j + 1;
                    const GEO::index_t v10 = (i + 1) * (n + 1) + j;
                    const GEO::index_t v11 = (i + 1) * (n + 1) + j + 1;
                    const GEO::index_t f = 2 * (i * n + j);
                    M.facets.set_vertex(f, 0, v00);
                    M.facets.set_vertex(f, 1, v10);
                    M.facets.set_vertex(f, 2, v11);
                    M.facets.set_vertex(f + 1, 0, v00);
                    M.facets.set_vertex(f + 1, 1, v11);
                    M.facets.set_vertex(f + 1, 2, v01);
                }
            }

            M.facets.connect();
        }

        /**
         * @brief Builds a sphere centered at the origin, with outward-facing triangles.
         * @param[out] M the mesh to fill; it is cleared first
         * @param[in] radius the radius of the sphere
         * @param[in] n number of segments around the axis
         * @param[in] rows number of latitude bands
         */
        void make_sphere(GEO::Mesh& M, const double radius,
                         const GEO::index_t n, const GEO::index_t rows) {
            const double pi = std::acos(-1.0);

            M.clear();
            M.vertices.create_vertices(1 + (rows - 1) * n + 1);
            M.vertices.point<3>(0) = GEO::vec3(0.0, 0.0, radius);
            for (GEO::index_t i = 1; i < rows; ++i) {
                const double theta = pi * static_cast<double>(i) / static_cast<double>(rows);
                for (GEO::index_t seg = 0; seg < n; ++seg) {
                    const double phi = 2.0 * pi * static_cast<double>(seg) / static_cast<double>(n);
                    M.vertices.point<3>(1 + (i - 1) * n + seg) = GEO::vec3(
                        radius * std::sin(theta) * std::cos(phi),
                        radius * std::sin(theta) * std::sin(phi),
                        radius * std::cos(theta)
                    );
                }
            }
            const GEO::index_t south = 1 + (rows - 1) * n;
            M.vertices.point<3>(south) = GEO::vec3(0.0, 0.0, -radius);

            const auto ring = [n](const GEO::index_t i) { return 1 + (i - 1) * n; };
            const GEO::index_t north = 0;

            M.facets.create_triangles(n + 2 * n * (rows - 2) + n);
            GEO::index_t f = 0;
            for (GEO::index_t seg = 0; seg < n; ++seg) {
                M.facets.set_vertex(f, 0, north);
                M.facets.set_vertex(f, 1, ring(1) + seg);
                M.facets.set_vertex(f, 2, ring(1) + (seg + 1) % n);
                ++f;
            }
            for (GEO::index_t i = 1; i + 1 < rows; ++i) {
                for (GEO::index_t seg = 0; seg < n; ++seg) {
                    const GEO::index_t s1 = (seg + 1) % n;
                    const GEO::index_t a = ring(i) + seg;
                    const GEO::index_t b = ring(i) + s1;
                    const GEO::index_t c = ring(i + 1) + s1;
                    const GEO::index_t d = ring(i + 1) + seg;
                    M.facets.set_vertex(f, 0, a);
                    M.facets.set_vertex(f, 1, d);
                    M.facets.set_vertex(f, 2, c);
                    ++f;
                    M.facets.set_vertex(f, 0, a);
                    M.facets.set_vertex(f, 1, c);
                    M.facets.set_vertex(f, 2, b);
                    ++f;
                }
            }
            for (GEO::index_t seg = 0; seg < n; ++seg) {
                M.facets.set_vertex(f, 0, south);
                M.facets.set_vertex(f, 1, ring(rows - 1) + (seg + 1) % n);
                M.facets.set_vertex(f, 2, ring(rows - 1) + seg);
                ++f;
            }
            EXPECT_EQ(f, M.facets.nb());

            M.facets.connect();
        }

        /**
         * @brief Builds a cylinder whose axis is z, with outward-facing triangles.
         * @details Used by the diagnostic below only: the long axial diagonals of this striped
         *          triangulation are exactly what makes the estimator report a direction that
         *          is not a curvature direction.
         * @param[out] M the mesh to fill; it is cleared first
         * @param[in] radius the radius of the tube
         * @param[in] height the height of the tube
         * @param[in] n number of segments around the axis
         * @param[in] m number of rows along the axis
         */
        void make_cylinder(GEO::Mesh& M, const double radius, const double height,
                           const GEO::index_t n, const GEO::index_t m) {
            const double pi = std::acos(-1.0);

            M.clear();
            M.vertices.create_vertices((m + 1) * n);
            for (GEO::index_t row = 0; row <= m; ++row) {
                for (GEO::index_t seg = 0; seg < n; ++seg) {
                    const double a = 2.0 * pi * static_cast<double>(seg) / static_cast<double>(n);
                    M.vertices.point<3>(row * n + seg) = GEO::vec3(
                        radius * std::cos(a), radius * std::sin(a),
                        height * static_cast<double>(row) / static_cast<double>(m)
                    );
                }
            }

            M.facets.create_triangles(2 * n * m);
            for (GEO::index_t row = 0; row < m; ++row) {
                for (GEO::index_t seg = 0; seg < n; ++seg) {
                    const GEO::index_t s1 = (seg + 1) % n;
                    const GEO::index_t v00 = row * n + seg;
                    const GEO::index_t v01 = row * n + s1;
                    const GEO::index_t v10 = (row + 1) * n + seg;
                    const GEO::index_t v11 = (row + 1) * n + s1;
                    const GEO::index_t f = 2 * (row * n + seg);
                    M.facets.set_vertex(f, 0, v00);
                    M.facets.set_vertex(f, 1, v01);
                    M.facets.set_vertex(f, 2, v11);
                    M.facets.set_vertex(f + 1, 0, v00);
                    M.facets.set_vertex(f + 1, 1, v11);
                    M.facets.set_vertex(f + 1, 2, v10);
                }
            }

            M.facets.connect();
        }

        /**
         * @brief Builds a torus around the z axis, with outward-facing triangles.
         * @details Used by the diagnostic below only. Each quad is split around its own
         *          centroid when \p regular is true, which keeps the triangles roughly
         *          isotropic; the striped split otherwise has long diagonal edges. Both give
         *          the same (wrong) direction, which is the point of the diagnostic: the tensor
         *          weights a direction by the square of the edge length, so the sampling, not
         *          the geometry, can decide the ordering.
         * @param[out] M the mesh to fill; it is cleared first
         * @param[in] big_radius distance from the axis to the center of the tube
         * @param[in] small_radius radius of the tube
         * @param[in] nu number of segments around the axis
         * @param[in] nv number of segments around the tube
         * @param[in] regular if true, union jack triangulation, else striped
         */
        void make_torus(GEO::Mesh& M, const double big_radius, const double small_radius,
                        const GEO::index_t nu, const GEO::index_t nv, const bool regular) {
            const double pi = std::acos(-1.0);

            const auto point = [&](const double u, const double v) {
                const double rr = big_radius + small_radius * std::cos(v);
                return GEO::vec3(rr * std::cos(u), rr * std::sin(u), small_radius * std::sin(v));
            };

            M.clear();
            const GEO::index_t nb_grid = nu * nv;
            M.vertices.create_vertices(regular ? 2 * nb_grid : nb_grid);
            for (GEO::index_t i = 0; i < nu; ++i) {
                const double u = 2.0 * pi * static_cast<double>(i) / static_cast<double>(nu);
                for (GEO::index_t j = 0; j < nv; ++j) {
                    const double v = 2.0 * pi * static_cast<double>(j) / static_cast<double>(nv);
                    M.vertices.point<3>(i * nv + j) = point(u, v);
                    if (regular) {
                        M.vertices.point<3>(nb_grid + i * nv + j) = point(
                            u + pi / static_cast<double>(nu), v + pi / static_cast<double>(nv)
                        );
                    }
                }
            }

            M.facets.create_triangles(regular ? 4 * nb_grid : 2 * nb_grid);
            GEO::index_t f = 0;
            for (GEO::index_t i = 0; i < nu; ++i) {
                const GEO::index_t i1 = (i + 1) % nu;
                for (GEO::index_t j = 0; j < nv; ++j) {
                    const GEO::index_t j1 = (j + 1) % nv;
                    const GEO::index_t a = i * nv + j;
                    const GEO::index_t b = i * nv + j1;
                    const GEO::index_t c = i1 * nv + j1;
                    const GEO::index_t d = i1 * nv + j;
                    if (regular) {
                        const GEO::index_t m = nb_grid + i * nv + j;
                        const GEO::index_t corner[4][3] = {{a, b, m}, {b, c, m}, {c, d, m}, {d, a, m}};
                        for (const auto& tri : corner) {
                            M.facets.set_vertex(f, 0, tri[0]);
                            M.facets.set_vertex(f, 1, tri[1]);
                            M.facets.set_vertex(f, 2, tri[2]);
                            ++f;
                        }
                    } else {
                        M.facets.set_vertex(f, 0, a);
                        M.facets.set_vertex(f, 1, b);
                        M.facets.set_vertex(f, 2, c);
                        ++f;
                        M.facets.set_vertex(f, 0, a);
                        M.facets.set_vertex(f, 1, c);
                        M.facets.set_vertex(f, 2, d);
                        ++f;
                    }
                }
            }

            M.facets.connect();
        }
    }

    class FrameFieldTest : public ::testing::Test {
    protected:
        void SetUp() override {
            ASSERT_TRUE(mesh.load(std::string(TEST_DATA_PATH) + "fandisk.geogram"));
            ASSERT_GT(mesh.facets.nb(), 0);
            ASSERT_TRUE(mesh.facets.are_simplices());

            // geogram::mesh_load() skips facets.connect() for the .geogram format, while the
            // curvature estimator reads dihedral angles through
            // facet_corners.adjacent_facet(), which is what connect() fills in.
            mesh.facets.connect();
        }

        GEO::Mesh mesh;
    };

    /**
     * @brief The field carries the curvature directions: W is the facet normal, and both
     *        in-plane vectors lie in the facet plane.
     */
    TEST_F(FrameFieldTest, directions_are_the_curvature_lines) {
        FrameField field;
        field.create_curvature_directions(mesh, 45.0, 0.0);

        ASSERT_EQ(field.frames().size(), 9 * mesh.facets.nb());
        for (const auto& f : mesh.facets) {
            expect_in_facet_plane(field, f);

            // W is the facet normal, and it points the same way.
            EXPECT_NEAR(
                GEO::distance(frame_vector(field, f, 2), GEO::normalize(GEO::Geom::mesh_facet_normal(mesh, f))),
                0.0, 1e-12
            );
        }
    }

    /**
     * @brief The published lengths are the curvature magnitudes: non-negative, the larger one
     *        first, W left unit.
     */
    TEST_F(FrameFieldTest, sizes_are_the_curvature_magnitudes) {
        FrameField field;
        field.create_curvature_directions(mesh, 45.0, 0.0);

        const std::vector<double> lengths_u = frame_lengths(field, mesh, 0);
        const std::vector<double> lengths_v = frame_lengths(field, mesh, 1);
        ASSERT_EQ(lengths_u.size(), mesh.facets.nb());
        ASSERT_EQ(lengths_v.size(), mesh.facets.nb());
        EXPECT_GT(max_of(lengths_u), 0.0);

        for (const auto& f : mesh.facets) {
            EXPECT_GE(lengths_u[f], 0.0);
            EXPECT_GE(lengths_v[f], 0.0);
            // U carries the larger of the two magnitudes. Exact equality happens - an umbilic
            // facet, or a flat one where the estimator's 1e-6 regularization makes all the
            // eigenvalues the same - and the two projected directions then round differently in
            // their last bits, hence the relative tolerance.
            EXPECT_LE(lengths_v[f], lengths_u[f] * (1.0 + 1e-9));
            EXPECT_NEAR(GEO::length(frame_vector(field, f, 2)), 1.0, 1e-12);
        }
    }

    /**
     * @brief A positive autoscale divides every size by the largest magnitude of the mesh and
     *        multiplies it by autoscale, so the largest U becomes exactly autoscale.
     */
    TEST_F(FrameFieldTest, autoscale_normalizes_the_largest_size) {
        FrameField raw;
        raw.create_curvature_directions(mesh, 45.0, 0.0);
        const std::vector<double> raw_u = frame_lengths(raw, mesh, 0);
        const std::vector<double> raw_v = frame_lengths(raw, mesh, 1);
        const double max_raw_u = max_of(raw_u);
        ASSERT_GT(max_raw_u, 0.0);

        constexpr double autoscale = 2.0;
        FrameField scaled;
        scaled.create_curvature_directions(mesh, 45.0, autoscale);
        const std::vector<double> scaled_u = frame_lengths(scaled, mesh, 0);
        const std::vector<double> scaled_v = frame_lengths(scaled, mesh, 1);

        for (const auto& f : mesh.facets) {
            EXPECT_NEAR(scaled_u[f], autoscale * raw_u[f] / max_raw_u, 1e-12);
            // V is divided by the same reference, the largest magnitude of U, not the largest
            // magnitude of V.
            EXPECT_NEAR(scaled_v[f], autoscale * raw_v[f] / max_raw_u, 1e-12);
        }
        EXPECT_NEAR(max_of(scaled_u), autoscale, 1e-12);
    }

    /**
     * @brief On a sphere the field is the one the estimator is exact on: the two curvatures are
     *        equal, the frame is perpendicular, and the sizes follow the measure semantics.
     * @details This is also where the meaning of the magnitude is pinned down: it is not a
     *          curvature but a curvature @b measure, proportional to the local facet area, hence
     *          divided by ~4.1 when the sphere is refined by 2 in both directions.
     */
    TEST_F(FrameFieldTest, sphere_is_the_well_conditioned_case) {
        constexpr double radius = 2.0;

        GEO::Mesh coarse;
        make_sphere(coarse, radius, 32, 16);
        FrameField on_coarse;
        on_coarse.create_curvature_directions(coarse, true, 0.0);

        std::vector<double> measure_ratio_coarse;
        std::vector<double> areas_coarse;
        for (const auto& f : coarse.facets) {
            const double area = GEO::Geom::mesh_facet_area(coarse, f, 3);
            const double length_u = GEO::length(frame_vector(on_coarse, f, 0));
            const double length_v = GEO::length(frame_vector(on_coarse, f, 1));
            // A sphere is umbilic: the two curvatures coincide.
            EXPECT_GT(length_v, 0.7 * length_u);
            // |U| = 12 * kappa * area, where the 12 is the counting convention of the estimator
            // (three vertices per facet, each accumulating its incident edges once per
            // endpoint). The scatter of a few tens of percent is the sampling.
            measure_ratio_coarse.push_back(length_u / (12.0 * (1.0 / radius) * area));
            areas_coarse.push_back(area);
            expect_orthogonal_frame(on_coarse, f, 5e-2);
        }
        EXPECT_NEAR(mean_of(measure_ratio_coarse), 1.0, 0.15);

        // Refined by a factor 2 in both directions: the magnitudes follow the facet area.
        GEO::Mesh fine;
        make_sphere(fine, radius, 64, 32);
        FrameField on_fine;
        on_fine.create_curvature_directions(fine, true, 0.0);

        std::vector<double> measure_ratio_fine;
        std::vector<double> areas_fine;
        for (const auto& f : fine.facets) {
            const double area = GEO::Geom::mesh_facet_area(fine, f, 3);
            measure_ratio_fine.push_back(GEO::length(frame_vector(on_fine, f, 0)) / (12.0 * (1.0 / radius) * area));
            areas_fine.push_back(area);
            expect_orthogonal_frame(on_fine, f, 5e-2);
        }
        EXPECT_NEAR(mean_of(measure_ratio_fine), 1.0, 0.15);

        const double refinement = mean_of(areas_coarse) / mean_of(areas_fine);
        EXPECT_NEAR(refinement, 4.0, 0.2);
        EXPECT_NEAR(
            mean_of(measure_ratio_coarse) / mean_of(measure_ratio_fine), 1.0, 0.1
        ) << "the magnitude is a measure: it must scale with the facet area";
    }

    /**
     * @brief A flat mesh has no size to preserve: the estimated magnitudes are zero up to the
     *        regularization of the estimator, so the sizes are numerically zero.
     */
    TEST_F(FrameFieldTest, flat_mesh_has_numerically_zero_sizes) {
        GEO::Mesh plane;
        make_plane(plane, 4);

        FrameField field;
        field.create_curvature_directions(plane, true, 0.0);

        ASSERT_EQ(field.frames().size(), 9 * plane.facets.nb());
        for (const auto& f : plane.facets) {
            // The estimator adds 1e-6 to the diagonal of a zero tensor, so "no curvature" comes
            // back as a magnitude of that order rather than as exactly zero.
            EXPECT_LT(GEO::length(frame_vector(field, f, 0)), 1e-5);
            EXPECT_LT(GEO::length(frame_vector(field, f, 1)), 1e-5);
            EXPECT_NEAR(GEO::length(frame_vector(field, f, 2)), 1.0, 1e-12);
        }
    }

    /**
     * @brief The inherited nearest-frame queries answer with the published frames, which is
     *        what the storage shared with the base class is for.
     */
    TEST_F(FrameFieldTest, inherited_queries_agree_with_the_published_frames) {
        FrameField field;
        field.create_curvature_directions(mesh, 45.0, 1.0);

        for (const auto& f : mesh.facets) {
            const GEO::vec3 center = GEO::Geom::mesh_facet_center(mesh, f);
            double queried[9];
            field.get_nearest_frame(center.data(), queried);
            for (GEO::index_t c = 0; c < 9; ++c) {
                EXPECT_NEAR(queried[c], field.frames()[9 * f + c], 1e-9);
            }
        }
    }

    TEST(FrameFieldExampleTest, example) {
        GEO::Mesh mesh;
        ASSERT_TRUE(mesh.load(std::string(TEST_DATA_PATH)+"three_holes.geogram"));

        FrameField frame_field;
        frame_field.create_curvature_directions(mesh, 45.0);
        const auto& frames = frame_field.frames();
        ASSERT_EQ(frames.size(), 9*mesh.facets.nb());

        /* Append frame field */
        GEO::Attribute<GEO::index_t> mesh_e_axis(mesh.edges.attributes(), "axis");
        GEO::index_t new_v = mesh.vertices.create_vertices(6*mesh.facets.nb());
        GEO::index_t new_e = mesh.edges.create_edges(3*mesh.facets.nb());
        for (const auto& f : mesh.facets) {
            const GEO::vec3 center = (mesh.facets.point(f, 0)+mesh.facets.point(f, 1)+mesh.facets.point(f, 2))/3;
            mesh.vertices.point(new_v)   = center + GEO::vec3(frames[9*f], frames[9*f+1], frames[9*f+2]);
            mesh.vertices.point(new_v+1) = center - GEO::vec3(frames[9*f], frames[9*f+1], frames[9*f+2]);
            mesh.vertices.point(new_v+2) = center + GEO::vec3(frames[9*f+3], frames[9*f+4], frames[9*f+5]);
            mesh.vertices.point(new_v+3) = center - GEO::vec3(frames[9*f+3], frames[9*f+4], frames[9*f+5]);
            mesh.vertices.point(new_v+4) = center + GEO::vec3(frames[9*f+6], frames[9*f+7], frames[9*f+8]);
            mesh.vertices.point(new_v+5) = center - GEO::vec3(frames[9*f+6], frames[9*f+7], frames[9*f+8]);
            mesh.edges.set_vertex(new_e, 0, new_v);
            mesh.edges.set_vertex(new_e, 1, new_v+1);
            mesh.edges.set_vertex(new_e+1, 0, new_v+2);
            mesh.edges.set_vertex(new_e+1, 1, new_v+3);
            mesh.edges.set_vertex(new_e+2, 0, new_v+4);
            mesh.edges.set_vertex(new_e+2, 1, new_v+5);
            mesh_e_axis[new_e] = 0;
            mesh_e_axis[new_e+1] = 1;
            mesh_e_axis[new_e+2] = 2;
            new_v += 6;
            new_e += 3;
        }

        EXPECT_TRUE(mesh.save(get_current_test_name()+".geogram"));
    }
}
