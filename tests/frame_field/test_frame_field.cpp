//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/3.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
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
         * @details The triangles of this striped triangulation have long axial diagonals, so
         *          the only edges whose dihedral angle is not zero run along the axis. That is
         *          what makes this mesh a sharp test of the direction the curvature tensor
         *          votes for: the bending of such a fold happens in the direction perpendicular
         *          to the fold line, i.e. around the tube, not along it.
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
         * @details Each quad is split around its own centroid when \p regular is true, which
         *          keeps the triangles roughly isotropic; the striped split otherwise has long
         *          diagonal edges. The curvature directions are only as good as the sampling,
         *          so the anisotropic test below uses the regular split.
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

        /**
         * @brief Reads the anisotropy matrix of one facet out of the packed output.
         * @param[in] matrices the 9 * M.facets.nb() coefficients, row-major per facet
         * @param[in] f the facet index
         * @return the matrix of facet @p f, as the row-major layout defines it.
         */
        GEO::mat3 anisotropy_matrix(const std::vector<double>& matrices, const GEO::index_t f) {
            GEO::mat3 M;
            for (GEO::index_t i = 0; i < 3; ++i) {
                for (GEO::index_t j = 0; j < 3; ++j) {
                    M(i, j) = matrices[9 * f + 3 * i + j];
                }
            }
            return M;
        }

        /**
         * @brief Evaluates a metric in one direction.
         * @details This is the squared length of @p v in the metric, so the larger it is the
         *          shorter the edges of a unit mesh are along @p v.
         * @param[in] M the metric matrix
         * @param[in] v the direction, not necessarily of unit length
         * @return @f$v^{\mathsf{T}} M v@f$.
         */
        double metric_square(const GEO::mat3& M, const GEO::vec3& v) {
            return GEO::dot(v, M * v);
        }

        /**
         * @brief Projects a direction into the plane of a facet and normalizes it.
         * @param[in] M the mesh
         * @param[in] f the facet index
         * @param[in] v the direction to project; it must not be parallel to the normal
         * @return the unit in-plane direction along @p v.
         */
        GEO::vec3 in_facet_plane(const GEO::Mesh& M, const GEO::index_t f, const GEO::vec3& v) {
            const GEO::vec3 N = GEO::normalize(GEO::Geom::mesh_facet_normal(M, f));
            return GEO::normalize(v - GEO::dot(v, N) * N);
        }

        /**
         * @brief Computes the direction around the tube of a torus at one of its points.
         * @details That is the meridian tangent @f$\partial P/\partial v@f$, which is the
         *          direction of the largest curvature wherever the tube radius is smaller
         *          than half the big radius. The point of the meridian plane it lives in is
         *          that the tangent plane of a torus meets that plane along exactly this
         *          direction, so projecting it onto a facet returns the direction itself.
         * @param[in] p a point of the surface of the torus
         * @param[in] big_radius distance from the axis to the center of the tube
         * @param[in] small_radius radius of the tube
         * @return a vector along the tube, at @p p.
         */
        GEO::vec3 torus_tube_direction(
            const GEO::vec3& p, const double big_radius, const double small_radius
            ) {
            const double rho = std::sqrt(p.x * p.x + p.y * p.y);
            const GEO::vec3 e_r(p.x / rho, p.y / rho, 0.0);
            return ((rho - big_radius) / small_radius) * GEO::vec3(0.0, 0.0, 1.0)
                - (p.z / small_radius) * e_r;
        }

        /**
         * @brief Checks that a matrix is the identity.
         * @param[in] matrices the packed anisotropy matrices
         * @param[in] f the facet index
         * @param[in] tolerance the bound on every coefficient
         */
        void expect_identity(const std::vector<double>& matrices, const GEO::index_t f,
                            const double tolerance) {
            for (GEO::index_t i = 0; i < 3; ++i) {
                for (GEO::index_t j = 0; j < 3; ++j) {
                    EXPECT_NEAR(matrices[9 * f + 3 * i + j], (i == j) ? 1.0 : 0.0, tolerance)
                        << "facet " << f << ", coefficient (" << i << ", " << j << ")";
                }
            }
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
     * @brief The anisotropy matrices are symmetric positive-definite metrics of unit
     *        determinant whose normal weight is 1.
     * @details They are built as @f$R\,\mathrm{diag}(s, 1/s, 1)\,R^{\mathsf{T}}@f$ from an
     *          orthonormal frame, so this pins down the layout, the symmetry, the fact that
     *          the anisotropy never changes the area of a facet - unit determinant, hence no
     *          density bias - and the fact that the third axis is left alone.
     */
    TEST_F(FrameFieldTest, anisotropy_matrices_are_unit_determinant_metrics) {
        FrameField field;
        field.create_curvature_directions(mesh, 45.0, 0.0);

        std::vector<double> matrices;
        field.compute_anisotropy_matrices(mesh, 3.0, matrices);
        ASSERT_EQ(matrices.size(), 9 * mesh.facets.nb());

        for (const auto& f : mesh.facets) {
            const GEO::mat3 M = anisotropy_matrix(matrices, f);
            for (GEO::index_t i = 0; i < 3; ++i) {
                for (GEO::index_t j = 0; j < 3; ++j) {
                    EXPECT_NEAR(M(i, j), M(j, i), 1e-12) << "facet " << f;
                }
            }

            EXPECT_NEAR(GEO::det(M), 1.0, 1e-12) << "facet " << f;

            // The normal is left at weight 1, and the two in-plane weights are positive and
            // multiply to 1, which is what makes the determinant 1.
            const GEO::vec3 N = GEO::normalize(GEO::Geom::mesh_facet_normal(mesh, f));
            EXPECT_NEAR(metric_square(M, N), 1.0, 1e-12) << "facet " << f;

            const GEO::vec3 U = GEO::normalize(frame_vector(field, f, 0));
            const double weight_u = metric_square(M, U);
            EXPECT_GT(weight_u, 0.0) << "facet " << f;
            EXPECT_NEAR(
                weight_u * metric_square(M, GEO::normalize(GEO::cross(N, U))), 1.0, 1e-12
                ) << "facet " << f;
        }
    }

    /**
     * @brief The large weight goes across a ridge, not along it.
     * @details A cylinder is curved around its axis and flat along it, so it is the sharpest
     *          case for the question of which of the two weights belongs to which direction:
     *          cells have to be short around the tube and long along the axis. The frame says
     *          the same thing, U carrying the larger magnitude around the tube, and the ratio
     *          of the two weights reaches @p max_aspect because the smallest curvature
     *          vanishes.
     */
    TEST_F(FrameFieldTest, metric_is_short_around_a_cylinder_and_long_along_it) {
        constexpr double max_aspect = 4.0;

        GEO::Mesh cylinder;
        make_cylinder(cylinder, 1.0, 3.0, 32, 18);

        FrameField field;
        field.create_curvature_directions(cylinder, 45.0, 0.0);

        std::vector<double> matrices;
        field.compute_anisotropy_matrices(cylinder, max_aspect, matrices);
        ASSERT_EQ(matrices.size(), 9 * cylinder.facets.nb());

        const GEO::vec3 axis(0.0, 0.0, 1.0);
        for (const auto& f : cylinder.facets) {
            const GEO::vec3 N = GEO::normalize(GEO::Geom::mesh_facet_normal(cylinder, f));
            const GEO::vec3 along = in_facet_plane(cylinder, f, axis);
            const GEO::vec3 around = GEO::normalize(GEO::cross(N, along));

            // U is the maximum curvature direction, i.e. around the tube.
            const GEO::vec3 stored_u = frame_vector(field, f, 0);
            const GEO::vec3 stored_v = frame_vector(field, f, 1);
            const GEO::vec3 U = GEO::normalize(stored_u);
            EXPECT_NEAR(::fabs(GEO::dot(U, along)), 0.0, 1e-6) << "facet " << f;
            EXPECT_NEAR(::fabs(GEO::dot(U, around)), 1.0, 1e-6) << "facet " << f;

            // ... and it carries the larger size. The other one is the direction of the
            // vanishing curvature, so its magnitude is down at the regularization of the
            // estimator rather than at a curvature of the surface: an order of magnitude
            // smaller is what the estimate leaves of it.
            EXPECT_LT(GEO::length(stored_v), 0.1 * GEO::length(stored_u)) << "facet " << f;

            const GEO::mat3 M = anisotropy_matrix(matrices, f);
            const double weight_around = metric_square(M, around);
            const double weight_along = metric_square(M, along);
            EXPECT_GT(weight_around, weight_along) << "facet " << f;
            EXPECT_GT(weight_around, 1.0) << "facet " << f;
            EXPECT_LT(weight_along, 1.0) << "facet " << f;

            // The in-plane weights multiply to 1 whatever the anisotropy, and their ratio is
            // bounded by max_aspect, which a developable facet reaches up to the accuracy of
            // the estimate.
            EXPECT_NEAR(weight_around * weight_along, 1.0, 1e-12) << "facet " << f;
            EXPECT_LE(weight_around / weight_along, max_aspect * (1.0 + 1e-9)) << "facet " << f;
            EXPECT_GT(weight_around / weight_along, 0.95 * max_aspect) << "facet " << f;
        }
    }

    /**
     * @brief Where both curvatures are non-zero, the frame and the metric follow the larger
     *        one, and the sizes follow the two curvatures.
     * @details On a torus the two principal directions are around the tube and around the big
     *          circle. With a tube radius smaller than half the big radius, the curvature
     *          around the tube is the larger one everywhere, so U must be the tube direction
     *          and carry the larger size, and the metric must be the stronger of the two
     *          there - cells short along the tube, long around the big circle.
     */
    TEST_F(FrameFieldTest, torus_prefers_the_tube_direction) {
        constexpr double big_radius = 1.0;
        constexpr double small_radius = 0.35;

        GEO::Mesh torus;
        make_torus(torus, big_radius, small_radius, 48, 24, true);

        FrameField field;
        field.create_curvature_directions(torus, 45.0, 0.0);

        std::vector<double> matrices;
        field.compute_anisotropy_matrices(torus, 2.0, matrices);
        ASSERT_EQ(matrices.size(), 9 * torus.facets.nb());

        for (const auto& f : torus.facets) {
            const GEO::vec3 N = GEO::normalize(GEO::Geom::mesh_facet_normal(torus, f));
            const GEO::vec3 tube = in_facet_plane(
                torus, f, torus_tube_direction(
                    GEO::Geom::mesh_facet_center(torus, f), big_radius, small_radius));
            const GEO::vec3 big_circle = GEO::normalize(GEO::cross(N, tube));

            const GEO::vec3 U = GEO::normalize(frame_vector(field, f, 0));
            const GEO::vec3 V = frame_vector(field, f, 1);
            EXPECT_GT(::fabs(GEO::dot(U, tube)), 0.99) << "facet " << f;
            EXPECT_GT(GEO::length(frame_vector(field, f, 0)), GEO::length(V)) << "facet " << f;

            const GEO::mat3 M = anisotropy_matrix(matrices, f);
            EXPECT_GT(metric_square(M, tube), metric_square(M, big_circle)) << "facet " << f;
            EXPECT_GT(metric_square(M, tube) / metric_square(M, big_circle), 1.2)
                << "facet " << f;
        }
    }

    /**
     * @brief A flat mesh carries no anisotropy and gets the identity.
     * @details Both eigenvalues of the estimator come back as the same regularization of a
     *          zero tensor, so the relative anisotropy is 0 and the metric is isotropic.
     */
    TEST_F(FrameFieldTest, flat_mesh_gets_the_identity_metric) {
        GEO::Mesh plane;
        make_plane(plane, 4);

        FrameField field;
        field.create_curvature_directions(plane, 45.0, 0.0);

        std::vector<double> matrices;
        field.compute_anisotropy_matrices(plane, 4.0, matrices);
        ASSERT_EQ(matrices.size(), 9 * plane.facets.nb());
        for (const auto& f : plane.facets) {
            expect_identity(matrices, f, 1e-12);
        }
    }

    /**
     * @brief max_aspect caps the ratio of the two weights, and a value below 1 is the
     *        identity.
     * @details The cap is on the ratio of the weights, i.e. on the square of the aspect ratio
     *          of the cells, and it is reached only where the smallest curvature vanishes.
     */
    TEST_F(FrameFieldTest, max_aspect_caps_the_ratio_of_the_weights) {
        GEO::Mesh cylinder;
        make_cylinder(cylinder, 1.0, 3.0, 32, 18);

        FrameField field;
        field.create_curvature_directions(cylinder, 45.0, 0.0);

        const GEO::vec3 axis(0.0, 0.0, 1.0);
        double previous = 0.0;
        for (const double max_aspect : {1.5, 2.0, 4.0, 8.0}) {
            std::vector<double> matrices;
            field.compute_anisotropy_matrices(cylinder, max_aspect, matrices);
            ASSERT_EQ(matrices.size(), 9 * cylinder.facets.nb());

            double smallest = std::numeric_limits<double>::max();
            for (const auto& f : cylinder.facets) {
                const GEO::mat3 M = anisotropy_matrix(matrices, f);
                const GEO::vec3 N = GEO::normalize(GEO::Geom::mesh_facet_normal(cylinder, f));
                const GEO::vec3 along = in_facet_plane(cylinder, f, axis);
                const double ratio = metric_square(M, GEO::normalize(GEO::cross(N, along)))
                    / metric_square(M, along);
                EXPECT_LE(ratio, max_aspect * (1.0 + 1e-9)) << "facet " << f;
                smallest = std::min(smallest, ratio);
            }
            EXPECT_GT(smallest, previous) << "max_aspect = " << max_aspect;
            previous = smallest;
        }

        // 1 is the isotropic case, and anything below it is treated as 1.
        std::vector<double> unit;
        field.compute_anisotropy_matrices(cylinder, 1.0, unit);
        std::vector<double> below;
        field.compute_anisotropy_matrices(cylinder, 0.25, below);
        ASSERT_EQ(unit.size(), below.size());
        for (const auto& f : cylinder.facets) {
            expect_identity(unit, f, 1e-12);
        }
        for (std::size_t c = 0; c < unit.size(); ++c) {
            EXPECT_NEAR(unit[c], below[c], 1e-12);
        }
    }

    /**
     * @brief The matrices depend on the ratio of the two magnitudes, not on their scale, so
     *        the autoscale of the field does not reach them.
     */
    TEST_F(FrameFieldTest, anisotropy_matrices_do_not_depend_on_the_autoscale) {
        FrameField raw;
        raw.create_curvature_directions(mesh, 45.0, 0.0);
        FrameField scaled;
        scaled.create_curvature_directions(mesh, 45.0, 2.0);

        std::vector<double> from_raw;
        std::vector<double> from_scaled;
        raw.compute_anisotropy_matrices(mesh, 3.0, from_raw);
        scaled.compute_anisotropy_matrices(mesh, 3.0, from_scaled);

        ASSERT_EQ(from_raw.size(), from_scaled.size());
        for (std::size_t c = 0; c < from_raw.size(); ++c) {
            EXPECT_NEAR(from_raw[c], from_scaled[c], 1e-12) << "coefficient " << c;
        }
    }

    /**
     * @brief A field that carries no curvature certainty gets its anisotropy from the lengths
     *        of its own frame vectors.
     * @details The inherited cross field publishes unit vectors, so it says nothing about any
     *          anisotropy and the matrices are the identity.
     */
    TEST_F(FrameFieldTest, field_without_certainty_gets_the_identity_metric) {
        FrameField field;
        field.create_from_surface_mesh(mesh, false, 45.0);
        ASSERT_TRUE(all_frames_are_unit(field, mesh));

        std::vector<double> matrices;
        field.compute_anisotropy_matrices(mesh, 4.0, matrices);
        ASSERT_EQ(matrices.size(), 9 * mesh.facets.nb());
        for (const auto& f : mesh.facets) {
            expect_identity(matrices, f, 1e-12);
        }
    }

    /**
     * @brief The matrices are only defined by a field with exactly one frame per facet.
     * @details The output is left untouched when the check fails, so a caller can reuse its
     *          buffer for another attempt.
     */
    TEST_F(FrameFieldTest, anisotropy_matrices_require_a_field_for_the_same_mesh) {
        GEO::Mesh plane;
        make_plane(plane, 2);

        FrameField field;
        field.create_curvature_directions(plane, 45.0, 0.0);

        std::vector<double> matrices{1.0, 2.0, 3.0};
        EXPECT_THROW(field.compute_anisotropy_matrices(mesh, 3.0, matrices), std::logic_error);

        // A field that was never built has no frame either.
        EXPECT_THROW(
            FrameField().compute_anisotropy_matrices(plane, 3.0, matrices), std::logic_error
            );
        EXPECT_EQ(matrices.size(), 3);
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
