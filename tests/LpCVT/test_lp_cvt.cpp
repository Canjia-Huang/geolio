//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/3.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <gtest/gtest.h>

#include <geogram/basic/process.h>
#include <geogram/mesh/mesh.h>
#include <geogram/voronoi/CVT.h>

#include <geolio/LpCVT/lp_cvt.h>
#include <geolio/LpCVT/lp_integration_simplex.h>
#include <geolio/LpCVT/lp_measure.h>
#include <geolio/LpCVT/lp_polynomial.h>

#include <cmath>
#include <functional>
#include <random>
#include <vector>

namespace geolio::test
{
    /**
     * @brief Exposes the protected objective/gradient evaluation for testing.
     * @details ``GEO::CentroidalVoronoiTesselation::funcgrad()`` is protected by
     *          design, since client code is meant to drive it through
     *          ``Newton_iterations()``. The tests need to call it directly to check
     *          the analytic gradient against finite differences, so it is promoted
     *          here.
     */
    class TestableLpCVT : public LpCVT {
    public:
        using LpCVT::LpCVT;

        /** @brief Promotes the protected objective evaluation to public. */
        using LpCVT::funcgrad;
    };

    /**
     * @brief Test fixture owning the background meshes shared by the LpCVT tests.
     * @details Provides a closed, triangulated unit cube (whose twelve triangles
     *          include coplanar neighbours, which is precisely the configuration
     *          that makes the three-plane system of configuration (B) singular) and
     *          a matching six-tetrahedron decomposition of the same cube.
     *          Multi-threading is disabled in SetUp() so that results do not depend
     *          on the RVD's mesh partitioning and on floating-point summation order.
     */
    class LpCVTTest : public ::testing::Test {
    protected:
        void SetUp() override {
            max_threads_backup_ = GEO::Process::maximum_concurrent_threads();
            GEO::Process::set_max_threads(1);

            create_cube();
            create_cube_tets();
        }

        void TearDown() override {
            GEO::Process::set_max_threads(max_threads_backup_);
        }

        /**
         * @brief Builds the unit cube as twelve consistently oriented triangles.
         */
        void create_cube() {            cube_.vertices.create_vertices(8);
            cube_.vertices.point(0) = GEO::vec3(0, 0, 0);
            cube_.vertices.point(1) = GEO::vec3(1, 0, 0);
            cube_.vertices.point(2) = GEO::vec3(1, 1, 0);
            cube_.vertices.point(3) = GEO::vec3(0, 1, 0);
            cube_.vertices.point(4) = GEO::vec3(0, 0, 1);
            cube_.vertices.point(5) = GEO::vec3(1, 0, 1);
            cube_.vertices.point(6) = GEO::vec3(1, 1, 1);
            cube_.vertices.point(7) = GEO::vec3(0, 1, 1);

            static const GEO::index_t tris[12][3] = {
                {0, 2, 1}, {0, 3, 2}, // z = 0
                {4, 5, 6}, {4, 6, 7}, // z = 1
                {0, 1, 5}, {0, 5, 4}, // y = 0
                {3, 7, 6}, {3, 6, 2}, // y = 1
                {0, 4, 7}, {0, 7, 3}, // x = 0
                {1, 2, 6}, {1, 6, 5} // x = 1
            };

            cube_.facets.create_triangles(12);
            for (GEO::index_t f = 0; f < 12; ++f) {
                for (GEO::index_t lv = 0; lv < 3; ++lv) {
                    cube_.facets.set_vertex(f, lv, tris[f][lv]);
                }
            }
            cube_.facets.connect();
        }

        /**
         * @brief Fills the same cube with the six tetrahedra of the Freudenthal
         *        decomposition along the main diagonal, so that both meshing modes
         *        can be tested on the same geometry.
         */
        void create_cube_tets() {
            static const GEO::index_t tets[6][4] = {
                {0, 1, 2, 6}, {0, 2, 3, 6}, {0, 3, 7, 6},
                {0, 7, 4, 6}, {0, 4, 5, 6}, {0, 5, 1, 6}
            };

            cube_.cells.create_tets(6);
            for (GEO::index_t t = 0; t < 6; ++t) {
                for (GEO::index_t lv = 0; lv < 4; ++lv) {
                    cube_.cells.set_vertex(t, lv, tets[t][lv]);
                }
            }
            cube_.cells.connect();
        }

        /**
         * @brief Evaluates the LpCVT objective and gradient at a point set.
         * @details Only one ``GEO::CentroidalVoronoiTesselation`` may exist at a
         *          time, so the object is created and destroyed within the call.
         * @param[in] mesh The background mesh.
         * @param[in] p The norm exponent.
         * @param[in] volumetric Whether to mesh the volume.
         * @param[in] pts The point coordinates, three doubles per point.
         * @param[out] f The objective value.
         * @param[out] g The gradient, resized to ``pts.size()``.
         */
        void evaluate(
            GEO::Mesh& mesh,
            const unsigned int p,
            const bool volumetric,
            const std::vector<double>& pts,
            double& f,
            std::vector<double>& g
            ) const {
            TestableLpCVT cvt(&mesh, p, volumetric);
            cvt.set_points(pts.size() / 3, pts.data());
            g.assign(pts.size(), 0.0);
            cvt.funcgrad(GEO::index_t(pts.size()), const_cast<double*>(pts.data()), f, g.data());
        }

        /**
         * @brief Samples points on the surface of the cube.
         * @param[in] nb_points Number of points to generate.
         * @param[in] seed Random seed, fixed for reproducibility.
         * @return The point coordinates, three doubles per point.
         */
        static std::vector<double> sample_cube_surface(
            const GEO::index_t nb_points,
            const unsigned int seed = 20100401
            ) {
            static const double face_normals[6][3] = {
                {0, 0, -1}, {0, 0, 1}, {0, -1, 0}, {0, 1, 0}, {-1, 0, 0}, {1, 0, 0}
            };
            std::mt19937 gen(seed);
            std::uniform_int_distribution<int> face_dist(0, 5);
            std::uniform_real_distribution<double> coord(0.0, 1.0);

            std::vector<double> pts;
            pts.reserve(nb_points * 3);
            for (GEO::index_t i = 0; i < nb_points; ++i) {
                const int face = face_dist(gen);
                double p[3];
                for (int c = 0; c < 3; ++c) {
                    p[c] = coord(gen);
                }
                // Project onto the selected face.
                for (int c = 0; c < 3; ++c) {
                    if (face_normals[face][c] > 0.0) {
                        p[c] = 1.0;
                    } else if (face_normals[face][c] < 0.0) {
                        p[c] = 0.0;
                    }
                }
                pts.push_back(p[0]);
                pts.push_back(p[1]);
                pts.push_back(p[2]);
            }
            return pts;
        }

        /**
         * @brief The background mesh: a triangulated cube that is also filled with
         *        the six tetrahedra of the Freudenthal decomposition, so that both
         *        meshing modes can be exercised on the same geometry.
         */
        GEO::Mesh cube_;
        GEO::index_t max_threads_backup_ = 1;
    };

    // =====================================================================
    // Reference quadrature.
    //
    // The tests below need an independent, high-accuracy value for
    //     integral over a simplex of ||x - p0||_p^p
    // which is exactly the physical energy that the ported objective reproduces up
    // to the constant factor documented in lp_measure.h. Collapsed
    // (Duffy-transformed) Gauss-Legendre rules are used, because they are exact for
    // the polynomial integrands involved at any degree.
    // =====================================================================

    /**
     * @brief Computes Gauss-Legendre nodes and weights on [0, 1].
     * @details Nodes are the roots of the Legendre polynomial of degree @p order,
     *          found by Newton iteration, with the standard weight formula.
     * @param[in] order Number of nodes.
     * @param[out] nodes The nodes, in [0, 1].
     * @param[out] weights The matching weights.
     */
    void gauss_legendre(
        const int order,
        std::vector<double>& nodes,
        std::vector<double>& weights
        ) {
        constexpr double pi = 3.14159265358979323846;
        nodes.resize(order);
        weights.resize(order);

        // Evaluates the Legendre polynomial of degree `n` and of degree `n - 1`
        // at `x` by the standard three-term recurrence.
        const auto legendre = [](const int n, const double x, double& pn, double& pn_1) {
            double p0 = 1.0;
            double p1 = x;
            for (int k = 2; k <= n; ++k) {
                const double p2 = ((2.0 * k - 1.0) * x * p1 - (k - 1.0) * p0) / double(k);
                p0 = p1;
                p1 = p2;
            }
            pn = (n == 1) ? x : p1;
            pn_1 = (n == 1) ? 1.0 : p0;
        };

        for (int i = 0; i < order; ++i) {
            // Initial guess: the Chebyshev-like distribution of the roots.
            double x = std::cos(pi * (double(i) + 0.75) / (double(order) + 0.5));
            for (int iter = 0; iter < 100; ++iter) {
                double pn = 1.0, pn_1 = 1.0;
                legendre(order, x, pn, pn_1);
                // P'_n(x) = n (x P_n(x) - P_{n-1}(x)) / (x^2 - 1)
                const double dp = double(order) * (x * pn - pn_1) / (x * x - 1.0);
                const double dx = pn / dp;
                x -= dx;
                if (std::fabs(dx) < 1e-15) {
                    break;
                }
            }

            double pn = 1.0, pn_1 = 1.0;
            legendre(order, x, pn, pn_1);
            const double dp = double(order) * (x * pn - pn_1) / (x * x - 1.0);

            // Mapped to [0, 1]: node = (x + 1) / 2 and weight = w_{-1,1} / 2,
            // with the classical w = 2 / ((1 - x^2) P'_n(x)^2).
            nodes[i] = 0.5 * (x + 1.0);
            weights[i] = 1.0 / ((1.0 - x * x) * dp * dp);
        }
    }

    /**
     * @brief Orders needed by the collapsed rules for a degree @p p integrand.
     * @details After the change of variables the integrand has degree ``p + 2`` in
     *          the slowest variable and ``p`` in the fastest, and an n-point
     *          Gauss-Legendre rule is exact up to degree ``2n - 1``.
     * @param[in] p Degree of the integrand.
     * @return A number of nodes sufficient for exactness.
     */
    int quadrature_order(const unsigned int p) {
        return int((p + 3) / 2) + 2;
    }

    /**
     * @brief Integrates a function over a triangle.
     * @details Uses the collapse ``x = a + s (b - a) + t (c - a)`` with
     *          ``s = u``, ``t = v (1 - u)`` and Jacobian ``(1 - u)``.
     * @param[in] a First triangle vertex.
     * @param[in] b Second triangle vertex.
     * @param[in] c Third triangle vertex.
     * @param[in] integrand The function to integrate.
     * @param[in] order Number of Gauss-Legendre nodes per variable.
     * @return The integral of @p integrand over the triangle, in physical units.
     */
    double integrate_triangle(
        const GEO::vec3& a,
        const GEO::vec3& b,
        const GEO::vec3& c,
        const std::function<double(const GEO::vec3&)>& integrand,
        const int order
        ) {
        std::vector<double> nodes, weights;
        gauss_legendre(order, nodes, weights);

        const double area_scale = GEO::length(GEO::cross(b - a, c - a));
        double total = 0.0;
        for (int i = 0; i < order; ++i) {
            for (int j = 0; j < order; ++j) {
                const double u = nodes[i];
                const double v = nodes[j];
                const double s = u;
                const double t = v * (1.0 - u);
                const GEO::vec3 x = a + s * (b - a) + t * (c - a);
                total += weights[i] * weights[j] * (1.0 - u) * integrand(x);
            }
        }
        return total * area_scale;
    }

    /**
     * @brief Integrates a function over a tetrahedron.
     * @details Uses the collapse ``x = p0 + s U1 + t U2 + u U3`` with
     *          ``s = a``, ``t = b (1 - a)``, ``u = c (1 - a) (1 - b)`` and Jacobian
     *          ``(1 - a)^2 (1 - b)``.
     * @param[in] p0 First tetrahedron vertex.
     * @param[in] p1 Second tetrahedron vertex.
     * @param[in] p2 Third tetrahedron vertex.
     * @param[in] p3 Fourth tetrahedron vertex.
     * @param[in] integrand The function to integrate.
     * @param[in] order Number of Gauss-Legendre nodes per variable.
     * @return The integral of @p integrand over the tetrahedron, in physical units.
     */
    double integrate_tetrahedron(
        const GEO::vec3& p0,
        const GEO::vec3& p1,
        const GEO::vec3& p2,
        const GEO::vec3& p3,
        const std::function<double(const GEO::vec3&)>& integrand,
        const int order
        ) {
        std::vector<double> nodes, weights;
        gauss_legendre(order, nodes, weights);

        const GEO::vec3 U1 = p1 - p0;
        const GEO::vec3 U2 = p2 - p0;
        const GEO::vec3 U3 = p3 - p0;
        const double volume_scale = ::fabs(GEO::dot(U1, GEO::cross(U2, U3)));

        double total = 0.0;
        for (int i = 0; i < order; ++i) {
            for (int j = 0; j < order; ++j) {
                for (int k = 0; k < order; ++k) {
                    const double a = nodes[i];
                    const double b = nodes[j];
                    const double c = nodes[k];
                    const double s = a;
                    const double t = b * (1.0 - a);
                    const double u = c * (1.0 - a) * (1.0 - b);
                    const GEO::vec3 x = p0 + s * U1 + t * U2 + u * U3;
                    total += weights[i] * weights[j] * weights[k] *
                        (1.0 - a) * (1.0 - a) * (1.0 - b) * integrand(x);
                }
            }
        }
        return total * volume_scale;
    }

    /**
     * @brief Builds a deterministic random vector.
     * @param[in,out] gen The generator to draw from.
     * @return A vector with coordinates uniform in [-0.5, 0.5].
     */
    GEO::vec3 random_vec3(std::mt19937& gen) {
        std::uniform_real_distribution<double> d(-0.5, 0.5);
        return GEO::vec3(d(gen), d(gen), d(gen));
    }

    // =====================================================================
    // T1: the integrand value against an independent quadrature.
    // =====================================================================

    TEST_F(LpCVTTest, surface_integrand_matches_quadrature) {
        std::mt19937 gen(1234);
        for (const unsigned int p : {2u, 4u, 6u, 8u}) {
            const GEO::vec3 seed = random_vec3(gen);
            const GEO::vec3 a = random_vec3(gen) + GEO::vec3(1, 0, 0);
            const GEO::vec3 b = random_vec3(gen) + GEO::vec3(0, 1, 0);
            const GEO::vec3 c = random_vec3(gen) + GEO::vec3(0, 0, 1);

            GEO::vec3 dFdp0, dFdp1, dFdp2, dFdp3;
            double f = 0.0;

            // Dispatch on p the same way the integrand factory does.
            switch (p) {
            case 2:
                f = LpPolynomial<2>().eval(
                    LpTriArea(), seed, a, b, c, GEO::mat3(),
                    dFdp0, dFdp1, dFdp2, dFdp3);
                break;
            case 4:
                f = LpPolynomial<4>().eval(
                    LpTriArea(), seed, a, b, c, GEO::mat3(),
                    dFdp0, dFdp1, dFdp2, dFdp3);
                break;
            case 6:
                f = LpPolynomial<6>().eval(
                    LpTriArea(), seed, a, b, c, GEO::mat3(),
                    dFdp0, dFdp1, dFdp2, dFdp3);
                break;
            default:
                f = LpPolynomial<8>().eval(
                    LpTriArea(), seed, a, b, c, GEO::mat3(),
                    dFdp0, dFdp1, dFdp2, dFdp3);
                break;
            }

            const double reference = integrate_triangle(
                a, b, c,
                [&](const GEO::vec3& x) {
                    double acc = 0.0;
                    for (int comp = 0; comp < 3; ++comp) {
                        acc += std::pow(std::fabs(x[comp] - seed[comp]), double(p));
                    }
                    return acc;
                },
                quadrature_order(p)
                );

            const double expected = lp_surface_energy_normalization(p) * reference;
            EXPECT_NEAR(f, expected, 1e-10 * std::fabs(expected) + 1e-12)
                << "p = " << p;
        }
    }

    TEST_F(LpCVTTest, volume_integrand_matches_quadrature) {
        std::mt19937 gen(4321);
        for (const unsigned int p : {2u, 4u, 6u, 8u}) {
            // Build a tetrahedron with a positive orientation so that the signed
            // measure of LpTetVolume is the physical volume.
            GEO::vec3 p0 = random_vec3(gen);
            const GEO::vec3 U1(1.0, 0.0, 0.0);
            const GEO::vec3 U2(0.3, 1.1, 0.0);
            const GEO::vec3 U3(0.2, 0.4, 0.9);
            const GEO::vec3 p1 = p0 + U1;
            const GEO::vec3 p2 = p0 + U2;
            const GEO::vec3 p3 = p0 + U3;

            GEO::vec3 dFdp0, dFdp1, dFdp2, dFdp3;
            double f = 0.0;

            switch (p) {
            case 2:
                f = LpPolynomial<2>().eval(
                    LpTetVolume(), p0, p1, p2, p3, GEO::mat3(),
                    dFdp0, dFdp1, dFdp2, dFdp3);
                break;
            case 4:
                f = LpPolynomial<4>().eval(
                    LpTetVolume(), p0, p1, p2, p3, GEO::mat3(),
                    dFdp0, dFdp1, dFdp2, dFdp3);
                break;
            case 6:
                f = LpPolynomial<6>().eval(
                    LpTetVolume(), p0, p1, p2, p3, GEO::mat3(),
                    dFdp0, dFdp1, dFdp2, dFdp3);
                break;
            default:
                f = LpPolynomial<8>().eval(
                    LpTetVolume(), p0, p1, p2, p3, GEO::mat3(),
                    dFdp0, dFdp1, dFdp2, dFdp3);
                break;
            }

            const double reference = integrate_tetrahedron(
                p0, p1, p2, p3,
                [&](const GEO::vec3& x) {
                    double acc = 0.0;
                    for (int comp = 0; comp < 3; ++comp) {
                        acc += std::pow(std::fabs(x[comp] - p0[comp]), double(p));
                    }
                    return acc;
                },
                quadrature_order(p)
                );

            const double expected = lp_volume_energy_normalization(p) * reference;
            EXPECT_NEAR(f, expected, 1e-9 * std::fabs(expected) + 1e-12)
                << "p = " << p;
        }
    }

    // =====================================================================
    // T2: the simplex gradients against central differences.
    // =====================================================================

    TEST_F(LpCVTTest, surface_integrand_gradient_matches_finite_differences) {
        std::mt19937 gen(555);
        for (const unsigned int p : {2u, 4u, 6u}) {
            GEO::vec3 q[4] = {
                random_vec3(gen), random_vec3(gen) + GEO::vec3(1, 0, 0),
                random_vec3(gen) + GEO::vec3(0, 1, 0), random_vec3(gen) + GEO::vec3(0, 0, 1)
            };
            // A non-trivial anisotropy matrix, to exercise both the M and the M^T
            // occurrences of the frame.
            GEO::mat3 M;
            M(0, 0) = 1.3;
            M(0, 1) = 0.2;
            M(1, 1) = 0.8;
            M(2, 2) = 1.1;
            M(2, 0) = -0.15;

            auto value_at = [&](const GEO::vec3* v) {
                GEO::vec3 g0, g1, g2, g3;
                switch (p) {
                case 2:
                    return LpPolynomial<2>().eval(LpTriArea(), v[0], v[1], v[2], v[3], M, g0, g1, g2, g3);
                case 4:
                    return LpPolynomial<4>().eval(LpTriArea(), v[0], v[1], v[2], v[3], M, g0, g1, g2, g3);
                default:
                    return LpPolynomial<6>().eval(LpTriArea(), v[0], v[1], v[2], v[3], M, g0, g1, g2, g3);
                }
            };

            GEO::vec3 analytic[4];
            switch (p) {
            case 2:
                LpPolynomial<2>().eval(LpTriArea(), q[0], q[1], q[2], q[3], M,
                                       analytic[0], analytic[1], analytic[2], analytic[3]);
                break;
            case 4:
                LpPolynomial<4>().eval(LpTriArea(), q[0], q[1], q[2], q[3], M,
                                       analytic[0], analytic[1], analytic[2], analytic[3]);
                break;
            default:
                LpPolynomial<6>().eval(LpTriArea(), q[0], q[1], q[2], q[3], M,
                                       analytic[0], analytic[1], analytic[2], analytic[3]);
                break;
            }

            constexpr double eps = 1e-6;
            std::uniform_int_distribution<int> vertex_dist(0, 3);
            for (int trial = 0; trial < 12; ++trial) {
                const int v = vertex_dist(gen);
                GEO::vec3 dir = random_vec3(gen);
                dir = GEO::normalize(dir);

                GEO::vec3 plus[4] = {q[0], q[1], q[2], q[3]};
                GEO::vec3 minus[4] = {q[0], q[1], q[2], q[3]};
                plus[v] = plus[v] + eps * dir;
                minus[v] = minus[v] - eps * dir;

                const double numeric = (value_at(plus) - value_at(minus)) / (2.0 * eps);
                const double exact = GEO::dot(analytic[v], dir);
                EXPECT_NEAR(numeric, exact, 2e-5 * std::fabs(exact) + 1e-9)
                    << "p = " << p << " vertex " << v;
            }
        }
    }

    TEST_F(LpCVTTest, volume_integrand_gradient_matches_finite_differences) {
        std::mt19937 gen(777);
        for (const unsigned int p : {2u, 4u, 6u}) {
            GEO::vec3 q[4] = {
                random_vec3(gen), random_vec3(gen), random_vec3(gen), random_vec3(gen)
            };
            GEO::mat3 M;
            M(0, 0) = 0.9;
            M(1, 1) = 1.2;
            M(2, 2) = 1.0;
            M(1, 2) = 0.25;

            auto value_at = [&](const GEO::vec3* v) {
                GEO::vec3 g0, g1, g2, g3;
                switch (p) {
                case 2:
                    return LpPolynomial<2>().eval(LpTetVolume(), v[0], v[1], v[2], v[3], M, g0, g1, g2, g3);
                case 4:
                    return LpPolynomial<4>().eval(LpTetVolume(), v[0], v[1], v[2], v[3], M, g0, g1, g2, g3);
                default:
                    return LpPolynomial<6>().eval(LpTetVolume(), v[0], v[1], v[2], v[3], M, g0, g1, g2, g3);
                }
            };

            GEO::vec3 analytic[4];
            switch (p) {
            case 2:
                LpPolynomial<2>().eval(LpTetVolume(), q[0], q[1], q[2], q[3], M,
                                       analytic[0], analytic[1], analytic[2], analytic[3]);
                break;
            case 4:
                LpPolynomial<4>().eval(LpTetVolume(), q[0], q[1], q[2], q[3], M,
                                       analytic[0], analytic[1], analytic[2], analytic[3]);
                break;
            default:
                LpPolynomial<6>().eval(LpTetVolume(), q[0], q[1], q[2], q[3], M,
                                       analytic[0], analytic[1], analytic[2], analytic[3]);
                break;
            }

            constexpr double eps = 1e-6;
            std::uniform_int_distribution<int> vertex_dist(0, 3);
            for (int trial = 0; trial < 12; ++trial) {
                const int v = vertex_dist(gen);
                const GEO::vec3 dir = GEO::normalize(random_vec3(gen));

                GEO::vec3 plus[4] = {q[0], q[1], q[2], q[3]};
                GEO::vec3 minus[4] = {q[0], q[1], q[2], q[3]};
                plus[v] = plus[v] + eps * dir;
                minus[v] = minus[v] - eps * dir;

                const double numeric = (value_at(plus) - value_at(minus)) / (2.0 * eps);
                const double exact = GEO::dot(analytic[v], dir);
                EXPECT_NEAR(numeric, exact, 2e-5 * std::fabs(exact) + 1e-9)
                    << "p = " << p << " vertex " << v;
            }
        }
    }

    // =====================================================================
    // T3: the full objective gradient, i.e. the chain rule through the four
    //     configurations of LpVoronoiVertex, against central differences.
    // =====================================================================

    TEST_F(LpCVTTest, global_gradient_matches_finite_differences) {
        for (const unsigned int p : {2u, 4u, 6u}) {
            const std::vector<double> pts = sample_cube_surface(24, 31415 + p);

            double f = 0.0;
            std::vector<double> g;
            evaluate(cube_, p, false, pts, f, g);

            ASSERT_EQ(g.size(), pts.size());
            for (const double gi : g) {
                ASSERT_TRUE(std::isfinite(gi)) << "p = " << p;
            }

            std::mt19937 gen(2718 + p);
            constexpr double eps = 1e-6;
            std::uniform_int_distribution<std::size_t> index_dist(0, pts.size() - 1);

            for (int trial = 0; trial < 8; ++trial) {
                std::vector<double> dir(pts.size(), 0.0);
                for (int k = 0; k < 3; ++k) {
                    dir[index_dist(gen)] = 1.0;
                }
                // Normalize the direction so that the step stays meaningful.
                double norm = 0.0;
                for (const double d : dir) {
                    norm += d * d;
                }
                norm = std::sqrt(norm);
                for (double& d : dir) {
                    d /= norm;
                }

                std::vector<double> plus(pts.size()), minus(pts.size());
                for (std::size_t i = 0; i < pts.size(); ++i) {
                    plus[i] = pts[i] + eps * dir[i];
                    minus[i] = pts[i] - eps * dir[i];
                }

                double f_plus = 0.0, f_minus = 0.0;
                std::vector<double> g_dummy;
                evaluate(cube_, p, false, plus, f_plus, g_dummy);
                evaluate(cube_, p, false, minus, f_minus, g_dummy);

                const double numeric = (f_plus - f_minus) / (2.0 * eps);
                double exact = 0.0;
                for (std::size_t i = 0; i < pts.size(); ++i) {
                    exact += g[i] * dir[i];
                }

                EXPECT_NEAR(numeric, exact, 1e-4 * std::fabs(exact) + 1e-9)
                    << "p = " << p << " trial " << trial;
            }
        }
    }

    // =====================================================================
    // T4: golden comparison against Geogram's built-in L2 CVT objective.
    //
    // The ported objective omits the same constants as the reference
    // implementation, and for p == 2 it must therefore equal the built-in L2
    // objective scaled by the constant documented in lp_measure.h: 12 for the
    // surface case and 60 for the volume case. This checks the RVD plumbing, the
    // simplex decomposition and the whole gradient chain rule in one go.
    // =====================================================================

    TEST_F(LpCVTTest, p2_surface_matches_geogram_builtin_l2) {
        const std::vector<double> pts = sample_cube_surface(24, 99);

        double f_ref = 0.0;
        std::vector<double> g_ref;
        {
            GEO::CentroidalVoronoiTesselation reference(&cube_, 3, "BDEL");
            reference.set_points(pts.size() / 3, pts.data());
            reference.delaunay()->set_vertices(pts.size() / 3, pts.data());
            g_ref.assign(pts.size(), 0.0);
            reference.RVD()->compute_CVT_func_grad(f_ref, g_ref.data());
        }

        double f_ours = 0.0;
        std::vector<double> g_ours;
        evaluate(cube_, 2, false, pts, f_ours, g_ours);

        const double factor = lp_surface_energy_normalization(2);
        EXPECT_NEAR(factor, 12.0, 1e-12);

        EXPECT_NEAR(f_ours, factor * f_ref, 1e-9 * std::fabs(factor * f_ref) + 1e-12);

        double norm_ref = 0.0, norm_ours = 0.0;
        for (std::size_t i = 0; i < g_ref.size(); ++i) {
            norm_ref += g_ref[i] * g_ref[i];
            norm_ours += g_ours[i] * g_ours[i];
        }
        ASSERT_GT(norm_ref, 1e-12);
        EXPECT_NEAR(std::sqrt(norm_ours), factor * std::sqrt(norm_ref),
                    1e-7 * factor * std::sqrt(norm_ref));
    }

    TEST_F(LpCVTTest, p2_volume_matches_geogram_builtin_l2) {
        std::mt19937 gen(2024);
        std::uniform_real_distribution<double> coord(0.1, 0.9);
        const GEO::index_t nb_points = 16;
        std::vector<double> pts(nb_points * 3);
        for (double& c : pts) {
            c = coord(gen);
        }

        double f_ref = 0.0;
        std::vector<double> g_ref;
        {
            GEO::CentroidalVoronoiTesselation reference(&cube_, 3, "BDEL");
            reference.set_volumetric(true);
            reference.set_points(nb_points, pts.data());
            reference.delaunay()->set_vertices(nb_points, pts.data());
            g_ref.assign(pts.size(), 0.0);
            reference.RVD()->compute_CVT_func_grad(f_ref, g_ref.data());
        }

        double f_ours = 0.0;
        std::vector<double> g_ours;
        evaluate(cube_, 2, true, pts, f_ours, g_ours);

        const double factor = lp_volume_energy_normalization(2);
        EXPECT_NEAR(factor, 60.0, 1e-12);

        EXPECT_NEAR(f_ours, factor * f_ref, 1e-9 * std::fabs(factor * f_ref) + 1e-12);

        double norm_ref = 0.0, norm_ours = 0.0;
        for (std::size_t i = 0; i < g_ref.size(); ++i) {
            norm_ref += g_ref[i] * g_ref[i];
            norm_ours += g_ours[i] * g_ours[i];
        }
        ASSERT_GT(norm_ref, 1e-12);
        EXPECT_NEAR(std::sqrt(norm_ours), factor * std::sqrt(norm_ref),
                    1e-7 * factor * std::sqrt(norm_ref));
    }

    // =====================================================================
    // T5/T6: robustness on coplanar facets.
    //
    // The twelve triangles of a triangulated cube include coplanar neighbours, so
    // configuration (B) of LpVoronoiVertex is exercised with two parallel facet
    // planes. Without the orthogonal-plane substitution this makes the three-plane
    // system singular and the gradient blows up.
    // =====================================================================

    TEST_F(LpCVTTest, coplanar_facets_do_not_blow_up) {
        for (const unsigned int p : {2u, 4u, 6u}) {
            const std::vector<double> pts = sample_cube_surface(30, 6060 + p);

            double f = 0.0;
            std::vector<double> g;
            evaluate(cube_, p, false, pts, f, g);

            EXPECT_TRUE(std::isfinite(f)) << "p = " << p;
            for (const double gi : g) {
                ASSERT_TRUE(std::isfinite(gi)) << "p = " << p;
            }
        }
    }

    // =====================================================================
    // T7: end to end, the objective must decrease.
    // =====================================================================

    TEST_F(LpCVTTest, newton_iterations_decrease_the_objective) {
        for (const unsigned int p : {2u, 4u}) {
            const std::vector<double> initial = sample_cube_surface(30, 4242 + p);

            double f_before = 0.0;
            std::vector<double> g;
            evaluate(cube_, p, false, initial, f_before, g);

            double f_after = 0.0;
            std::vector<double> final_pts;
            {
                LpCVT cvt(&cube_, p, false);
                cvt.set_points(initial.size() / 3, initial.data());
                cvt.Newton_iterations(20, 7);

                final_pts.resize(initial.size());
                for (GEO::index_t i = 0; i < cvt.nb_points(); ++i) {
                    const double* p_i = cvt.embedding(i);
                    final_pts[3 * i] = p_i[0];
                    final_pts[3 * i + 1] = p_i[1];
                    final_pts[3 * i + 2] = p_i[2];
                    ASSERT_TRUE(std::isfinite(p_i[0]));
                    ASSERT_TRUE(std::isfinite(p_i[1]));
                    ASSERT_TRUE(std::isfinite(p_i[2]));
                }
            }

            evaluate(cube_, p, false, final_pts, f_after, g);
            EXPECT_LT(f_after, f_before) << "p = " << p;
        }
    }

    // =====================================================================
    // T8: unsupported parameters must be rejected, not silently accepted.
    // =====================================================================

    TEST_F(LpCVTTest, unsupported_norm_exponents_are_rejected) {
        EXPECT_TRUE(create_lp_integration_simplex(cube_, 0, false).is_null());
        EXPECT_TRUE(create_lp_integration_simplex(cube_, 1, false).is_null());
        EXPECT_TRUE(create_lp_integration_simplex(cube_, 18, false).is_null());
        EXPECT_FALSE(create_lp_integration_simplex(cube_, 2, false).is_null());
        EXPECT_FALSE(create_lp_integration_simplex(cube_, 16, false).is_null());
        EXPECT_FALSE(create_lp_integration_simplex(cube_, 4, true).is_null());
    }

    // =====================================================================
    // T9: the multi-threaded RVD traversal shares a single integrand instance
    //     between its worker threads, so the reentrancy of eval() and the
    //     per-vertex spin locks have to produce the same answer as the
    //     single-threaded path.
    // =====================================================================

    TEST_F(LpCVTTest, multithreaded_evaluation_matches_singlethreaded) {
        if (GEO::Process::number_of_cores() < 2) {
            GTEST_SKIP() << "single core machine";
        }

        const std::vector<double> pts = sample_cube_surface(40, 8080);

        double f_st = 0.0;
        std::vector<double> g_st;
        evaluate(cube_, 4, false, pts, f_st, g_st);

        double f_mt = 0.0;
        std::vector<double> g_mt;
        GEO::Process::set_max_threads(GEO::Process::number_of_cores());
        evaluate(cube_, 4, false, pts, f_mt, g_mt);
        GEO::Process::set_max_threads(1);

        ASSERT_EQ(g_st.size(), g_mt.size());
        EXPECT_NEAR(f_mt, f_st, 1e-10 * std::fabs(f_st) + 1e-12);
        for (std::size_t i = 0; i < g_st.size(); ++i) {
            EXPECT_NEAR(g_mt[i], g_st[i], 1e-8 * (1.0 + std::fabs(g_st[i])))
                << "component " << i;
        }
    }

    // =====================================================================
    // T10: the anisotropy frame plumbing. Identity frames must reproduce the
    //      isotropic result exactly, which checks the frame indexing, the
    //      row-major layout and the range pinning that keeps frames aligned
    //      with their elements.
    // =====================================================================

    TEST_F(LpCVTTest, identity_frames_reproduce_the_isotropic_result) {
        const std::vector<double> pts = sample_cube_surface(24, 1212);

        double f_iso = 0.0;
        std::vector<double> g_iso;
        evaluate(cube_, 4, false, pts, f_iso, g_iso);

        std::vector<double> frames(cube_.facets.nb() * 9, 0.0);
        for (GEO::index_t t = 0; t < cube_.facets.nb(); ++t) {
            frames[9 * t + 0] = 1.0;
            frames[9 * t + 4] = 1.0;
            frames[9 * t + 8] = 1.0;
        }

        double f_frames = 0.0;
        std::vector<double> g_frames;
        {
            TestableLpCVT cvt(&cube_, 4, false);
            cvt.set_frames(frames, cube_.facets.nb());
            cvt.set_points(pts.size() / 3, pts.data());
            g_frames.assign(pts.size(), 0.0);
            cvt.funcgrad(
                GEO::index_t(pts.size()), const_cast<double*>(pts.data()),
                f_frames, g_frames.data()
                );
        }

        EXPECT_NEAR(f_frames, f_iso, 1e-12 * std::fabs(f_iso) + 1e-15);
        for (std::size_t i = 0; i < g_iso.size(); ++i) {
            EXPECT_NEAR(g_frames[i], g_iso[i], 1e-12 * (1.0 + std::fabs(g_iso[i])))
                << "component " << i;
        }
    }
}
