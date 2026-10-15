//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/1.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//

#include <ranges>
#include <gtest/gtest.h>
#include <geolio/common/Gauss_Legendre_quadrature_cube.h>

namespace
{
    /* Calculate the analytic integral of the monomial x^a * y^b * z^c over [-1, 1]^3 */

    /**
     * Formula: if any of a, b, or c is odd, the integral is 0;
     *          if all are even, the result is 2/(a+1) * 2/(b+1) * 2/(c+1)
     */
    double analytical_integral(
        const GEO::index_t a,
        const GEO::index_t b,
        const GEO::index_t c
        ) {
        if (a % 2 != 0 || b % 2 != 0 || c % 2 != 0)
            return 0.0;
        return (2.0 / (a + 1)) * (2.0 / (b + 1)) * (2.0 / (c + 1));
    }

    /**
     * x^a * y^b * z^c
     */
    double numerical_integral(
        const GEO::index_t a,
        const GEO::index_t b,
        const GEO::index_t c,
        const std::vector<std::pair<GEO::vec3, double>>& points_and_weights
        ) {
        double numerical_integral = 0.0;
        for (const auto& [p, w] : points_and_weights)
            numerical_integral += w * std::pow(p.x, a) * std::pow(p.y, b) * std::pow(p.z, c);
        return numerical_integral;
    }
}

namespace geolio::test
{
    class GaussLegendreQuadratureCubeTest : public ::testing::TestWithParam<GEO::index_t> {
    public:
        void get_points_and_weights(
            const GEO::index_t degree
            ) {
            get_Gauss_Legendre_quadrature_cube(degree, points_and_weights_);

            /* [0, 1]^3 -> [-1, 1]^3 */
            for (auto& [p, w] : points_and_weights_) {
                p.x = 2*p.x-1;
                p.y = 2*p.y-1;
                p.z = 2*p.z-1;
                w *= 8;
            }

            /* An n-point 1D rule is exact up to degree 2n-1, so the 3D rule returned
             * for `degree` is built from the smallest n with 2n-1 >= degree. */
            num_points_ = (degree + 2) / 2;
            ASSERT_EQ(points_and_weights_.size(), num_points_*num_points_*num_points_);
        }

        void eval_nodes_symmetry(
            ) const {
            for (const auto& [p, w] : points_and_weights_) {
                EXPECT_GE(p.x, -1.0);
                EXPECT_LE(p.x, 1.0);
                EXPECT_GE(p.y, -1.0);
                EXPECT_LE(p.y, 1.0);
                EXPECT_GE(p.z, -1.0);
                EXPECT_LE(p.z, 1.0);

                for (const auto& [p0, w0] : points_and_weights_) {
                    if (GEO::distance2(GEO::vec3(-p.x, p.y, p.z), p0) < 1e-30)
                        EXPECT_NEAR(w, w0, 1e-15);
                    if (GEO::distance2(GEO::vec3(p.x, -p.y, p.z), p0) < 1e-30)
                        EXPECT_NEAR(w, w0, 1e-15);
                    if (GEO::distance2(GEO::vec3(p.x, p.y, -p.z), p0) < 1e-30)
                        EXPECT_NEAR(w, w0, 1e-15);
                    if (GEO::distance2(GEO::vec3(-p.x, -p.y, -p.z), p0) < 1e-30)
                        EXPECT_NEAR(w, w0, 1e-15);
                }
            }
        }

        void eval_weights_sum(
            ) {
            double sum = 0;
            for (const auto &w: points_and_weights_ | std::views::values)
                sum += w;
            EXPECT_NEAR(sum, 8, 1e-13);
        }

        void eval_centroid(
            ) {
            GEO::vec3 centroid(0, 0, 0);
            for (const auto& [p, w] : points_and_weights_)
                centroid += p * w;
            EXPECT_NEAR(centroid.x, 0, 1e-13);
            EXPECT_NEAR(centroid.y, 0, 1e-13);
            EXPECT_NEAR(centroid.z, 0, 1e-13);
        }

        void eval_k_polynomial(
            ) const {
            const GEO::index_t MAX_DEGREE = 2*num_points_-1;
            for (GEO::index_t a = 0; a <= MAX_DEGREE; ++a) {
                for (GEO::index_t b = 0; b <= MAX_DEGREE; ++b) {
                    for (GEO::index_t c = 0; c <= MAX_DEGREE; ++c) {
                        const double analytical = analytical_integral(a, b, c);
                        const double numerical = numerical_integral(a, b, c, points_and_weights_);
                        EXPECT_NEAR(analytical, numerical, 1e-13);
                    }
                }
            }
        }

        GEO::index_t num_points_ = 0;
        std::vector<std::pair<GEO::vec3, double>> points_and_weights_;
    };

    TEST_P(GaussLegendreQuadratureCubeTest, SupportsMultipleDegrees) {
        get_points_and_weights(GetParam());

        eval_nodes_symmetry();
        eval_weights_sum();
        eval_centroid();
        eval_k_polynomial();
    }

    INSTANTIATE_TEST_SUITE_P(DegreeTests, GaussLegendreQuadratureCubeTest,
        ::testing::Range(GEO::index_t(1), GEO::index_t(30)));
}