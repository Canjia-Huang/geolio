//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/15.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//

#include <cmath>
#include <ranges>
#include <tuple>
#include <gtest/gtest.h>
#include <geolio/common/Gauss_Jacobi_quadrature.h>
#include <geolio/common/Gauss_Legendre_quadrature.h>

namespace
{
    /**
     * Calculate the analytic integral of the monomial x^k against (1 - x)^alpha.
     *
     * Formula: \int_0^1 x^k (1 - x)^alpha dx
     *        = k! * alpha! / (k + alpha + 1)!
     *        = Gamma(k + 1) * Gamma(alpha + 1) / Gamma(k + alpha + 2)
     */
    double analytical_integral(
        const int k,
        const double alpha
        ) {
        return std::tgamma(k + 1.0) * std::tgamma(alpha + 1.0) / std::tgamma(k + alpha + 2.0);
    }

    /**
     * x^k
     */
    double numerical_integral(
        const int k,
        const std::vector<std::pair<double, double>>& points_and_weights
        ) {
        double sum = 0.0;
        for (const auto& [p, w] : points_and_weights)
            sum += w * std::pow(p, k);
        return sum;
    }
}

namespace geolio::test
{
    class GaussJacobiQuadratureTest : public ::testing::TestWithParam<std::tuple<GEO::index_t, double>> {
    public:
        void get_points_and_weights() {
            const GEO::index_t degree = std::get<0>(GetParam());
            alpha_ = std::get<1>(GetParam());

            geolio::get_Gauss_Jacobi_quadrature(degree, alpha_, points_and_weights_);

            /* An n-point rule is exact up to degree 2n-1, so the rule returned for
             * `degree` has the smallest n with 2n-1 >= degree. */
            num_points_ = (degree + 2) / 2;
            ASSERT_EQ(points_and_weights_.size(), num_points_);
        }

        void eval_nodes_range(
            ) const {
            for (const auto& [x, w] : points_and_weights_) {
                EXPECT_GE(x, 0.0);
                EXPECT_LE(x, 1.0);
                EXPECT_GT(w, 0.0);
            }
            for (GEO::index_t i = 1, i_end = points_and_weights_.size(); i < i_end; ++i)
                EXPECT_LT(points_and_weights_[i-1].first, points_and_weights_[i].first);
        }

        void eval_weights_sum(
            ) const {
            double sum = 0;
            for (const auto &w: points_and_weights_ | std::views::values)
                sum += w;
            EXPECT_NEAR(sum, 1.0/(alpha_+1.0), 1e-14);
        }

        void eval_k_polynomial(
            ) const {
            for (GEO::index_t k = 0, k_end = 2*num_points_-1; k <= k_end; ++k) {
                const double analytical = analytical_integral(int(k), alpha_);
                const double numerical = numerical_integral(int(k), points_and_weights_);
                EXPECT_NEAR(analytical, numerical, 1e-14);
            }
        }

        double alpha_ = 0.0;
        GEO::index_t num_points_ = 0;
        std::vector<std::pair<double, double>> points_and_weights_;
    };

    TEST_P(GaussJacobiQuadratureTest, SupportsMultipleDegrees) {
        get_points_and_weights();

        eval_nodes_range();
        eval_weights_sum();
        eval_k_polynomial();
    }

    INSTANTIATE_TEST_SUITE_P(DegreeTests, GaussJacobiQuadratureTest,
        ::testing::Combine(
            ::testing::Range(GEO::index_t(1), GEO::index_t(30)),
            ::testing::Values(0.0, 0.5, 1.0, 2.0, 3.0)));

    /* An identically one weight function is the Legendre case. */
    TEST(GaussJacobiZeroAlphaTest, MatchesGaussLegendre) {
        for (const GEO::index_t degree : {1, 2, 3, 4, 5, 6, 11, 20, 29}) {
            std::vector<std::pair<double, double>> jacobi, legendre;
            geolio::get_Gauss_Jacobi_quadrature(degree, 0.0, jacobi);
            geolio::get_Gauss_Legendre_quadrature(degree, legendre);

            ASSERT_EQ(jacobi.size(), legendre.size());
            for (GEO::index_t i = 0, i_end = jacobi.size(); i < i_end; ++i) {
                EXPECT_DOUBLE_EQ(jacobi[i].first, legendre[i].first);
                EXPECT_DOUBLE_EQ(jacobi[i].second, legendre[i].second);
            }
        }
    }
}
