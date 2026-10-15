//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/1.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//

#include <ranges>
#include <gtest/gtest.h>
#include <geolio/common/log.h>
#include <geolio/common/Hammer_tri_quadrature.h>

namespace
{
    double factorial(const int n) {
        double res = 1.0;
        for (int i = 2; i <= n; ++i) res *= i;
        return res;
    }

    /**
     * f(x, y) = x^a * y^b
     */
    double func(
        const int a,
        const int b,
        const GEO::vec2& p
        ) {
        return std::pow(p.x, a) * std::pow(p.y, b);
    }

    /**
     * Calculate the analytic integral of x^a * y^b over the reference triangle.
     *
     * Formula: \int_0^1 \int_0^{1-x} x^a * y^b dy dx = a! * b! / (a + b + 2)!
     */
    double get_triangle_analytic_integral(
        const int a,
        const int b
        ) {
        return factorial(a) * factorial(b) / factorial(a + b + 2);
    }
}

namespace geolio::test
{
    class HammerTriQuadratureTest : public ::testing::TestWithParam<int> {
    public:
        void get_points_and_weights(const GEO::index_t degree) {
            get_Hammer_tri_quadrature(degree, points_and_weights_);
        }

        void eval_weights_sum() {
            double sum = 0;
            for (const auto &w: points_and_weights_ | std::views::values)
                sum += w;
            EXPECT_NEAR(sum, 0.5, 1e-15);
        }

        void eval_polynomial(
            const GEO::index_t degree
            ) {
            for (int a = 0; a <= degree; ++a) {
                for (int b = 0; b <= degree; ++b) {
                    if (a+b > degree)
                        continue;

                    const double analytic = get_triangle_analytic_integral(a, b);

                    double numeric = 0.0;
                    for (const auto& [p, w] : points_and_weights_)
                        numeric += w * func(a, b, p);

                    EXPECT_NEAR(numeric, analytic, 1e-11);
                }
            }
        }

        std::vector<std::pair<GEO::vec2, double>> points_and_weights_;
    };

    TEST_P(HammerTriQuadratureTest, SupportsMultipleDegrees) {
        const GEO::index_t k = GetParam();
        get_points_and_weights(k);

        eval_weights_sum();
        eval_polynomial(k);
    }

    INSTANTIATE_TEST_SUITE_P(DegreeTests, HammerTriQuadratureTest,
        ::testing::Values(1, 2, 3, 4, 5, 6, 7, 8, 9, 11, 13, 15, 10, 12, 14, 16, 17, 20));
}