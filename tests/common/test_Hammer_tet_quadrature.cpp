//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/1.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//

#include <ranges>
#include <gtest/gtest.h>
#include <geolio/common/Hammer_tet_quadrature.h>

namespace
{
    double factorial(const int n) {
        double res = 1.0;
        for (int i = 2; i <= n; ++i) res *= i;
        return res;
    }

    /**
     * f(x, y, z) = x^a * y^b * z^c
     */
    double func(
        const int a,
        const int b,
        const int c,
        const GEO::vec3& p
        ) {
        return std::pow(p.x, a) * std::pow(p.y, b) * std::pow(p.z, c);
    }

    /**
    * Calculate the analytic integral of x^a * y^b * z^c over the reference triangle.
    *
    * Formula: \int_0^1 \int_0^{1-x} x^a * y^b * z^c dz dy dx = a! * b! * c! / (a + b + c + 3)!
    */
    double get_tetrahedron_analytic_integral(
        const int a,
        const int b,
        const int c
        ) {
        return factorial(a) * factorial(b) * factorial(c) / factorial(a + b + c + 3);
    }
}

namespace geolio::test
{
    class HammerTetQuadratureTest : public ::testing::TestWithParam<int> {
    public:
        void get_points_and_weights(const GEO::index_t degree) {
            get_Hammer_tet_quadrature(degree, points_and_weights_);
        }

        void eval_weights_sum() {
            double sum = 0;
            for (const auto &w: points_and_weights_ | std::views::values)
                sum += w;
            EXPECT_NEAR(sum, 1.0/6.0, 1e-15);
        }

        void eval_polynomial(
            const GEO::index_t degree
            ) {
            for (int a = 0; a <= degree; ++a) {
                for (int b = 0; b <= degree; ++b) {
                    for (int c = 0; c <= degree; ++c) {
                        if (a+b+c > degree)
                            continue;

                        const double analytic = get_tetrahedron_analytic_integral(a, b, c);

                        double numeric = 0.0;
                        for (const auto& [p, w] : points_and_weights_)
                            numeric += w * func(a, b, c, p);

                        EXPECT_NEAR(numeric, analytic, 1e-11);
                    }
                }
            }
        }

        std::vector<std::pair<GEO::vec3, double>> points_and_weights_;
    };

    TEST_P(HammerTetQuadratureTest, SupportsMultipleDegrees) {
        const GEO::index_t k = GetParam();
        get_points_and_weights(k);

        eval_weights_sum();
        eval_polynomial(k);
    }

    INSTANTIATE_TEST_SUITE_P(DegreeTests, HammerTetQuadratureTest,
        ::testing::Values(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 15));
}