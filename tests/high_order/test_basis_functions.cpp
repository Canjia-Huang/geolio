//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/2.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <gtest/gtest.h>
#include <geolio/high_order/basis_functions.h>

namespace geolio::test
{
    TEST(Lagrange1DTest, DeltaProperty) {
        const std::vector<double> nodes = {0.0, 0.5, 1.0, 2.0};
        const GEO::index_t n = nodes.size();

        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                const double val = Lagrange_val_1D(nodes[j], i, nodes);
                if (i == j)
                    EXPECT_DOUBLE_EQ(val, 1.0);
                else
                    EXPECT_NEAR(val, 0.0, 1e-15);
            }
        }
    }

    TEST(Lagrange1DTest, PartitionOfUnity) {
        const std::vector<double> nodes = {-1.0, 0.0, 1.0, 5.0};
        constexpr double test_x = 0.25;

        double sum = 0.0;
        for (int i = 0; i < nodes.size(); ++i)
            sum += Lagrange_val_1D(test_x, i, nodes);

        EXPECT_NEAR(sum, 1.0, 1e-15);
    }

    TEST(Lagrange1DTest, LinearInterpolation) {
        const std::vector<double> nodes = {0.0, 1.0};
        constexpr double x = 0.3;

        // L0(x) = (x-1)/(0-1) = 1-x, L1(x) = (x-0)/(1-0) = x
        EXPECT_DOUBLE_EQ(Lagrange_val_1D(x, 0, nodes), 0.7);
        EXPECT_DOUBLE_EQ(Lagrange_val_1D(x, 1, nodes), 0.3);
    }

    TEST(Lagrange1DTest, QuadraticInterpolation) {
        const std::vector<double> nodes = {0.0, 1.0, 2.0};
        constexpr double x = 0.5;

        // L1(x) = (x-0)(x-2) / (1-0)(1-2) = x(x-2)/(-1) = -0.5 * (-1.5) / -1 = 0.75
        EXPECT_NEAR(Lagrange_val_1D(x, 1, nodes), 0.75, 1e-15);
    }

    TEST(Lagrange1DTest, NonUniformNodes) {
        const std::vector<double> nodes = {0.0, 0.1, 0.9, 1.0};
        constexpr double x = 0.5;

        double sum = 0.0;
        for (int i = 0; i < nodes.size(); ++i)
            sum += Lagrange_val_1D(x, i, nodes);
        EXPECT_NEAR(sum, 1.0, 1e-15);
    }
}