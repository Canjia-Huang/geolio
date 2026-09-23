//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/1.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <gtest/gtest.h>
#include <geolio/common/lbfgs_optimizer_geogram.h>
#include <geolio/common/lbfgs_optimizer_lbfgs_lite.h>

namespace
{
    class RosenbrockFunction {
    public:
        static double func(const GEO::index_t n, const double* x) {
            double val = 0;
            for (GEO::index_t i = 0; i < n-1; ++i)
                val += 100.0 * std::pow(x[i+1] - x[i] * x[i], 2) + std::pow(1 - x[i], 2);
            return val;
        }

        static void gradient(const GEO::index_t n, const double* x, double* grad) {
            std::fill_n(grad, n, 0.0);
            for (int i = 0; i < n-1; i++) {
                grad[i] += -400.0 * x[i] * (x[i+1] - x[i] * x[i]) - 2.0 * (1 - x[i]);
                grad[i+1] += 200.0 * (x[i+1] - x[i] * x[i]);
            }
        }
    };
}

namespace geolio::test
{
    template<typename optimizer>
    class RosenbrockOptimizer : public optimizer {
    public:
        void funcgrad(const unsigned int n, double *x, double& f, double* g) override {
            f = RosenbrockFunction::func(n, x);
            RosenbrockFunction::gradient(n, x, g);
        }
    };

    template class RosenbrockOptimizer<LbfgsOptimizerGeogram>;

    TEST(LbfgsOptimizerGeogramTest, RosenbrockFunction) {
        constexpr GEO::index_t n = 2;
        std::vector<double> x{-1.0, 2.0};
        const std::vector<double> x_gt{1.0, 1.0};

        RosenbrockOptimizer<LbfgsOptimizerGeogram> opt;
        opt.optimize(n, x.data());

        double f;
        std::vector<double> g(n);
        opt.funcgrad(n, x.data(), f, g.data());

        for (GEO::index_t i = 0; i < n; ++i)
            EXPECT_NEAR(x[i], x_gt[i], 1e-12);
        EXPECT_NEAR(f, 0.0, 1e-12);
    }

#ifdef GEOLIO_ENABLE_LBFGS_LITE
    template class RosenbrockOptimizer<LbfgsOptimizerLBFGSLite>;

    TEST(RosenbrockOptimizerLBFGSLite, RosenbrockFunction) {
        constexpr GEO::index_t n = 2;
        std::vector<double> x{-1.0, 2.0};
        const std::vector<double> x_gt{1.0, 1.0};

        RosenbrockOptimizer<LbfgsOptimizerLBFGSLite> opt;
        opt.optimize(n, x.data());

        double f;
        std::vector<double> g(n);
        opt.funcgrad(n, x.data(), f, g.data());

        for (GEO::index_t i = 0; i < n; ++i)
            EXPECT_NEAR(x[i], x_gt[i], 1e-12);
        EXPECT_NEAR(f, 0.0, 1e-12);
    }
#endif
}