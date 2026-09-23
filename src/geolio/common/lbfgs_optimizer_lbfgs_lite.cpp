//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/1.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifdef GEOLIO_ENABLE_LBFGS_LITE

#include "lbfgs_optimizer_lbfgs_lite.h"
#include <iomanip>
#include <iostream>
#include <lbfgs_lite.hpp>

namespace geolio
{
    LbfgsOptimizerLBFGSLite* LbfgsOptimizerLBFGSLite::instance_ = nullptr;

    LbfgsOptimizerLBFGSLite::LbfgsOptimizerLBFGSLite(
        ) {
        instance_ = this;
    }

    void LbfgsOptimizerLBFGSLite::optimize(
        unsigned int n,
        double* x
        ) {
        double final_cost;
        Eigen::VectorXd X(n);
        for (unsigned int i = 0; i < n; ++i)
            X(i) = x[i];

        /* Set the minimization parameters */
        lbfgs::lbfgs_parameter_t params;
        params.g_epsilon = EPSG;
        params.past = PAST;
        params.delta = DELTA;
        params.max_iterations = MAX_ITERATION;

        /* Start minimization */
        const int ret = lbfgs::lbfgs_optimize(
            X,
            final_cost,
            LBFGS_Lite_cost_function,
            nullptr,
            LBFGS_Lite_monitor_progress,
            this,
            params);

        /* Report the result. */
        if (VERBOSE) {
            std::cout << std::setprecision(4)
              << "================================" << std::endl
              << "L-BFGS Optimization Returned: " << ret << std::endl
              << "Minimized Cost: " << final_cost << std::endl
              << "Optimal Variables: " << std::endl;
            // << x.transpose() << std::endl;
        }

        for (unsigned int i = 0; i < n; ++i)
            x[i] = X(i);
    }

    double LbfgsOptimizerLBFGSLite::LBFGS_Lite_cost_function(
        void *instance,
        const Eigen::VectorXd &X,
        Eigen::VectorXd &G
        ) {
        double f;
        std::vector<double> x(X.size());
        std::vector<double> g(G.size());
        for (unsigned int i = 0; i < X.size(); ++i)
            x[i] = X(i);

        instance_->funcgrad(X.size(), x.data(), f, g.data());

        for (unsigned int i = 0; i < G.size(); ++i)
            G(i) = g[i];

        return f;
    }

    int LbfgsOptimizerLBFGSLite::LBFGS_Lite_monitor_progress(
        void *instance,
        const Eigen::VectorXd &x,
        const Eigen::VectorXd &g,
        const double fx,
        const double step,
        const int k,
        const int ls
        ) {
        if (instance_->VERBOSE) {
            std::cout << std::setprecision(4)
              << "================================" << std::endl
              << "Iteration: " << k << std::endl
              << "Function Value: " << fx << std::endl
              << "Gradient Inf Norm: " << g.cwiseAbs().maxCoeff() << std::endl;
            // << "Variables: " << std::endl
            // << x.transpose() << std::endl;
        }

        return 0;
    }
}

#endif //GEOLIO_ENABLE_LBFGS_LITE