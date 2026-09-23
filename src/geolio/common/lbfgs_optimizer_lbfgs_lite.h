//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/1.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_LBFGS_OPTIMIZER_LBFGS_LITE_H
#define GEOLIO_LBFGS_OPTIMIZER_LBFGS_LITE_H

#ifdef GEOLIO_ENABLE_LBFGS_LITE

#include "lbfgs_optimizer_base.h"
#include <Eigen/Dense>

namespace geolio
{
    class LbfgsOptimizerLBFGSLite: public LbfgsOptimizerBase {
    public:
        /**
         * @copydoc geolio::LbfgsOptimizerBase::LbfgsOptimizerBase
         *
         * Initializes the static callback target used by the LBFGS-Lite backend.
         */
        LbfgsOptimizerLBFGSLite();

        /**
         * @copydoc geolio::LbfgsOptimizerBase::optimize
         *
         * The LBFGS-Lite backend updates the input vector in place and returns the
         * objective value at the final iterate.
         */
        void optimize(unsigned int n, double* x) override;

        bool         VERBOSE = true;
        double       EPSG = 0.0; // Gradient norm tolerance.
        int          PAST = 3; // Number of previous iterations kept for convergence checks.
        double       DELTA = 1.0e-8; // Convergence threshold for objective changes.
        int          MAX_ITERATION = 1000; // Maximum number of solver iterations.

    private:
        static LbfgsOptimizerLBFGSLite* instance_;

        /**
         * @brief Adapter for LBFGS-Lite cost callback.
         *
         * Converts LBFGS-Lite's Eigen::VectorXd input into the format expected by the
         * QualityOptimization instance (retrieved through the provided instance pointer),
         * calls the instance's objective/gradient computation, fills G with the gradient
         * and returns the objective value.
         *
         * @param[in] instance User pointer (should point to a QualityOptimization instance).
         * @param[in] X Current variable vector from LBFGS-Lite.
         * @param[out] G Gradient vector to be filled by this function.
         * @return Objective function value evaluated at X.
         */
        static double LBFGS_Lite_cost_function(
            void *instance, const Eigen::VectorXd &X, Eigen::VectorXd &G);

        /**
         * @brief LBFGS-Lite progress monitoring callback.
         *
         * Called by LBFGS-Lite after each main iteration to report progress. Implementations
         * can log iteration statistics or implement custom stopping logic. The return value
         * should follow LBFGS-Lite conventions (0 to continue, non-zero to request termination).
         *
         * @param[in] instance User pointer (should point to a QualityOptimization instance).
         * @param[in] x Current variable vector.
         * @param[in] g Current gradient vector.
         * @param[in] fx Current objective value.
         * @param[in] step Step length used in the last update.
         * @param[in] k Number of completed main iterations.
         * @param[in] ls Line-search iteration count or status.
         * @return 0 to continue optimization, non-zero to stop early (per LBFGS-Lite API).
         */
        static int LBFGS_Lite_monitor_progress(
            void *instance, const Eigen::VectorXd &x, const Eigen::VectorXd &g,
            double fx, double step, int k, int ls);
    };
}

#endif //GEOLIO_ENABLE_LBFGS_LITE

#endif //GEOLIO_LBFGS_OPTIMIZER_LBFGS_LITE_H
