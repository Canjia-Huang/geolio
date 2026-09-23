//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/1.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_LBFGS_OPTIMIZER_GEOGRAM_H
#define GEOLIO_LBFGS_OPTIMIZER_GEOGRAM_H
#include "lbfgs_optimizer_base.h"
#include <geogram/basic/geometry.h>

namespace geolio
{
    class LbfgsOptimizerGeogram : public LbfgsOptimizerBase {
    public:
        /**
         * @copydoc geolio::LbfgsOptimizerBase::LbfgsOptimizerBase
         *
         * Initializes the static callback target used by the Geogram HLBFGS backend.
         */
        LbfgsOptimizerGeogram();

        /**
         * @copydoc geolio::LbfgsOptimizerBase::optimize
         *
         * The Geogram HLBFGS backend updates the input vector in place and returns the
         * objective value at the final iterate.
         */
        void optimize(unsigned int n, double* x) override;

        bool GEOGRAM_DEBUG = true;
        GEO::index_t VERBOSE = 1; // Print progress every N iterations, 0 -> off
        double       EPSG = 0.0; // Gradient norm tolerance.
        GEO::index_t INNER_ITERATIONS_NB = 7; // Number of inner iterations per outer step.
        GEO::index_t MAX_ITERATION = 1000; // Maximum number of solver iterations.

    private:
        /**
         * @brief Pointer to the active LBFGSOptimizer instance used by C-style callbacks.
         *
         * This static pointer allows static callback functions to forward calls to the
         * appropriate instance methods. Current design assumes a single active optimizer
         * at a time; change to per-call user-data if multiple concurrent instances are needed.
         */
        static LbfgsOptimizerGeogram* instance_;

        /**
         * @brief Geogram HLBFGS progress callback invoked before/at each new iteration.
         *
         * Receives the current iterate x, objective f, gradient g and gradient norm gnorm.
         * Typical uses include logging and optional custom stopping checks.
         *
         * @param[in] n Dimensionality of x and g.
         * @param[in] x Current variable vector (read-only).
         * @param[in] f Current objective value.
         * @param[in] g Current gradient vector (read-only).
         * @param[in] gnorm Euclidean norm of the gradient.
         */
        static void geogram_HLBFGS_newiteration_CB(
            GEO::index_t n, const double* x, double f, const double* g, double gnorm);

        /**
         * @brief Geogram HLBFGS callback to evaluate objective and gradient.
         *
         * The optimizer calls this when it needs the objective value and gradient for
         * the current iterate. Implementations should translate x into control point
         * positions (if necessary), compute f and g, and return them to the solver.
         *
         * @param[in] n Number of optimization variables.
         * @param[in,out] x Variable vector provided by the solver (treated as read-only).
         * @param[out] f Objective value computed for x.
         * @param[out] g Gradient vector for x.
         */
        static void geogram_HLBFGS_funcgrad_CB(
            GEO::index_t n, double* x, double& f, double* g);
    };
}

#endif //GEOLIO_LBFGS_OPTIMIZER_GEOGRAM_H
