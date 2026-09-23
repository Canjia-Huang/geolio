//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/1.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_LBFGS_OPTIMIZER_BASE_H
#define GEOLIO_LBFGS_OPTIMIZER_BASE_H
#include <algorithm>

namespace geolio
{
    /**
     * Base interface for L-BFGS optimizers used in the project.
     *
     * Derived classes provide the objective/gradient evaluation, while this class
     * wraps the solver-specific entry points and common solver parameters.
     */
    class LbfgsOptimizerBase {
    public:
        /**
         * Construct a default optimizer instance.
         *
         * The constructor initializes solver-related state and leaves all tuning
         * parameters at their default values.
         */
        LbfgsOptimizerBase() = default;

        /**
         * Destroy the optimizer base interface.
         *
         * Declared virtual because this class is intended to be used as a
         * polymorphic base class.
         */
        virtual ~LbfgsOptimizerBase() = default;

        /**
         * Evaluate the objective function and its gradient at the current iterate.
         *
         * Derived classes must implement this function to provide the optimization
         * objective. The gradient array `g` must have length `n`.
         *
         * @param[in] n Number of optimization variables.
         * @param[in] x Current variable vector.
         * @param[out] f Objective function value at `x`.
         * @param[out] g Gradient vector at `x`.
         */
        virtual void funcgrad(unsigned int n, const double* x, double& f, double* g) = 0;

        /**
         * Minimize the objective function starting from the initial variables in `x`.
         *
         * The solver overwrites the input array in place with the optimized iterate and
         * returns the objective value at the final solution. The exact backend-specific
         * algorithm is implemented by derived classes.
         *
         * @param[in] n Number of optimization variables.
         * @param[in,out] x Variable array containing the initial guess on entry and the
         * optimized solution on exit.
         * @return Objective value at the final optimized point.
         */
        virtual void optimize(unsigned int n, double* x) = 0;

    protected:
        /**
         * Initialize the objective value and gradient buffer = 0.
         *
         * This helper resets the objective value and gradient entries before a new
         * evaluation begins.
         *
         * @param[in] n Number of optimization variables.
         * @param[out] f Objective value to initialize.
         * @param[out] g Gradient buffer to initialize.
         */
        static void initialize_f_g(unsigned int n, double& f, double* g) {
            f = 0.0;
            std::fill_n(g, n, 0.0);
        }
    };
}

#endif //GEOLIO_LBFGS_OPTIMIZER_BASE_H
