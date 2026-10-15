//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/15.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_GAUSS_JACOBI_QUADRATURE_H
#define GEOLIO_GAUSS_JACOBI_QUADRATURE_H

#include <utility>
#include <vector>
#include <cmath>
#include <cassert>
#include <geogram/basic/numeric.h>
#include <Eigen/Eigenvalues>
#include <geolio/common/Gauss_Legendre_quadrature.h>

namespace geolio
{
    /**
     * Get 1D Gauss-Jacobi quadrature points and weights on [0, 1] for the weight
     * function `(1 - x)^alpha`, for the requested degree of exactness.
     *
     * The returned rule is the smallest Gauss-Jacobi rule with parameters
     * `(alpha, 0)` that is exact for the requested degree, i.e. it has
     * `ceil((degree + 1) / 2)` points, because an n-point Gauss rule is exact up
     * to degree `2n - 1`. The nodes and weights are mapped from `[-1, 1]` to
     * `[0, 1]` and computed from the eigen-decomposition of the symmetric
     * tridiagonal Jacobi matrix (Golub-Welsch), so any degree is available; for
     * `alpha == 0` the weight function is identically one and the result is the
     * Gauss-Legendre rule returned by get_Gauss_Legendre_quadrature().
     *
     * @param[in] degree Target polynomial degree of exactness.
     * @param[in] alpha Exponent of the weight function, `alpha > -1`.
     * @param[out] points_and_weights Output pairs `(xi, wi)`, where `xi` is the
     * quadrature node and `wi` is the corresponding weight, such that
     * `sum_i wi * f(xi)` approximates `integral_0^1 f(x) (1 - x)^alpha dx`.
     * @note The resulting rule is exact for polynomials up to degree `2*num_points-1`,
     * which is at least @p degree.
     */
    inline void get_Gauss_Jacobi_quadrature(
        const GEO::index_t degree,
        const double alpha,
        std::vector<std::pair<double, double>>& points_and_weights
        ) {
        assert(alpha > -1.0);

        /* An n-point Gauss rule is exact up to degree `2n - 1`, so this is the
         * smallest number of points that covers the requested degree. */
        const GEO::index_t num_points = (degree + 2) / 2;

        /* The weight function is identically one, i.e. the Jacobi weight is a
         * Legendre weight: reuse the tabulated and computed rules directly. */
        if (alpha == 0.0) {
            get_Gauss_Legendre_quadrature(degree, points_and_weights);
            return;
        }

        /* Symmetric tridiagonal Jacobi matrix of the Jacobi polynomials with
         * parameters `(alpha, beta = 0)`: its eigenvalues are the nodes on
         * `[-1, 1]`, and `mu_0` times the squared first components of its
         * eigenvectors are the corresponding weights. */
        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(num_points, num_points);
        for (GEO::index_t k = 0, k_end = num_points; k < k_end; ++k) {
            constexpr double beta = 0.0;
            const auto kd = static_cast<double>(k);
            J(k, k) = k == 0
                ? (beta - alpha) / (alpha + beta + 2.0)
                : (beta*beta - alpha*alpha) / ((2.0*kd + alpha + beta) * (2.0*kd + alpha + beta + 2.0));

            if (k + 1 < num_points) {
                const double k1 = kd + 1.0;
                const double beta_k = 2.0 / (2.0*kd + alpha + beta + 2.0)
                    * std::sqrt(k1 * (k1 + alpha) * (k1 + beta) * (k1 + alpha + beta)
                        / ((2.0*kd + alpha + beta + 1.0) * (2.0*kd + alpha + beta + 3.0)));
                J(k, k+1) = beta_k;
                J(k+1, k) = beta_k;
            }
        }
        const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(J);
        const auto& nodes = es.eigenvalues();

        /* `mu_0 = integral_{-1}^{1} (1 - x)^alpha dx = 2^(alpha + 1) / (alpha + 1)`
         * and the mapping `[-1, 1] -> [0, 1]` divides the weights by
         * `2^(alpha + 1)`, which leaves `wi = (first eigenvector component)^2 / (alpha + 1)`
         * and a total weight `integral_0^1 (1 - x)^alpha dx = 1 / (alpha + 1)`. */
        const Eigen::ArrayXd weights = es.eigenvectors().row(0).array().square() / (alpha + 1.0);
        assert(nodes.size() == weights.size());

        points_and_weights.clear();
        points_and_weights.reserve(nodes.size());
        for (GEO::index_t k = 0, k_end = nodes.size(); k < k_end; ++k) {
            /* `[-1, 1] -> [0, 1]` */
            points_and_weights.emplace_back(0.5 * (nodes[k] + 1.0), weights[k]);
        }

        assert(points_and_weights.size() == num_points);
    }
}

#endif //GEOLIO_GAUSS_JACOBI_QUADRATURE_H
