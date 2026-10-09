//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/1.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_GAUSS_LEGENDRE_QUADRATURE_H
#define GEOLIO_GAUSS_LEGENDRE_QUADRATURE_H

#include <array>
#include <utility>
#include <geogram/basic/geometry.h>
#include <geogram/basic/numeric.h>
#include <cassert>
#include <Eigen/Eigenvalues>

namespace geolio
{
    // Each pair is {abscissa xi, weight wi} for integration on [-1, 1].
    // 1-point Gauss-Legendre rule (exact for polynomials up to degree 1).
    inline const std::vector<std::pair<double, double>> GAUSS_LEGENDRE_QUADRATURE_ORDER_1 = {
        {
            {0.0, 2.0}
        }
    };

    // 2-point Gauss-Legendre rule (exact for polynomials up to degree 3).
    inline const std::vector<std::pair<double, double>> GAUSS_LEGENDRE_QUADRATURE_ORDER_2 = {
        {
            {-1.0/std::sqrt(3.0),   1.0},
            {1.0/std::sqrt(3.0),    1.0}
        }
    };

    // 3-point Gauss-Legendre rule (exact for polynomials up to degree 5).
    inline const std::vector<std::pair<double, double>> GAUSS_LEGENDRE_QUADRATURE_ORDER_3 = {
        {
            {-std::sqrt(3.0/5.0),   5.0/9.0},
            {0.0,                   8.0/9.0},
            {std::sqrt(3.0/5.0),    5.0/9.0}
        }
    };

    // 4-point Gauss-Legendre rule (exact for polynomials up to degree 7).
    inline const std::vector<std::pair<double, double>> GAUSS_LEGENDRE_QUADRATURE_ORDER_4 = {
        {
            {-std::sqrt(525+70*std::sqrt(30))/35,  (18-std::sqrt(30))/36},
            {-std::sqrt(525-70*std::sqrt(30))/35,  (18+std::sqrt(30))/36},
            {std::sqrt(525-70*std::sqrt(30))/35,   (18+std::sqrt(30))/36},
            {std::sqrt(525+70*std::sqrt(30))/35,   (18-std::sqrt(30))/36}
        }
    };

    // 5-point Gauss-Legendre rule (exact for polynomials up to degree 9).
    inline const std::vector<std::pair<double, double>> GAUSS_LEGENDRE_QUADRATURE_ORDER_5 = {
        {
            {-std::sqrt(245+14*std::sqrt(70))/21,   (322-13*std::sqrt(70))/900},
            {-std::sqrt(245-14*std::sqrt(70))/21,   (322+13*std::sqrt(70))/900},
            {0,                                     128.0/225.0},
            {std::sqrt(245-14*std::sqrt(70))/21,    (322+13*std::sqrt(70))/900},
            {std::sqrt(245+14*std::sqrt(70))/21,    (322-13*std::sqrt(70))/900}
        }
    };

    /**
     * Get 1D Gauss-Legendre quadrature points and weights on [0, 1].
     *
     * For orders 1-5, this function uses precomputed tables.
     * For higher orders, it computes nodes and weights from the symmetric
     * Jacobi matrix eigen-decomposition.
     *
     * @param[in] order Number of quadrature points (positive integer).
     * @param[out] points_and_weights Output pairs `(xi, wi)`, where `xi` is the
     * quadrature node and `wi` is the corresponding weight on [0, 1].
     * @note The resulting rule is exact for polynomials up to degree `2*order-1`.
     */
    inline void get_Gauss_Legendre_quadrature(
        const GEO::index_t order,
        std::vector<std::pair<double, double>>& points_and_weights
        ) {
        switch (order) {
            case 1:
                points_and_weights = GAUSS_LEGENDRE_QUADRATURE_ORDER_1;
                break;
            case 2:
                points_and_weights = GAUSS_LEGENDRE_QUADRATURE_ORDER_2;
                break;
            case 3:
                points_and_weights = GAUSS_LEGENDRE_QUADRATURE_ORDER_3;
                break;
            case 4:
                points_and_weights = GAUSS_LEGENDRE_QUADRATURE_ORDER_4;
                break;
            case 5:
                points_and_weights = GAUSS_LEGENDRE_QUADRATURE_ORDER_5;
                break;
            default:
                Eigen::MatrixXd J = Eigen::MatrixXd::Zero(order, order);
                for (GEO::index_t i = 0, i_end = order-1; i < i_end; ++i) {
                    const double beta = (i+1.0) / std::sqrt(4.0 * (i+1.0) * (i+1.0) - 1.0);
                    J(i, i+1) = beta;
                    J(i+1, i) = beta;
                }
                const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(J);
                const auto& nodes = es.eigenvalues();
                const auto weights = 2.0 * es.eigenvectors().row(0).array().square();
                assert(nodes.size() == weights.size());

                points_and_weights.clear();
                points_and_weights.reserve(nodes.size());
                for (GEO::index_t i = 0, i_end = nodes.size(); i < i_end; ++i)
                    points_and_weights.emplace_back(nodes[i], weights[i]);
        }

        /* [-1, 1] -> [0, 1] */
        for (auto& [x, w] : points_and_weights) {
            x = 0.5*(x+1);
            w *= 0.5;
        }

        assert(points_and_weights.size() == order);
    }
}

#endif //GEOLIO_GAUSS_LEGENDRE_QUADRATURE_H
