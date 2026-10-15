//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/1.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_GAUSS_LEGENDRE_QUADRATURE_CUBE_H
#define GEOLIO_GAUSS_LEGENDRE_QUADRATURE_CUBE_H

#include "Gauss_Legendre_quadrature.h"

namespace geolio
{
    /**
     * @brief Generate the Gauss-Legendre quadrature rule on the unit cube [0, 1]^3.
     *
     * This function builds the 3D tensor-product rule from a 1D Gauss-Legendre rule
     * that is exact for the requested degree.
     *
     * @param[in] degree Target polynomial degree of exactness in each variable.
     * @param[out] points_and_weights The resulting 3D quadrature points and weights,
     *                                stored as (point, weight) pairs.
     * @note The resulting rule is exact for all monomials `x^a y^b z^c` with
     *       `a <= degree`, `b <= degree` and `c <= degree`.
     */
    inline void get_Gauss_Legendre_quadrature_cube(
        const GEO::index_t degree,
        std::vector<std::pair<GEO::vec3, double>>& points_and_weights
        ) {
        std::vector<std::pair<double, double>> points_and_weights_1d;
        get_Gauss_Legendre_quadrature(degree, points_and_weights_1d);

        points_and_weights.clear();
        points_and_weights.reserve(std::pow(points_and_weights_1d.size(),3));
        for (const auto& [x, wx] : points_and_weights_1d) {
            for (const auto& [y, wy] : points_and_weights_1d) {
                for (const auto& [z, wz] : points_and_weights_1d)
                    points_and_weights.emplace_back(GEO::vec3(x, y, z), wx*wy*wz);
            }
        }
    }
}

#endif //GEOLIO_GAUSS_LEGENDRE_QUADRATURE_CUBE_H
