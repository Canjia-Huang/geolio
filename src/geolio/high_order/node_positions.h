//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/2.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_NODE_POSITIONS_H
#define GEOLIO_NODE_POSITIONS_H
#include <cassert>
#include <vector>
#include <cmath>

namespace geolio
{

    /**
     * @brief Generate equally spaced interpolation nodes on [0, 1].
     * @param order Polynomial order; expects order > 0.
     * @param[out] node_positions Output node coordinates, resized to order + 1.
     */
    inline void compute_equally_spaced_nodes(
        const unsigned int order,
        std::vector<double>& node_positions
        ) {
        assert(order > 0);
        node_positions.assign(order+1, 0.0);
        const double INV_ORDER = 1.0/order;
        for (unsigned int i = 0, i_end = order+1; i < i_end; ++i)
            node_positions[i] = i*INV_ORDER;
    }

    /**
     * @brief Generate Chebyshev-Gauss nodes mapped to [0, 1].
     * @param order Polynomial order; expects order > 0.
     * @param[out] node_positions Output node coordinates, resized to order + 1.
     */
    inline void compute_Chebyshev_Gauss_nodes(
        const unsigned int order,
        std::vector<double>& node_positions
        ) {
        assert(order > 0);
        node_positions.assign(order+1, 0.0);
        for (unsigned int i = 0, i_end = order+1; i < i_end; ++i)
            node_positions[i] = 0.5 * (1.0 - std::cos((2.0*i+1.0)*M_PI/(2.0*order+2.0)));
    }

    /**
     * @brief Generate Chebyshev-Gauss-Lobatto nodes on [0, 1] including both endpoints.
     * @param order Polynomial order; expects order > 0.
     * @param[out] node_positions Output node coordinates, resized to order + 1.
     */
    inline void compute_Chebyshev_Gauss_Lobatto_nodes(
        const unsigned int order,
        std::vector<double>& node_positions
        ) {
        assert(order > 0);
        node_positions.assign(order+1, 0.0);
        for (unsigned int i = 1; i < order; ++i)
            node_positions[i] = 0.5 * (1.0 - std::cos(i*M_PI/order));
        node_positions[0] = 0.0;
        node_positions[order] = 1.0;
    }

    /**
     * @brief Generate Legendre-Gauss-Lobatto nodes on [0, 1] using hard-coded values.
     * @param order Polynomial order in [2, 6]; validated by an assertion.
     * @param[out] node_positions Output node coordinates, resized to order + 1.
     */
    inline void compute_Legendre_Gauss_Lobatto_nodes(
        const unsigned int order,
        std::vector<double>& node_positions
        ) {
        assert(order > 0 && order <= 6);
        node_positions.assign(order+1, 0.0);
        switch (order) {
            case 1:
                break;
            case 2:
                node_positions[1] = 0.5;
                break;
            case 3:
                node_positions[1] = 0.276393202250021;
                node_positions[2] = 0.723606797749979;
                break;
            case 4:
                node_positions[1] = 0.172673164646011;
                node_positions[2] = 0.5;
                node_positions[3] = 0.827326835353989;
                break;
            case 5:
                node_positions[1] = 0.117472338046291;
                node_positions[2] = 0.357384241766285;
                node_positions[3] = 0.642615758233715;
                node_positions[4] = 0.882527661953709;
                break;
            case 6:
                node_positions[1] = 0.084888051860765;
                node_positions[2] = 0.265575603264643;
                node_positions[3] = 0.5;
                node_positions[4] = 0.734424396735357;
                node_positions[5] = 0.915111948139235;
                break;
            default:
                break;
        }
        node_positions[0] = 0.0;
        node_positions[order] = 1.0;
    }
}

#endif //GEOLIO_NODE_POSITIONS_H
