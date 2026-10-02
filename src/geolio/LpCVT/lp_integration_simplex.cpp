//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/3.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "lp_integration_simplex.h"

#include <geolio/common/log.h>
#include "lp_cvt.h"

namespace geolio
{
    GEO::IntegrationSimplex_var create_lp_integration_simplex(
        const GEO::Mesh& mesh,
        const GEO::index_t p,
        const bool volumetric,
        const GEO::index_t nb_matrices,
        const double* matrices
        ) {
        assert(LpCentroidalVoronoiTesselation::is_supported_norm_exponent(p));

        // A matrix is a full 3x3, stored row-major, so 9 doubles per element.
        const GEO::index_t nb_comp_per_matrix = (nb_matrices != 0) ? 9 : 0;

        // Dispatch over the norm exponent, mirroring compute_F_Lp_internal() in
        // LpCVT/algebra/F_Lp.cpp: the volume and surface cases instantiate the same
        // integrand with two different measures.
        if (volumetric) {
            switch (p) {
            case 2:
                return {new LpIntegrationSimplex<2, LpTetVolume>(
                        mesh, true, nb_matrices, nb_comp_per_matrix, matrices)};
            case 4:
                return {new LpIntegrationSimplex<4, LpTetVolume>(
                        mesh, true, nb_matrices, nb_comp_per_matrix, matrices)};
            case 6:
                return {new LpIntegrationSimplex<6, LpTetVolume>(
                        mesh, true, nb_matrices, nb_comp_per_matrix, matrices)};
            case 8:
                return {new LpIntegrationSimplex<8, LpTetVolume>(
                        mesh, true, nb_matrices, nb_comp_per_matrix, matrices)};
            case 10:
                return {new LpIntegrationSimplex<10, LpTetVolume>(
                        mesh, true, nb_matrices, nb_comp_per_matrix, matrices)};
            case 12:
                return {new LpIntegrationSimplex<12, LpTetVolume>(
                        mesh, true, nb_matrices, nb_comp_per_matrix, matrices)};
            case 14:
                return {new LpIntegrationSimplex<14, LpTetVolume>(
                        mesh, true, nb_matrices, nb_comp_per_matrix, matrices)};
            case 16:
                return {new LpIntegrationSimplex<16, LpTetVolume>(
                        mesh, true, nb_matrices, nb_comp_per_matrix, matrices)};
            default:
                break;
            }
        }
        else {
            switch (p) {
            case 2:
                return {new LpIntegrationSimplex<2, LpTriArea>(
                        mesh, false, nb_matrices, nb_comp_per_matrix, matrices)};
            case 4:
                return {new LpIntegrationSimplex<4, LpTriArea>(
                        mesh, false, nb_matrices, nb_comp_per_matrix, matrices)};
            case 6:
                return {new LpIntegrationSimplex<6, LpTriArea>(
                        mesh, false, nb_matrices, nb_comp_per_matrix, matrices)};
            case 8:
                return {new LpIntegrationSimplex<8, LpTriArea>(
                        mesh, false, nb_matrices, nb_comp_per_matrix, matrices)};
            case 10:
                return {new LpIntegrationSimplex<10, LpTriArea>(
                        mesh, false, nb_matrices, nb_comp_per_matrix, matrices)};
            case 12:
                return {new LpIntegrationSimplex<12, LpTriArea>(
                        mesh, false, nb_matrices, nb_comp_per_matrix, matrices)};
            case 14:
                return {new LpIntegrationSimplex<14, LpTriArea>(
                        mesh, false, nb_matrices, nb_comp_per_matrix, matrices)};
            case 16:
                return {new LpIntegrationSimplex<16, LpTriArea>(
                        mesh, false, nb_matrices, nb_comp_per_matrix, matrices)};
            default:
                break;
            }
        }

        return {};
    }
}
