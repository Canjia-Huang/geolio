//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/3.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "lp_integration_simplex.h"

#include <geogram/basic/logger.h>

namespace geolio
{
    GEO::IntegrationSimplex_var create_lp_integration_simplex(
        const GEO::Mesh& mesh,
        const unsigned int p,
        const bool volumetric,
        const GEO::index_t nb_frames,
        const double* frames
        ) {
        // The reference implementation asserts p >= 2 && p <= 16 and p even, but
        // its command line driver accepted any even exponent and silently produced
        // a null objective (f == 0 and g == 0) for p == 0, because the dispatch
        // switch then matched no case. The exponent is validated here instead.
        if (p < 2 || p > 16 || (p / 2) * 2 != p) {
            GEO::Logger::err("LpCVT") << "Unsupported Lp norm exponent p = " << p
                << " (expected an even integer in [2, 16])" << std::endl;
            return GEO::IntegrationSimplex_var();
        }

        // A frame is a full 3x3 matrix, stored row-major, so 9 doubles per element.
        const GEO::index_t nb_comp_per_frame = (nb_frames != 0) ? 9 : 0;

        // Dispatch over the norm exponent, mirroring compute_F_Lp_internal() in
        // LpCVT/algebra/F_Lp.cpp: the volume and surface cases instantiate the same
        // integrand with two different measures.
        if (volumetric) {
            switch (p) {
            case 2:
                return GEO::IntegrationSimplex_var(
                    new LpIntegrationSimplex<2, LpTetVolume>(
                        mesh, true, nb_frames, nb_comp_per_frame, frames));
            case 4:
                return GEO::IntegrationSimplex_var(
                    new LpIntegrationSimplex<4, LpTetVolume>(
                        mesh, true, nb_frames, nb_comp_per_frame, frames));
            case 6:
                return GEO::IntegrationSimplex_var(
                    new LpIntegrationSimplex<6, LpTetVolume>(
                        mesh, true, nb_frames, nb_comp_per_frame, frames));
            case 8:
                return GEO::IntegrationSimplex_var(
                    new LpIntegrationSimplex<8, LpTetVolume>(
                        mesh, true, nb_frames, nb_comp_per_frame, frames));
            case 10:
                return GEO::IntegrationSimplex_var(
                    new LpIntegrationSimplex<10, LpTetVolume>(
                        mesh, true, nb_frames, nb_comp_per_frame, frames));
            case 12:
                return GEO::IntegrationSimplex_var(
                    new LpIntegrationSimplex<12, LpTetVolume>(
                        mesh, true, nb_frames, nb_comp_per_frame, frames));
            case 14:
                return GEO::IntegrationSimplex_var(
                    new LpIntegrationSimplex<14, LpTetVolume>(
                        mesh, true, nb_frames, nb_comp_per_frame, frames));
            case 16:
                return GEO::IntegrationSimplex_var(
                    new LpIntegrationSimplex<16, LpTetVolume>(
                        mesh, true, nb_frames, nb_comp_per_frame, frames));
            default:
                break;
            }
        } else {
            switch (p) {
            case 2:
                return GEO::IntegrationSimplex_var(
                    new LpIntegrationSimplex<2, LpTriArea>(
                        mesh, false, nb_frames, nb_comp_per_frame, frames));
            case 4:
                return GEO::IntegrationSimplex_var(
                    new LpIntegrationSimplex<4, LpTriArea>(
                        mesh, false, nb_frames, nb_comp_per_frame, frames));
            case 6:
                return GEO::IntegrationSimplex_var(
                    new LpIntegrationSimplex<6, LpTriArea>(
                        mesh, false, nb_frames, nb_comp_per_frame, frames));
            case 8:
                return GEO::IntegrationSimplex_var(
                    new LpIntegrationSimplex<8, LpTriArea>(
                        mesh, false, nb_frames, nb_comp_per_frame, frames));
            case 10:
                return GEO::IntegrationSimplex_var(
                    new LpIntegrationSimplex<10, LpTriArea>(
                        mesh, false, nb_frames, nb_comp_per_frame, frames));
            case 12:
                return GEO::IntegrationSimplex_var(
                    new LpIntegrationSimplex<12, LpTriArea>(
                        mesh, false, nb_frames, nb_comp_per_frame, frames));
            case 14:
                return GEO::IntegrationSimplex_var(
                    new LpIntegrationSimplex<14, LpTriArea>(
                        mesh, false, nb_frames, nb_comp_per_frame, frames));
            case 16:
                return GEO::IntegrationSimplex_var(
                    new LpIntegrationSimplex<16, LpTriArea>(
                        mesh, false, nb_frames, nb_comp_per_frame, frames));
            default:
                break;
            }
        }

        return GEO::IntegrationSimplex_var();
    }
}
