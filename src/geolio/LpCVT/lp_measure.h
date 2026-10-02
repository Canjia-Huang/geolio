//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/3.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_LP_MEASURE_H
#define GEOLIO_LP_MEASURE_H

#include <geogram/basic/geometry.h>
#include <cmath>

namespace geolio
{
    /**
     * @brief Computes the (signed) volume of a tetrahedron together with its
     *        gradients with respect to the three edge vectors.
     * @details See Appendix B.1 in the paper.
     *          Note: the 1/6 factor is ignored, i.e. the returned value is the
     *          determinant ``dot(U1, cross(U2, U3))``, which is six times the
     *          volume of the tetrahedron ``(0, U1, U2, U3)``.
     * @note The measure is @b signed on purpose. The volumetric integration
     *       simplices produced by ``GEO::RestrictedVoronoiDiagram`` are
     *       documented as "geometrically incorrect but algebraically correct",
     *       meaning that their signed volumes sum as the volume of the
     *       restricted Voronoi cell. Keeping the sign here is what makes that
     *       algebraic cancellation work.
     */
    class LpTetVolume {
    public:
        /**
         * @brief Evaluates the signed tetrahedron volume and its gradient.
         * @details dT/dU1 = U2 x U3, dT/dU2 = U3 x U1 and dT/dU3 = U1 x U2,
         *          which is the exact gradient of ``dot(U1, cross(U2, U3))``.
         * @param[in] U1 First edge vector of the tetrahedron.
         * @param[in] U2 Second edge vector of the tetrahedron.
         * @param[in] U3 Third edge vector of the tetrahedron.
         * @param[out] dTdU1 Gradient of the measure with respect to @p U1.
         * @param[out] dTdU2 Gradient of the measure with respect to @p U2.
         * @param[out] dTdU3 Gradient of the measure with respect to @p U3.
         * @return The signed measure ``dot(U1, cross(U2, U3))``.
         */
        double operator()(
            const GEO::vec3& U1,
            const GEO::vec3& U2,
            const GEO::vec3& U3,
            GEO::vec3& dTdU1,
            GEO::vec3& dTdU2,
            GEO::vec3& dTdU3
            ) const {
            dTdU1 = GEO::cross(U2, U3);
            dTdU2 = GEO::cross(U3, U1);
            dTdU3 = GEO::cross(U1, U2);
            return GEO::dot(U1, dTdU1);
        }
    };

    /**
     * @brief Computes the area of a triangle together with its gradients with
     *        respect to the three edge vectors.
     * @details See Appendix B.1 in the paper.
     *          Note: the 1/2 factor is ignored, i.e. the returned value is
     *          ``length(cross(U1 - U3, U2 - U3))``, which is twice the area of
     *          the triangle ``(U1, U2, U3)``.
     * @note Unlike LpTetVolume this measure is always non-negative, so the
     *       orientation of the surface integration triangles does not matter.
     */
    class LpTriArea {
    public:
        /**
         * @brief Evaluates the triangle area and its gradient.
         * @details The gradients are the exact derivatives of
         *          ``length(cross(U1 - U3, U2 - U3))``:
         *          dT/dU1 = N x (U3 - U2), dT/dU2 = N x (U1 - U3) and
         *          dT/dU3 = N x (U2 - U1), with ``N`` the unit normal of the
         *          triangle.
         * @note Degenerate configurations (area below 1e-10) yield a zero
         *       gradient; the 1e-10 threshold is inherited from the original
         *       LpCVT implementation and is kept as-is so that the ported
         *       objective reproduces the reference one.
         * @param[in] U1 First vertex of the triangle (relative to the origin).
         * @param[in] U2 Second vertex of the triangle (relative to the origin).
         * @param[in] U3 Third vertex of the triangle (relative to the origin).
         * @param[out] dTdU1 Gradient of the measure with respect to @p U1.
         * @param[out] dTdU2 Gradient of the measure with respect to @p U2.
         * @param[out] dTdU3 Gradient of the measure with respect to @p U3.
         * @return The measure ``length(cross(U1 - U3, U2 - U3))``.
         */
        double operator()(
            const GEO::vec3& U1,
            const GEO::vec3& U2,
            const GEO::vec3& U3,
            GEO::vec3& dTdU1,
            GEO::vec3& dTdU2,
            GEO::vec3& dTdU3
            ) const {
            GEO::vec3 N = GEO::cross(U1 - U3, U2 - U3);
            const double T = GEO::length(N);
            if (::fabs(T) < 1e-10) {
                dTdU1 = GEO::vec3(0.0, 0.0, 0.0);
                dTdU2 = GEO::vec3(0.0, 0.0, 0.0);
                dTdU3 = GEO::vec3(0.0, 0.0, 0.0);
            }
            else {
                N = (1.0 / T) * N;
                dTdU1 = GEO::cross(N, U3 - U2);
                dTdU2 = GEO::cross(N, U1 - U3);
                dTdU3 = GEO::cross(N, U2 - U1);
            }
            return T;
        }
    };

    /**
     * @brief Constant factor relating the surfacic LpCVT objective to the
     *        physical energy ``integral over the simplex of ||M (x - p0)||_p^p``.
     * @details The original LpCVT code deliberately omits two factors from its
     *          energy: the binomial coefficient ``(n + p choose n)`` (see the
     *          remark in the header of IntegrationSimplex) and the 1/2 factor of
     *          the triangle area. Their product is exactly
     *          ``(p + 1) * (p + 2)``, so dividing the computed objective by this
     *          constant yields the physically scaled energy. For ``p == 2`` this
     *          is 12, which matches the ratio measured against Geogram's
     *          built-in L2 CVT objective.
     * @param[in] p The integer norm exponent (even, 2 <= p <= 16).
     * @return The constant ``(p + 1) * (p + 2)``.
     */
    constexpr double lp_surface_energy_normalization(const GEO::index_t p) {
        return static_cast<double>(p + 1) * static_cast<double>(p + 2);
    }

    /**
     * @brief Constant factor relating the volumetric LpCVT objective to the
     *        physical energy ``integral over the simplex of ||M (x - p0)||_p^p``.
     * @details Same reasoning as lp_surface_energy_normalization(), with the
     *          omitted 1/6 factor of the tetrahedron volume in place of the
     *          1/2 factor of the triangle area, which gives
     *          ``(p + 1) * (p + 2) * (p + 3)``. For ``p == 2`` this is 60, which
     *          matches the ratio measured against Geogram's built-in L2 CVT
     *          objective in volumetric mode.
     * @param[in] p The integer norm exponent (even, 2 <= p <= 16).
     * @return The constant ``(p + 1) * (p + 2) * (p + 3)``.
     */
    constexpr double lp_volume_energy_normalization(const GEO::index_t p) {
        return static_cast<double>(p + 1) * static_cast<double>(p + 2) * static_cast<double>(p + 3);
    }
}

#endif //GEOLIO_LP_MEASURE_H
