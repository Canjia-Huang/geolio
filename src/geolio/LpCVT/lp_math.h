//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/3.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_LP_MATH_H
#define GEOLIO_LP_MATH_H
#include <geogram/basic/geometry.h>

/**
 * @file lp_math.h
 * @brief The handful of small linear-algebra helpers used by the LpCVT port.
 * @details Geogram provides ``GEO::dot``, ``GEO::cross`` and ``GEO::length`` for
 *          ``GEO::vec3``, but it has no equivalent for the component-wise
 *          products and the matrix/vector products that the LpCVT integrand is
 *          written in terms of. These helpers are ported verbatim from
 *          ``LpCVT/common/types.h`` so that the ported expressions stay
 *          recognisably identical to the reference implementation.
 * @note ``GEO::mat3`` is ``GEO::Matrix<3, double>``, which stores its
 *       coefficients row-major (``coeff_[i][j]``, row ``i`` varying slowest), so
 *       the conventions below match both ``LpCVT/common/matrix.h`` and
 *       ``GEO::mat3``.
 */
namespace geolio
{
    /**
     * @brief Computes ``V = M * U``.
     * @param[in] M A 3x3 matrix (row-major).
     * @param[in] U The vector to transform.
     * @param[out] V The transformed vector.
     */
    inline void lp_matvecmul(
        const GEO::mat3& M,
        const GEO::vec3& U,
        GEO::vec3& V
        ) {
        V.x = M(0, 0) * U.x + M(0, 1) * U.y + M(0, 2) * U.z;
        V.y = M(1, 0) * U.x + M(1, 1) * U.y + M(1, 2) * U.z;
        V.z = M(2, 0) * U.x + M(2, 1) * U.y + M(2, 2) * U.z;
    }

    /**
     * @brief Computes ``V = M^T * U``.
     * @details The LpCVT gradient is naturally expressed in terms of the
     *          transformed vectors ``U_k = M (p_k - p0)``. Since
     *          ``grad(F(MX)) = (grad F)^T M``, going back to the original
     *          variables requires the transposed matrix.
     * @param[in] M A 3x3 matrix (row-major).
     * @param[in] U The vector to transform.
     * @param[out] V The transformed vector.
     */
    inline void lp_matTvecmul(
        const GEO::mat3& M,
        const GEO::vec3& U,
        GEO::vec3& V
        ) {
        V.x = M(0, 0) * U.x + M(1, 0) * U.y + M(2, 0) * U.z;
        V.y = M(0, 1) * U.x + M(1, 1) * U.y + M(2, 1) * U.z;
        V.z = M(0, 2) * U.x + M(1, 2) * U.y + M(2, 2) * U.z;
    }

    /**
     * @brief Computes the component-wise product ``to = p1 * p2``.
     * @param[in] p1 First vector.
     * @param[in] p2 Second vector.
     * @param[out] to Component-wise product of @p p1 and @p p2.
     */
    inline void lp_vecmul(
        const GEO::vec3& p1,
        const GEO::vec3& p2,
        GEO::vec3& to
        ) {
        to.x = p1.x * p2.x;
        to.y = p1.y * p2.y;
        to.z = p1.z * p2.z;
    }

    /**
     * @brief Computes the component-wise product ``to = p1 * p2 * p3``.
     * @param[in] p1 First vector.
     * @param[in] p2 Second vector.
     * @param[in] p3 Third vector.
     * @param[out] to Component-wise product of @p p1, @p p2 and @p p3.
     */
    inline void lp_vecmul(
        const GEO::vec3& p1,
        const GEO::vec3& p2,
        const GEO::vec3& p3,
        GEO::vec3& to
        ) {
        to.x = p1.x * p2.x * p3.x;
        to.y = p1.y * p2.y * p3.y;
        to.z = p1.z * p2.z * p3.z;
    }

    /**
     * @brief Accumulates ``to += s * (p1 * p2 * p3)`` (component-wise).
     * @param[in] s Scalar multiplier.
     * @param[in] p1 First vector.
     * @param[in] p2 Second vector.
     * @param[in] p3 Third vector.
     * @param[in,out] to Accumulator, incremented in place.
     */
    inline void lp_vecmadd(
        const double s,
        const GEO::vec3& p1,
        const GEO::vec3& p2,
        const GEO::vec3& p3,
        GEO::vec3& to
        ) {
        to.x += s * p1.x * p2.x * p3.x;
        to.y += s * p1.y * p2.y * p3.y;
        to.z += s * p1.z * p2.z * p3.z;
    }

    /**
     * @brief Computes ``to = s * p1 + t * p2`` (component-wise).
     * @param[in] s Scalar multiplier of @p p1.
     * @param[in] p1 First vector.
     * @param[in] t Scalar multiplier of @p p2.
     * @param[in] p2 Second vector.
     * @param[out] to The resulting combination.
     */
    inline void lp_vecmadd(
        const double s,
        const GEO::vec3& p1,
        const double t,
        const GEO::vec3& p2,
        GEO::vec3& to
        ) {
        to.x = s * p1.x + t * p2.x;
        to.y = s * p1.y + t * p2.y;
        to.z = s * p1.z + t * p2.z;
    }

    /**
     * @brief Sums the three coordinates of a vector.
     * @details Used to contract a component-wise monomial ``U1^a * U2^b * U3^c``
     *          into the scalar contribution of one exponent triple.
     * @param[in] p The vector to sum.
     * @return ``p.x + p.y + p.z``.
     */
    inline double lp_vecbar(const GEO::vec3& p) {
        return p.x + p.y + p.z;
    }
}

#endif //GEOLIO_LP_MATH_H
