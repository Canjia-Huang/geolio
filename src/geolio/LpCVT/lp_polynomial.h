//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/3.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_LP_POLYNOMIAL_H
#define GEOLIO_LP_POLYNOMIAL_H

#include "lp_math.h"

namespace geolio
{
    /**
     * @brief Computes the Lp integrand ``E`` of one integration simplex and its
     *        gradients with respect to the four simplex vertices.
     * @details This is a direct port of ``IntegrationSimplex<P, MEASURE>`` from
     *          ``LpCVT/algebra/integration_simplex.h``. For an integration
     *          simplex ``(p0, p1, p2, p3)`` and the transformed edge vectors
     *          ``U_k = M (p_k - p0)`` it evaluates
     *          @f[
     *            E = \sum_{\alpha + \beta + \gamma = P} \sum_{c \in \{x,y,z\}}
     *                (U_1)_c^{\alpha} (U_2)_c^{\beta} (U_3)_c^{\gamma}
     *          @f]
     *          i.e. the complete homogeneous symmetric polynomial of degree @p P
     *          applied coordinate-wise, and returns the objective
     *          ``F = |T| * E`` where the measure @p T comes from @p MEASURE
     *          (@ref LpTriArea or @ref LpTetVolume).
     *
     *          Rem: the binomial coefficient ``(n + p \choose n)`` is ignored.
     *          See Appendices A and B.1 in the paper.
     * @tparam P The integer norm exponent. Must be even and in ``[2, 16]``;
     *         the original implementation asserts ``P >= 2 && P <= 16``.
     * @note @b Reentrancy. The exponent tables are immutable after construction,
     *       and the per-evaluation powers of ``U_k`` are kept in a local array
     *       (rather than as a member, as the reference implementation did).
     *       This is required because ``GEO::RestrictedVoronoiDiagram`` shares a
     *       single ``IntegrationSimplex`` instance between all of its worker
     *       threads, so eval() must be reentrant.
     */
    template <unsigned int P>
    class LpPolynomial {
    public:
        /** @brief The degree of the polynomial, i.e. the norm exponent @p P. */
        static constexpr unsigned int degree = P;

        /**
         * @brief Number of exponent triples ``(alpha, beta, gamma)`` with
         *        ``alpha + beta + gamma == P``.
         * @details Equals ``binomial(P + 2, 2)``, which is also the binomial
         *          coefficient that the original implementation omits from the
         *          energy; see lp_surface_energy_normalization().
         */
        static constexpr unsigned int nb_coeffs = ((P + 1) * (P + 2)) / 2;

        /**
         * @brief Number of exponent triples that contribute to the derivative,
         *        i.e. those whose differentiated exponent is non-zero.
         * @details Equals ``nb_coeffs - (P + 1)``.
         */
        static constexpr unsigned int nb_dcoeffs = nb_coeffs - (P + 1);

        /**
         * @brief Constructs the polynomial and precomputes its exponent tables.
         * @details Precomputes the exponent triples used by the value and by the
         *          gradient, so that eval() only performs multiplications.
         */
        LpPolynomial() {
            // Pre-compute indices for evaluating F_{L_p}^T
            // (see Appendix A)
            {
                unsigned int cur = 0;
                for (unsigned int alpha = 0; alpha <= P; alpha++) {
                    for (unsigned int beta = 0; beta <= P - alpha; beta++) {
                        const unsigned int gamma = P - alpha - beta;
                        E_pow_[cur][0] = alpha;
                        E_pow_[cur][1] = beta;
                        E_pow_[cur][2] = gamma;
                        cur++;
                    }
                }
            }
            // Pre-compute indices for evaluating \nabla F_{L_p}^T
            // (see Appendix B.1)
            {
                unsigned int cur_dU1 = 0, cur_dU2 = 0, cur_dU3 = 0;
                for (unsigned int alpha = 0; alpha <= P; alpha++) {
                    for (unsigned int beta = 0; beta <= P - alpha; beta++) {
                        const unsigned int gamma = P - alpha - beta;
                        if (alpha != 0) {
                            dE_pow_[0][cur_dU1][0] = alpha;
                            dE_pow_[0][cur_dU1][1] = beta;
                            dE_pow_[0][cur_dU1][2] = gamma;
                            cur_dU1++;
                        }
                        if (beta != 0) {
                            dE_pow_[1][cur_dU2][0] = alpha;
                            dE_pow_[1][cur_dU2][1] = beta;
                            dE_pow_[1][cur_dU2][2] = gamma;
                            cur_dU2++;
                        }
                        if (gamma != 0) {
                            dE_pow_[2][cur_dU3][0] = alpha;
                            dE_pow_[2][cur_dU3][1] = beta;
                            dE_pow_[2][cur_dU3][2] = gamma;
                            cur_dU3++;
                        }
                    }
                }
            }
        }

        /**
         * @brief Evaluates the objective and its gradients on one integration
         *        simplex.
         * @details Computes ``F = T * E`` and the four gradients ``dF/dp_k``,
         *          using the chain rule through ``U_k = M (p_k - p0)``. Since
         *          ``grad(F(MX)) = (grad F)^T M``, the gradients with respect to
         *          the original vertices are obtained by applying ``M^T``, and
         *          ``dF/dp0`` follows from the fact that ``E`` and ``T`` only
         *          depend on ``p0`` through the differences ``p_k - p0``.
         * @tparam MEASURE Either @ref LpTriArea or @ref LpTetVolume.
         * @param[in] measure The measure functor used for @p T.
         * @param[in] p0 First vertex of the integration simplex. This is the
         *               Delaunay seed / Voronoi cell center in the LpCVT setting.
         * @param[in] p1 Second vertex of the integration simplex.
         * @param[in] p2 Third vertex of the integration simplex.
         * @param[in] p3 Fourth vertex of the integration simplex.
         * @param[in] M Anisotropy matrix. Use the identity for the isotropic
         *              LpCVT of the original implementation.
         * @param[out] dFdp0 Gradient of the objective with respect to @p p0.
         * @param[out] dFdp1 Gradient of the objective with respect to @p p1.
         * @param[out] dFdp2 Gradient of the objective with respect to @p p2.
         * @param[out] dFdp3 Gradient of the objective with respect to @p p3.
         * @return The objective ``T * E`` for this integration simplex.
         */
        template <class MEASURE>
        double eval(
            const MEASURE& measure,
            const GEO::vec3& p0,
            const GEO::vec3& p1,
            const GEO::vec3& p2,
            const GEO::vec3& p3,
            const GEO::mat3& M,
            GEO::vec3& dFdp0,
            GEO::vec3& dFdp1,
            GEO::vec3& dFdp2,
            GEO::vec3& dFdp3
            ) const {
            // U_pow[k][n] is the n-th component-wise power of U_k = M (p_k - p0).
            // It is intentionally a local (and not a member) so that eval() stays
            // reentrant under Geogram's multi-threaded RVD traversal.
            GEO::vec3 U_pow[3][P + 1];
            U_pow[0][0] = GEO::vec3(1.0, 1.0, 1.0);
            U_pow[1][0] = GEO::vec3(1.0, 1.0, 1.0);
            U_pow[2][0] = GEO::vec3(1.0, 1.0, 1.0);

            {
                lp_matvecmul(M, p1 - p0, U_pow[0][1]);
                lp_matvecmul(M, p2 - p0, U_pow[1][1]);
                lp_matvecmul(M, p3 - p0, U_pow[2][1]);
            }

            for (unsigned int i = 2; i <= P; ++i) {
                lp_vecmul(U_pow[0][1], U_pow[0][i - 1], U_pow[0][i]);
                lp_vecmul(U_pow[1][1], U_pow[1][i - 1], U_pow[1][i]);
                lp_vecmul(U_pow[2][1], U_pow[2][i - 1], U_pow[2][i]);
            }

            // Computation of function value.
            double E = 0.0;
            for (unsigned int i = 0; i < nb_coeffs; ++i) {
                GEO::vec3 W;
                const unsigned int alpha = E_pow_[i][0];
                const unsigned int beta = E_pow_[i][1];
                const unsigned int gamma = E_pow_[i][2];
                lp_vecmul(U_pow[0][alpha], U_pow[1][beta], U_pow[2][gamma], W);
                E += lp_vecbar(W);
            }

            // Computation of gradient
            GEO::vec3 dEdU1(0, 0, 0), dEdU2(0, 0, 0), dEdU3(0, 0, 0);
            for (unsigned int i = 0; i < nb_dcoeffs; ++i) {
                {
                    const unsigned int alpha = dE_pow_[0][i][0];
                    const unsigned int beta = dE_pow_[0][i][1];
                    const unsigned int gamma = dE_pow_[0][i][2];
                    lp_vecmadd(alpha, U_pow[0][alpha - 1], U_pow[1][beta], U_pow[2][gamma], dEdU1);
                }
                {
                    const unsigned int alpha = dE_pow_[1][i][0];
                    const unsigned int beta = dE_pow_[1][i][1];
                    const unsigned int gamma = dE_pow_[1][i][2];
                    lp_vecmadd(beta, U_pow[0][alpha], U_pow[1][beta - 1], U_pow[2][gamma], dEdU2);
                }
                {
                    const unsigned int alpha = dE_pow_[2][i][0];
                    const unsigned int beta = dE_pow_[2][i][1];
                    const unsigned int gamma = dE_pow_[2][i][2];
                    lp_vecmadd(gamma, U_pow[0][alpha], U_pow[1][beta], U_pow[2][gamma - 1], dEdU3);
                }
            }

            // Compute the measure and its
            // derivatives relative to U1, U2 and U3.
            GEO::vec3 dTdU1, dTdU2, dTdU3;
            const double T = measure(
                U_pow[0][1], U_pow[1][1], U_pow[2][1],
                dTdU1, dTdU2, dTdU3
                );

            // Assemble dF = E.d|T| + |T|.dE
            // Rem: anisotropy matrix needs to be transposed
            // grad(F(MX)) = J(F(MX))^T = (gradF^T M)^T = M^T grad F
            GEO::vec3 dFdU1, dFdU2, dFdU3;
            lp_vecmadd(E, dTdU1, T, dEdU1, dFdU1);
            lp_matTvecmul(M, dFdU1, dFdp1);
            lp_vecmadd(E, dTdU2, T, dEdU2, dFdU2);
            lp_matTvecmul(M, dFdU2, dFdp2);
            lp_vecmadd(E, dTdU3, T, dEdU3, dFdU3);
            lp_matTvecmul(M, dFdU3, dFdp3);

            // Gradient relative to p1 (resp. p2,p3) = gradient relative to U1 (resp U2,U3)
            // Gradient relative to p0 is equal to minus gradient relative to U1,U2 and U3
            dFdp0.x = -dFdp1.x - dFdp2.x - dFdp3.x;
            dFdp0.y = -dFdp1.y - dFdp2.y - dFdp3.y;
            dFdp0.z = -dFdp1.z - dFdp2.z - dFdp3.z;

            return T * E;
        }

    private:
        /** @brief Exponent triples of the monomials summed into ``E``. */
        unsigned int E_pow_[nb_coeffs][3];
        /** @brief Per-variable exponent triples contributing to ``dE/dU_k``. */
        unsigned int dE_pow_[3][nb_dcoeffs][3];
    };
}

#endif //GEOLIO_LP_POLYNOMIAL_H
