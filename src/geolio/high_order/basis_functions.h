//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/2.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_BASIS_FUNCTIONS_H
#define GEOLIO_BASIS_FUNCTIONS_H
#include <cassert>
#include <geogram/basic/numeric.h>

namespace geolio
{
	/**
	 * Compute the values of 1D Bernstein basis polynomials at a given parameter.
	 *
	 * @param[in] t Evaluation parameter in the reference interval [0, 1].
	 * @param[in] order Polynomial order.
	 * @param[out] basis Output basis-value array resized to `order + 1`.
	 * @post `basis[i]` stores the value of the i-th Bernstein basis function at @p t.
	 */
	inline void Bezier_basis_1D(
		const double t,
		const GEO::index_t order,
		std::vector<double>& basis
		) {
		assert(t >= 0 && t <= 1);

		basis.assign(order+1, 0.0);

		basis[0] = 1.0;
		for (GEO::index_t k = 1; k <= order; ++k) {
			double prev = basis[0];
			basis[0] = (1.0 - t) * prev;

			for (int i = 1; i < k; ++i) {
				const double current = basis[i];
				basis[i] = (1.0 - t) * current + t * prev;
				prev = current;
			}

			basis[k] = t * prev;
		}
	}

	/**
	 * Compute one 1D Lagrange basis function value at a given point.
	 *
	 * @param[in] x Evaluation coordinate.
	 * @param[in] i Basis-function index in `[0, node_positions.size() - 1]`.
	 * @param[in] node_positions Interpolation node coordinates; all entries must be distinct.
	 * @return Value of the i-th Lagrange basis function at @p x.
	 */
	inline double Lagrange_val_1D(
		const double x,
		const GEO::index_t i,
		const std::vector<double>& node_positions
		) {
		assert(i < node_positions.size());
		double res = 1.0;
		for (GEO::index_t k = 0, i_end = node_positions.size(); k < i_end; ++k) {
			if (k != i)
				res *= (x - node_positions[k]) / (node_positions[i] - node_positions[k]);
		}
		return res;
	}

	/**
	 * Compute all 1D Lagrange basis function values at a given point.
	 *
	 * @param[in] x Evaluation coordinate.
	 * @param[in] node_positions Interpolation node coordinates (size should be `order + 1`).
	 * @param[out] basis Output basis-value array resized to `node_positions.size()`.
	 * @post `basis[i]` stores the value of the i-th Lagrange basis function at @p x.
	 */
	inline void Lagrange_basis_1D(
		const double x,
		const std::vector<double>& node_positions,
		std::vector<double>& basis
		) {
		basis.assign(node_positions.size(), 1.0);

		for (GEO::index_t i = 0, i_end = node_positions.size(); i < i_end; ++i)
			basis[i] = Lagrange_val_1D(x, i, node_positions);
	}

	/**
	 * Compute one 1D Lagrange basis function derivative at a given point.
	 *
	 * @param[in] x Evaluation coordinate.
	 * @param[in] i Basis-function index in `[0, node_positions.size() - 1]`.
	 * @param[in] node_positions Interpolation node coordinates; all entries must be distinct.
	 * @return Derivative value of the i-th Lagrange basis function at @p x.
	 */
	inline double Lagrange_deriv_1D(
		const double x,
		const GEO::index_t i,
		const std::vector<double>& node_positions
		) {
		assert(i < node_positions.size());
		const GEO::index_t n = node_positions.size();
		double sum_deriv = 0.0;

		for (GEO::index_t k = 0; k < n; ++k) {
			if (k == i)
				continue;

			double term = 1.0 / (node_positions[i] - node_positions[k]);
			for (GEO::index_t m = 0; m < n; ++m) {
				if (m == i || m == k)
					continue;
				term *= (x - node_positions[m]) / (node_positions[i] - node_positions[m]);
			}
			sum_deriv += term;
		}
		return sum_deriv;
	}

	/**
	 * Compute all 1D Lagrange basis function derivatives at a given point.
	 *
	 * @param[in] x Evaluation coordinate.
	 * @param[in] node_positions Interpolation node coordinates; all entries must be distinct.
	 * @param[out] deriv_basis Output derivative array resized to `node_positions.size()`.
	 * @post `deriv_basis[i]` stores the derivative of the i-th Lagrange basis function at @p x.
	 */
	inline void Lagrange_basis_deriv_1D(
		const double x,
		const std::vector<double>& node_positions,
		std::vector<double>& deriv_basis
		) {
		deriv_basis.assign(node_positions.size(), 0.0);

		for (GEO::index_t i = 0, i_end = node_positions.size(); i < i_end; ++i)
			deriv_basis[i] = Lagrange_deriv_1D(x, i, node_positions);
	}
}

#endif //GEOLIO_BASIS_FUNCTIONS_H
