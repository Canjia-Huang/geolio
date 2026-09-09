//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/2.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "volume_control_grid.h"
#include "basis_functions.h"

namespace geolio
{
    GEO::vec3 VolumeControlGrid::compute_cell_uvw_position(
        const GEO::index_t c,
        const GEO::vec3& uvw
        ) const {
        assert(c < mesh_.cells.nb());
        assert(uvw.x >= 0 && uvw.x <= 1);
        assert(uvw.y >= 0 && uvw.y <= 1);
        assert(uvw.z >= 0 && uvw.z <= 1);

        GEO::vec3 p(0, 0, 0);

        std::vector<double> Bu(order_+1);
        std::vector<double> Bv(order_+1);
        std::vector<double> Bw(order_+1);
        Lagrange_basis_1D(uvw.x, node_positions_1D_, Bu);
        Lagrange_basis_1D(uvw.y, node_positions_1D_, Bv);
        Lagrange_basis_1D(uvw.z, node_positions_1D_, Bw);

        for (GEO::index_t i = 0; i <= order_; ++i) {
            for (GEO::index_t j = 0; j <= order_; ++j) {
                const double basis_uv = Bu[i] * Bv[j];
                for (GEO::index_t k = 0; k <= order_; ++k) {
                    const double lag_basis = basis_uv * Bw[k];
                    p += lag_basis * control_node(cell_nd(c, i, j, k));
                }
            }
        }

        return p;
    }

    [[nodiscard]] GEO::vec3 VolumeControlGrid::compute_cell_uvw_position(
        const GEO::index_t c,
        const GEO::vec3& uvw,
        const double* cur_control_nodes_ptr
        ) const {
        assert(c < mesh_.cells.nb());
        assert(uvw.x >= 0 && uvw.x <= 1);
        assert(uvw.y >= 0 && uvw.y <= 1);
        assert(uvw.z >= 0 && uvw.z <= 1);

        GEO::vec3 p(0, 0, 0);

        std::vector<double> Bu(order_+1);
        std::vector<double> Bv(order_+1);
        std::vector<double> Bw(order_+1);
        Lagrange_basis_1D(uvw.x, node_positions_1D_, Bu);
        Lagrange_basis_1D(uvw.y, node_positions_1D_, Bv);
        Lagrange_basis_1D(uvw.z, node_positions_1D_, Bw);

        for (GEO::index_t i = 0; i <= order_; ++i) {
            for (GEO::index_t j = 0; j <= order_; ++j) {
                const double basis_uv = Bu[i] * Bv[j];
                for (GEO::index_t k = 0; k <= order_; ++k) {
                    const auto& cv = cell_nd(c, i, j, k);
                    const double lag_basis = basis_uv * Bw[k];
                    p += lag_basis * GEO::vec3(
                        cur_control_nodes_ptr[3*cv],
                        cur_control_nodes_ptr[3*cv+1],
                        cur_control_nodes_ptr[3*cv+2]);
                }
            }
        }

        return p;
    }

    GEO::vec3 VolumeControlGrid::compute_cell_facet_uv_normal(
        const GEO::index_t c,
        const GEO::index_t lf,
        const GEO::vec2& uv
        ) const {
        assert(c < mesh_.cells.nb());
        assert(lf < mesh_.cells.nb_facets(c));
        assert(uv.x >= 0 && uv.x <= 1);
        assert(uv.y >= 0 && uv.y <= 1);

        std::vector<double> Bu(order_+1);
        std::vector<double> Bv(order_+1);
        std::vector<double> dBu(order_+1);
        std::vector<double> dBv(order_+1);
        Lagrange_basis_1D(uv.x, node_positions_1D_, Bu);
        Lagrange_basis_1D(uv.y, node_positions_1D_, Bv);
        Lagrange_basis_deriv_1D(uv.x, node_positions_1D_, dBu);
        Lagrange_basis_deriv_1D(uv.y, node_positions_1D_, dBv);

        GEO::vec3 Tu(0, 0, 0), Tv(0, 0, 0);
        for (GEO::index_t i = 0; i <= order_; ++i) {
            for (GEO::index_t j = 0; j <= order_; ++j) {
                const auto& p = control_node(cell_facet_nd(c, lf, i, j));
                Tu += p * dBu[i] * Bv[j];
                Tv += p * Bu[i] * dBv[j];
            }
        }
        return -GEO::cross(Tu, Tv); /* The orientation of the vertices of the cell facet is towards the interior of the
            cell, so the normal direction needs to be reversed. */
    }

    void VolumeControlGrid::compute_cell_uvw_dudvdw(
        const GEO::index_t c,
        const GEO::vec3& uvw,
        GEO::vec3& du,
        GEO::vec3& dv,
        GEO::vec3& dw,
        std::vector<double>& Bu,
        std::vector<double>& Bv,
        std::vector<double>& Bw,
        std::vector<double>& dBu,
        std::vector<double>& dBv,
        std::vector<double>& dBw
        ) const {
        assert(c < mesh_.cells.nb());
        assert(uvw.x >= 0 && uvw.x <= 1);
        assert(uvw.y >= 0 && uvw.y <= 1);
        assert(uvw.z >= 0 && uvw.z <= 1);

        du.x = 0; du.y = 0; du.z = 0;
        dv.x = 0; dv.y = 0; dv.z = 0;
        dw.x = 0; dw.y = 0; dw.z = 0;

        Bu.resize(order_+1);
        Bv.resize(order_+1);
        Bw.resize(order_+1);
        dBu.resize(order_+1);
        dBv.resize(order_+1);
        dBw.resize(order_+1);
        Lagrange_basis_1D(uvw.x, node_positions_1D_, Bu);
        Lagrange_basis_1D(uvw.y, node_positions_1D_, Bv);
        Lagrange_basis_1D(uvw.z, node_positions_1D_, Bw);
        Lagrange_basis_deriv_1D(uvw.x, node_positions_1D_, dBu);
        Lagrange_basis_deriv_1D(uvw.y, node_positions_1D_, dBv);
        Lagrange_basis_deriv_1D(uvw.z, node_positions_1D_, dBw);

        for (GEO::index_t k = 0; k <= order_; ++k) {
            for (GEO::index_t j = 0; j <= order_; ++j) {
                const double basis_vw = Bv[j] * Bw[k];
                const double basis_dvw= dBv[j] * Bw[k];
                const double basis_vdw= Bv[j] * dBw[k];
                for (GEO::index_t i = 0; i <= order_; ++i) {
                    const double lag_basis_duvw = dBu[i] * basis_vw;
                    const double lag_basis_udvw = Bu[i] * basis_dvw;
                    const double lag_basis_uvdw = Bu[i] * basis_vdw;
                    du += lag_basis_duvw * control_node(cell_nd(c, i, j, k));
                    dv += lag_basis_udvw * control_node(cell_nd(c, i, j, k));
                    dw += lag_basis_uvdw * control_node(cell_nd(c, i, j, k));
                }
            }
        }
    }

    double VolumeControlGrid::compute_cell_uvw_quantity(
        const GEO::index_t c,
        const GEO::vec3& uvw,
        const GEO::index_t d
        ) const {
        assert(c < mesh_.cells.nb());
        assert(uvw.x >= 0 && uvw.x <= 1);
        assert(uvw.y >= 0 && uvw.y <= 1);
        assert(uvw.z >= 0 && uvw.z <= 1);
        assert(control_nodes_quantities_.is_bound());
        const auto dim = control_node_quantities_dimension();
        assert(d < dim);

        double q = 0;

        std::vector<double> Bu(order_+1);
        std::vector<double> Bv(order_+1);
        std::vector<double> Bw(order_+1);
        Lagrange_basis_1D(uvw.x, node_positions_1D_, Bu);
        Lagrange_basis_1D(uvw.y, node_positions_1D_, Bv);
        Lagrange_basis_1D(uvw.z, node_positions_1D_, Bw);

        for (GEO::index_t i = 0; i <= order_; ++i) {
            for (GEO::index_t j = 0; j <= order_; ++j) {
                const double basis_uv = Bu[i] * Bv[j];
                for (GEO::index_t k = 0; k <= order_; ++k) {
                    const double lag_basis = basis_uv * Bw[k];
                    q += lag_basis * control_nodes_quantities_[dim*cell_nd(c, i, j, k)+d];
                }
            }
        }

        return q;
    }

    void VolumeControlGrid::compute_cell_uvw_quantities(
        const GEO::index_t c,
        const GEO::vec3& uvw,
        double* q
        ) const {
        assert(c < mesh_.cells.nb());
        assert(uvw.x >= 0 && uvw.x <= 1);
        assert(uvw.y >= 0 && uvw.y <= 1);
        assert(uvw.z >= 0 && uvw.z <= 1);
        assert(control_nodes_quantities_.is_bound());
        const auto dim = control_node_quantities_dimension();

        std::fill_n(q, dim, 0.0);

        std::vector<double> Bu(order_+1);
        std::vector<double> Bv(order_+1);
        std::vector<double> Bw(order_+1);
        Lagrange_basis_1D(uvw.x, node_positions_1D_, Bu);
        Lagrange_basis_1D(uvw.y, node_positions_1D_, Bv);
        Lagrange_basis_1D(uvw.z, node_positions_1D_, Bw);

        for (GEO::index_t i = 0; i <= order_; ++i) {
            for (GEO::index_t j = 0; j <= order_; ++j) {
                const double basis_uv = Bu[i] * Bv[j];
                for (GEO::index_t k = 0; k <= order_; ++k) {
                    const double lag_basis = basis_uv * Bw[k];
                    for (GEO::index_t d = 0; d < dim; ++d)
                        q[d] += lag_basis * control_nodes_quantities_[dim*cell_nd(c, i, j, k)+d];
                }
            }
        }
    }
}