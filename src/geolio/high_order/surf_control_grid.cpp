//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/2.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "surf_control_grid.h"
#include "basis_functions.h"

namespace geolio
{
    template<GEO::index_t DIM>
    GEO::vecng<DIM, double> SurfaceControlGrid<DIM>::compute_facet_uv_position(
        GEO::index_t f,
        const GEO::vec2& uv
        ) const {
        assert(f < this->mesh_.facets.nb());
        assert(uv.x >= 0 && uv.x <= 1);
        assert(uv.y >= 0 && uv.y <= 1);

        GEO::vecng<DIM, double> p;

        std::vector<double> Bu(this->order_+1);
        std::vector<double> Bv(this->order_+1);
        Lagrange_basis_1D(uv.x, this->node_positions_1D_, Bu);
        Lagrange_basis_1D(uv.y, this->node_positions_1D_, Bv);

        for (GEO::index_t i = 0; i <= this->order_; ++i) {
            for (GEO::index_t j = 0; j <= this->order_; ++j) {
                const double lag_basis = Bu[i] * Bv[j];
                p += lag_basis * this->control_node(facet_nd(f, i, j));
            }
        }

        return p;
    }

    template<GEO::index_t DIM>
    [[nodiscard]] GEO::vecng<DIM, double> SurfaceControlGrid<DIM>::compute_facet_uv_position(
        const GEO::index_t f,
        const GEO::vec2& uv,
        const double* cur_control_nodes_ptr
        ) const {
        assert(f < this->mesh_.facets.nb());
        assert(uv.x >= 0 && uv.x <= 1);
        assert(uv.y >= 0 && uv.y <= 1);

        GEO::vecng<DIM, double> p;

        std::vector<double> Bu(this->order_+1);
        std::vector<double> Bv(this->order_+1);
        Lagrange_basis_1D(uv.x, this->node_positions_1D_, Bu);
        Lagrange_basis_1D(uv.y, this->node_positions_1D_, Bv);

        for (GEO::index_t i = 0; i <= this->order_; ++i) {
            for (GEO::index_t j = 0; j <= this->order_; ++j) {
                const auto& fv = facet_nd(f, i, j);
                const double lag_basis = Bu[i] * Bv[j];
                p += lag_basis * GEO::vecng<DIM, double>(cur_control_nodes_ptr);
            }
        }

        return p;
    }

    template<GEO::index_t DIM>
    GEO::vec3 SurfaceControlGrid<DIM>::compute_facet_uv_normal(
        const GEO::index_t f,
        const GEO::vec2& uv
        ) const requires (DIM == 3) {
        assert(f < this->mesh_.facets.nb());
        assert(uv.x >= 0 && uv.x <= 1);
        assert(uv.y >= 0 && uv.y <= 1);

        std::vector<double> Bu(this->order_+1);
        std::vector<double> Bv(this->order_+1);
        std::vector<double> dBu(this->order_+1);
        std::vector<double> dBv(this->order_+1);
        Lagrange_basis_1D(uv.x, this->node_positions_1D_, Bu);
        Lagrange_basis_1D(uv.y, this->node_positions_1D_, Bv);
        Lagrange_basis_deriv_1D(uv.x, this->node_positions_1D_, dBu);
        Lagrange_basis_deriv_1D(uv.y, this->node_positions_1D_, dBv);

        GEO::vec3 Tu(0, 0, 0), Tv(0, 0, 0);
        for (GEO::index_t i = 0; i <= this->order_; ++i) {
            for (GEO::index_t j = 0; j <= this->order_; ++j) {
                const auto& p = this->control_node(this->facet_nd(f, i, j));
                Tu += p * dBu[i] * Bv[j];
                Tv += p * Bu[i] * dBv[j];
            }
        }
        return -GEO::cross(Tu, Tv); /* The orientation of the vertices of the cell facet is towards the interior of the
            cell, so the normal direction needs to be reversed. */
    }

    template<GEO::index_t DIM>
    void SurfaceControlGrid<DIM>::compute_facet_uv_dudv(
        const GEO::index_t f,
        const GEO::vec2& uv,
        GEO::vecng<DIM, double>& du,
        GEO::vecng<DIM, double>& dv,
        std::vector<double>& Bu,
        std::vector<double>& Bv,
        std::vector<double>& dBu,
        std::vector<double>& dBv
        ) const {
        assert(f < this->mesh_.facets.nb());
        assert(uv.x >= 0 && uv.x <= 1);
        assert(uv.y >= 0 && uv.y <= 1);

        du.x = 0; du.y = 0;
        dv.x = 0; dv.y = 0;

        Bu.resize(this->order_+1);
        Bv.resize(this->order_+1);
        dBu.resize(this->order_+1);
        dBv.resize(this->order_+1);
        Lagrange_basis_1D(uv.x, this->node_positions_1D_, Bu);
        Lagrange_basis_1D(uv.y, this->node_positions_1D_, Bv);
        Lagrange_basis_deriv_1D(uv.x, this->node_positions_1D_, dBu);
        Lagrange_basis_deriv_1D(uv.y, this->node_positions_1D_, dBv);

        for (GEO::index_t i = 0; i <= this->order_; ++i) {
            for (GEO::index_t j = 0; j <= this->order_; ++j) {
                const double lag_basis_duv = dBu[i] * Bv[j];
                const double lag_basis_udv = Bu[i] * dBv[j];
                du += lag_basis_duv * this->control_node(this->facet_nd(f, i, j));
                dv += lag_basis_udv * this->control_node(this->facet_nd(f, i, j));
            }
        }
    }

    template<GEO::index_t DIM>
    double SurfaceControlGrid<DIM>::compute_facet_uv_quantity(
        const GEO::index_t f,
        const GEO::vec2& uv,
        const GEO::index_t d
        ) const {
        assert(f < this->mesh_.facets.nb());
        assert(uv.x >= 0 && uv.x <= 1);
        assert(uv.y >= 0 && uv.y <= 1);
        assert(this->control_nodes_quantities_.is_bound());
        const auto dim = this->control_node_quantities_dimension();
        assert(d < dim);

        double q = 0;

        std::vector<double> Bu(this->order_+1);
        std::vector<double> Bv(this->order_+1);
        geolio::Lagrange_basis_1D(uv.x, this->node_positions_1D_, Bu);
        geolio::Lagrange_basis_1D(uv.y, this->node_positions_1D_, Bv);

        for (GEO::index_t i = 0; i <= this->order_; ++i) {
            for (GEO::index_t j = 0; j <= this->order_; ++j) {
                const double lag_basis = Bu[i] * Bv[j];
                q += lag_basis * this->control_nodes_quantities_[dim*this->facet_nd(f, i, j)+d];
            }
        }

        return q;
    }

    template<GEO::index_t DIM>
    void SurfaceControlGrid<DIM>::compute_facet_uv_quantities(
        const GEO::index_t f,
        const GEO::vec2& uv,
        double* q
        ) const {
        assert(f < this->mesh_.facets.nb());
        assert(uv.x >= 0 && uv.x <= 1);
        assert(uv.y >= 0 && uv.y <= 1);
        assert(this->control_nodes_quantities_.is_bound());
        const auto dim = this->control_node_quantities_dimension();

        std::fill_n(q, dim, 0.0);

        std::vector<double> Bu(this->order_+1);
        std::vector<double> Bv(this->order_+1);
        geolio::Lagrange_basis_1D(uv.x, this->node_positions_1D_, Bu);
        geolio::Lagrange_basis_1D(uv.y, this->node_positions_1D_, Bv);

        for (GEO::index_t i = 0; i <= this->order_; ++i) {
            for (GEO::index_t j = 0; j <= this->order_; ++j) {
                const double lag_basis = Bu[i] * Bv[j];
                for (GEO::index_t d = 0; d < dim; ++d)
                    q[d] += lag_basis * this->control_nodes_quantities_[dim*this->facet_nd(f, i, j)+d];
            }
        }
    }

    template class SurfaceControlGrid<2>;
    template class SurfaceControlGrid<3>;
}