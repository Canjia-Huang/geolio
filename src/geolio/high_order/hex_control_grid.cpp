//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/2.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "hex_control_grid.h"
#include <cassert>
#include <geolio/common/Gauss_Legendre_quadrature_cube.h>
#include <geolio/common/log.h>
#include <geolio/common/pair_hash.h>
#include <geolio/mesh/hex_operations.h>
#include "basis_functions.h"

namespace geolio
{
    HexControlGrid::HexControlGrid(
        const GEO::Mesh& mesh,
        const GEO::index_t order
        ) : VolumeControlGrid(mesh, order)
    {
        assert(std::all_of(
            mesh.cells.cell_type_ptr(0),
            mesh.cells.cell_type_ptr(0)+mesh.cells.nb(),
            [&](const auto cell_type) { return cell_type == GEO::MESH_HEX; })); // check all-hex mesh

        HexControlGrid::initialize_nodes_arrangement();
        HexControlGrid::initialize_control_nodes();
    }

    void HexControlGrid::compute_cell_uvw_Jacobian(
        const GEO::index_t c,
        const GEO::vec3& uvw,
        Eigen::Matrix3d& J
        ) const {
        assert(c < mesh_.cells.nb());
        assert(uvw.x >= 0 && uvw.x <= 1);
        assert(uvw.y >= 0 && uvw.y <= 1);
        assert(uvw.z >= 0 && uvw.z <= 1);

        GEO::vec3 du, dv, dw;
        std::vector<double> Bu, Bv, Bw, dBu, dBv, dBw;
        compute_cell_uvw_dudvdw(c, uvw, du, dv, dw, Bu, Bv, Bw, dBu, dBv, dBw);

        J(0, 0) = du.x;
        J(1, 0) = du.y;
        J(2, 0) = du.z;
        J(0, 1) = dv.x;
        J(1, 1) = dv.y;
        J(2, 1) = dv.z;
        J(0, 2) = dw.x;
        J(1, 2) = dw.y;
        J(2, 2) = dw.z;
    }

    double HexControlGrid::compute_cell_uvw_measure(
        const GEO::index_t c,
        const GEO::vec3& uvw,
        const MeasureType quality_type
        ) const {
        assert(c < mesh_.cells.nb());
        assert(uvw.x >= 0 && uvw.x <= 1);
        assert(uvw.y >= 0 && uvw.y <= 1);
        assert(uvw.z >= 0 && uvw.z <= 1);

        GEO::vec3 du, dv, dw;
        std::vector<double> Bu, Bv, Bw, dBu, dBv, dBw;
        compute_cell_uvw_dudvdw(c, uvw, du, dv, dw, Bu, Bv, Bw, dBu, dBv, dBw);

        const double det_J = GEO::dot(dw,GEO::cross(du,dv));
        switch (quality_type) {
            case MeasureType::DET_JACOBIAN: {
                return det_J;
            }
            case MeasureType::MIPS: {
                const double F_sq_norm = du.length2()+dv.length2()+dw.length2();
                return F_sq_norm / (3.0*std::cbrt(det_J*det_J));
            }
            case MeasureType::SCALED_JACOBIAN: {
                return det_J/(du.length()*dv.length()*dw.length());
            }
            case MeasureType::INVERSE_MEAN_RATIO: {
                const double F_sq_norm = du.length2()+dv.length2()+dw.length2();
                return 3.0*std::cbrt(det_J*det_J)/F_sq_norm;
            }
            default: assert(0);
        }
        return 0;
    }

    void HexControlGrid::compute_cell_uvw_detJ_gradient(
        const GEO::index_t c,
        const GEO::vec3& uvw,
        std::vector<double>& gradient
        ) const {
        assert(c < mesh_.cells.nb());
        assert(uvw.x >= 0 && uvw.x <= 1);
        assert(uvw.y >= 0 && uvw.y <= 1);
        assert(uvw.z >= 0 && uvw.z <= 1);

        GEO::vec3 du, dv, dw;
        std::vector<double> Bu, Bv, Bw, dBu, dBv, dBw;
        compute_cell_uvw_dudvdw(c, uvw, du, dv, dw, Bu, Bv, Bw, dBu, dBv, dBw);

        const GEO::vec3 cross_dvdw = GEO::cross(dv, dw);
        const GEO::vec3 cross_dwdu = GEO::cross(dw, du);
        const GEO::vec3 cross_dudv = GEO::cross(du, dv);

        gradient.resize(3*CONTROL_POINTS_NB_PER_CELL_);

        for (GEO::index_t k = 0; k < CONTROL_POINTS_NB_PER_EDGE_; ++k) {
            for (GEO::index_t j = 0; j < CONTROL_POINTS_NB_PER_EDGE_; ++j) {
                const double basis_vw = Bv[j] * Bw[k];
                const double basis_dvw= dBv[j] * Bw[k];
                const double basis_vdw= Bv[j] * dBw[k];
                for (GEO::index_t i = 0; i < CONTROL_POINTS_NB_PER_EDGE_; ++i) {
                    const double lag_basis_duvw = dBu[i] * basis_vw;
                    const double lag_basis_udvw = Bu[i] * basis_dvw;
                    const double lag_basis_uvdw = Bu[i] * basis_vdw;
                    const auto& lcv = cell_lnd(i, j, k);
                    const auto& g = lag_basis_duvw*cross_dvdw + lag_basis_udvw*cross_dwdu + lag_basis_uvdw*cross_dudv;
                    gradient[3*lcv] = g.x;
                    gradient[3*lcv+1] = g.y;
                    gradient[3*lcv+2] = g.z;
                }
            }
        }
    }

    void HexControlGrid::compute_cells_volume(
        std::vector<double>& volumes
        ) const {
        volumes.resize(mesh_.cells.nb());

        /* 2k-1 >= 3*order-1  ->  k >= 1.5*order */
        std::vector<std::pair<GEO::vec3, double>> points_and_weights;
        geolio::get_Gauss_Legendre_quadrature_cube(std::ceil(1.5*order_), points_and_weights);

        for (const auto& c : mesh_.cells) {
            double V = 0;
            for (const auto& [uvw, w] : points_and_weights)
                V += w * compute_cell_uvw_measure(c, uvw, HexControlGrid::MeasureType::DET_JACOBIAN);
            volumes[c] = V;
        }
    }

    void HexControlGrid::compute_cell_vertices_position_matrix(
        const GEO::index_t c,
        Eigen::MatrixXd& P
        ) {
        assert(c < mesh_.cells.nb());
        assert(P.rows() == 3);
        assert(P.cols() == CONTROL_POINTS_NB_PER_CELL_);

        for (GEO::index_t i = 0; i < CONTROL_POINTS_NB_PER_CELL_; ++i) {
            const auto& cv = element_control_nodes_[CONTROL_POINTS_NB_PER_CELL_*c+i];
            const auto& cp = control_node(cv);
            P(0, i) = cp.x;
            P(1, i) = cp.y;
            P(2, i) = cp.z;
        }
    }

    void HexControlGrid::compute_basis_gradient_matrix(
        const GEO::vec3& uvw,
        Eigen::MatrixXd& Bg
        ) const {
        assert(uvw.x >= 0 && uvw.x <= 1);
        assert(uvw.y >= 0 && uvw.y <= 1);
        assert(uvw.z >= 0 && uvw.z <= 1);
        assert(Bg.rows() == CONTROL_POINTS_NB_PER_CELL_);
        assert(Bg.cols() == 3);

        std::vector<double> Bu(order_+1);
        std::vector<double> Bv(order_+1);
        std::vector<double> Bw(order_+1);
        std::vector<double> dBu(order_+1);
        std::vector<double> dBv(order_+1);
        std::vector<double> dBw(order_+1);
        Lagrange_basis_1D(uvw.x, node_positions_1D_, Bu);
        Lagrange_basis_1D(uvw.y, node_positions_1D_, Bv);
        Lagrange_basis_1D(uvw.z, node_positions_1D_, Bw);
        Lagrange_basis_deriv_1D(uvw.x, node_positions_1D_, dBu);
        Lagrange_basis_deriv_1D(uvw.y, node_positions_1D_, dBv);
        Lagrange_basis_deriv_1D(uvw.z, node_positions_1D_, dBw);
        for (GEO::index_t i = 0; i < CONTROL_POINTS_NB_PER_EDGE_; ++i) {
            for (GEO::index_t j = 0; j < CONTROL_POINTS_NB_PER_EDGE_; ++j) {
                const auto dBu_Bv = dBu[i]*Bv[j];
                const auto Bu_dBv = Bu[i]*dBv[j];
                const auto Bu_Bv = Bu[i]*Bv[j];
                for (GEO::index_t k = 0; k < CONTROL_POINTS_NB_PER_EDGE_; ++k) {
                    const auto N = cell_lnd(i, j, k);
                    Bg(N, 0) = dBu_Bv*Bw[k];
                    Bg(N, 1) = Bu_dBv*Bw[k];
                    Bg(N, 2) = Bu_Bv*dBw[k];
                }
            }
        }
    }

    void HexControlGrid::initialize_nodes_arrangement(
        ) {
        // LOG::TRACE(__FUNCTION__);

        const GEO::index_t LAST_LAYER_BEGIN_IDX = (CONTROL_POINTS_NB_PER_EDGE_-1)*CONTROL_POINTS_NB_PER_FACET_;

        /* == Vertex =============================================================================================== */
        ELEMENT_VERTEX_CONTROL_POINTS_BEGIN_IDX_ = {
            0,
            CONTROL_POINTS_NB_PER_EDGE_-1,
            (CONTROL_POINTS_NB_PER_EDGE_-1)*CONTROL_POINTS_NB_PER_EDGE_,
            CONTROL_POINTS_NB_PER_FACET_-1,
            LAST_LAYER_BEGIN_IDX,
            LAST_LAYER_BEGIN_IDX+CONTROL_POINTS_NB_PER_EDGE_-1,
            LAST_LAYER_BEGIN_IDX+(CONTROL_POINTS_NB_PER_EDGE_-1)*CONTROL_POINTS_NB_PER_EDGE_,
            LAST_LAYER_BEGIN_IDX+CONTROL_POINTS_NB_PER_FACET_-1
        };

        /* == Edge ================================================================================================= */
        ELEMENT_EDGE_CONTROL_POINTS_BEGIN_IDX_ = {
            0,
            CONTROL_POINTS_NB_PER_EDGE_-1,
            CONTROL_POINTS_NB_PER_FACET_-1,
            (CONTROL_POINTS_NB_PER_EDGE_-1)*CONTROL_POINTS_NB_PER_EDGE_,
            LAST_LAYER_BEGIN_IDX,
            LAST_LAYER_BEGIN_IDX+CONTROL_POINTS_NB_PER_EDGE_-1,
            LAST_LAYER_BEGIN_IDX+CONTROL_POINTS_NB_PER_FACET_-1,
            LAST_LAYER_BEGIN_IDX+(CONTROL_POINTS_NB_PER_EDGE_-1)*CONTROL_POINTS_NB_PER_EDGE_,
            0,
            CONTROL_POINTS_NB_PER_EDGE_-1,
            CONTROL_POINTS_NB_PER_FACET_-1,
            (CONTROL_POINTS_NB_PER_EDGE_-1)*CONTROL_POINTS_NB_PER_EDGE_
        };
        ELEMENT_EDGE_CONTROL_POINTS_NEXT_IDX_STEP_ = {
            1,
            static_cast<int>(CONTROL_POINTS_NB_PER_EDGE_),
            -1,
            -static_cast<int>(CONTROL_POINTS_NB_PER_EDGE_),
            1,
            static_cast<int>(CONTROL_POINTS_NB_PER_EDGE_),
            -1,
            -static_cast<int>(CONTROL_POINTS_NB_PER_EDGE_),
            static_cast<int>(CONTROL_POINTS_NB_PER_FACET_),
            static_cast<int>(CONTROL_POINTS_NB_PER_FACET_),
            static_cast<int>(CONTROL_POINTS_NB_PER_FACET_),
            static_cast<int>(CONTROL_POINTS_NB_PER_FACET_)
        };
        ELEMENT_EDGE_INTERNAL_CONTROL_POINTS_BEGIN_IDX_ = {
            1,
            2*CONTROL_POINTS_NB_PER_EDGE_-1,
            CONTROL_POINTS_NB_PER_FACET_-2,
            (CONTROL_POINTS_NB_PER_EDGE_-2)*CONTROL_POINTS_NB_PER_EDGE_,
            LAST_LAYER_BEGIN_IDX+1,
            LAST_LAYER_BEGIN_IDX+2*CONTROL_POINTS_NB_PER_EDGE_-1,
            LAST_LAYER_BEGIN_IDX+CONTROL_POINTS_NB_PER_FACET_-2,
            LAST_LAYER_BEGIN_IDX+(CONTROL_POINTS_NB_PER_EDGE_-2)*CONTROL_POINTS_NB_PER_EDGE_,
            CONTROL_POINTS_NB_PER_FACET_,
            CONTROL_POINTS_NB_PER_EDGE_-1+CONTROL_POINTS_NB_PER_FACET_,
            CONTROL_POINTS_NB_PER_FACET_-1+CONTROL_POINTS_NB_PER_FACET_,
            (CONTROL_POINTS_NB_PER_EDGE_-1)*CONTROL_POINTS_NB_PER_EDGE_+CONTROL_POINTS_NB_PER_FACET_
        };
        ELEMENT_EDGE_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP_ = ELEMENT_EDGE_CONTROL_POINTS_NEXT_IDX_STEP_;

        /* == Facet ================================================================================================ */
        ELEMENT_FACET_CONTROL_POINTS_BEGIN_IDX_ = {
            0,
            CONTROL_POINTS_NB_PER_FACET_-1,
            CONTROL_POINTS_NB_PER_EDGE_-1,
            (CONTROL_POINTS_NB_PER_EDGE_-1)*CONTROL_POINTS_NB_PER_EDGE_,
            CONTROL_POINTS_NB_PER_EDGE_-1,
            LAST_LAYER_BEGIN_IDX
        };
        ELEMENT_FACET_CONTROL_POINTS_NEXT_IDX_STEP0_ = {
            static_cast<int>(CONTROL_POINTS_NB_PER_EDGE_),
            -static_cast<int>(CONTROL_POINTS_NB_PER_EDGE_),
            -1,
            1,
            static_cast<int>(CONTROL_POINTS_NB_PER_EDGE_),
            static_cast<int>(CONTROL_POINTS_NB_PER_EDGE_)
        };
        ELEMENT_FACET_CONTROL_POINTS_NEXT_IDX_STEP1_ = {
            static_cast<int>(CONTROL_POINTS_NB_PER_FACET_),
            static_cast<int>(CONTROL_POINTS_NB_PER_FACET_),
            static_cast<int>(CONTROL_POINTS_NB_PER_FACET_),
            static_cast<int>(CONTROL_POINTS_NB_PER_FACET_),
            -1,
            1
        };
        ELEMENT_FACET_INTERNAL_CONTROL_POINTS_BEGIN_IDX_ = {
            CONTROL_POINTS_NB_PER_EDGE_+CONTROL_POINTS_NB_PER_FACET_,
            (CONTROL_POINTS_NB_PER_EDGE_-1)*CONTROL_POINTS_NB_PER_EDGE_-1+CONTROL_POINTS_NB_PER_FACET_,
            CONTROL_POINTS_NB_PER_EDGE_-2+CONTROL_POINTS_NB_PER_FACET_,
            (CONTROL_POINTS_NB_PER_EDGE_-1)*CONTROL_POINTS_NB_PER_EDGE_+1+CONTROL_POINTS_NB_PER_FACET_,
            2*CONTROL_POINTS_NB_PER_EDGE_-2,
            LAST_LAYER_BEGIN_IDX+CONTROL_POINTS_NB_PER_EDGE_+1
        };
        ELEMENT_FACET_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP0_ = ELEMENT_FACET_CONTROL_POINTS_NEXT_IDX_STEP0_;
        ELEMENT_FACET_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP1_ = ELEMENT_FACET_CONTROL_POINTS_NEXT_IDX_STEP1_;

        /* == Cell ================================================================================================= */
        ELEMENT_CONTROL_POINTS_BEGIN_IDX_ = 0;
        ELEMENT_CONTROL_POINTS_NEXT_IDX_STEP0_ = 1;
        ELEMENT_CONTROL_POINTS_NEXT_IDX_STEP1_ = static_cast<int>(CONTROL_POINTS_NB_PER_EDGE_);
        ELEMENT_CONTROL_POINTS_NEXT_IDX_STEP2_ = static_cast<int>(CONTROL_POINTS_NB_PER_FACET_);
        ELEMENT_INTERNAL_CONTROL_POINTS_BEGIN_IDX_ = CONTROL_POINTS_NB_PER_EDGE_+1+CONTROL_POINTS_NB_PER_FACET_;
        ELEMENT_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP0_ = ELEMENT_CONTROL_POINTS_NEXT_IDX_STEP0_;
        ELEMENT_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP1_ = ELEMENT_CONTROL_POINTS_NEXT_IDX_STEP1_;
        ELEMENT_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP2_ = ELEMENT_CONTROL_POINTS_NEXT_IDX_STEP2_;
    }

    void HexControlGrid::initialize_control_nodes(
        ) {
        // LOG::TRACE(__FUNCTION__);
        assert(node_positions_1D_.size() == order_+1);

        /* == Get all shared edges and facets ====================================================================== */
        GEO::index_t hex_facets_nb = 0;
        std::unordered_map<std::pair<GEO::index_t, GEO::index_t>, std::vector<GEO::index_t>, PairHash> hex_edges_control_points; /* (ev0, ev1), ev0 < ev1 -> control vertices from ev0 -> ev1 */
        {
            std::vector<bool> processed_hex_cf(8*mesh_.cells.nb(), false); // only need the first 6 facets
            for (const auto& c : mesh_.cells) {
                /* For all facets */
                for (GEO::index_t lf = 0; lf < 6; ++lf) {
                    if (processed_hex_cf[8*c+lf]) // this facet is already been processed
                        continue;

                    ++hex_facets_nb;

                    processed_hex_cf[8*c+lf] = true;
                    if (const auto nc = mesh_.cells.adjacent(c, lf);
                        nc != GEO::NO_CELL) {
                        const auto nlf = find_hex_facet(
                            mesh_,
                            nc,
                            mesh_.cells.facet_vertex(c, lf, 2),
                            mesh_.cells.facet_vertex(c, lf, 1),
                            mesh_.cells.facet_vertex(c, lf, 0));
                        assert(nlf != GEO::NO_INDEX);
                        processed_hex_cf[8*nc+nlf] = true;
                    }
                }

                /* For all edges */
                for (GEO::index_t le = 0; le < 12; ++le) {
                    const std::pair<GEO::index_t, GEO::index_t> edge = std::minmax(
                        mesh_.cells.edge_vertex(c, le, 0),
                        mesh_.cells.edge_vertex(c, le, 1));
                    hex_edges_control_points.emplace(edge, std::vector<GEO::index_t>(INTERNAL_CONTROL_POINTS_NB_PER_EDGE_, GEO::NO_VERTEX));
                }
            }
        }

        // LOG::DEBUG("found {} facets and {} edges in the hex mesh", hex_facets_nb, hex_edges_control_points.size());

        /* == Create grid elements ================================================================================= */
        control_nodes_.vertices.clear();
        GEO::index_t new_v = control_nodes_.vertices.create_vertices(
                            mesh_.vertices.nb() + // vertices
                            hex_edges_control_points.size() * INTERNAL_CONTROL_POINTS_NB_PER_EDGE_ + // edges
                            hex_facets_nb * INTERNAL_CONTROL_POINTS_NB_PER_FACET_ + // facets
                            mesh_.cells.nb() * INTERNAL_CONTROL_POINTS_NB_PER_CELL_ // cells
                            );
        assert(new_v == 0);

        /* == For vertices == */
        for (const auto& v : mesh_.vertices)
            control_node(new_v++) = mesh_.vertices.point(v);

        /* == For edges == */
        for (auto& [edge, control_vertices] : hex_edges_control_points) {
            const auto& ep0 = mesh_.vertices.point(edge.first);
            const auto& ep1 = mesh_.vertices.point(edge.second);
            for (GEO::index_t i = 0; i < INTERNAL_CONTROL_POINTS_NB_PER_EDGE_; ++i) {
                const double r = node_positions_1D_[i+1];
                control_node(new_v) = (1-r)*ep0 + r*ep1;
                control_vertices[i] = new_v;
                ++new_v;
            }
        }

        /* == For facets == */
        std::vector<std::vector<GEO::index_t>> hex_facets_control_points(8*mesh_.cells.nb()); /*
            [8*c+lf] -> the idx of the control points of this cell facet,
                        from fv0 -> fv1, ..., fv3 -> fv2 */
        for (const auto& c : mesh_.cells) {
            assert(mesh_.cells.nb_facets(c) == 6);
            for (GEO::index_t lf = 0; lf < 6; ++lf) {
                if (!hex_facets_control_points[8*c+lf].empty())
                    continue;

                assert(mesh_.cells.facet_nb_vertices(c, lf) == 4);
                const auto& lf_v0 = mesh_.cells.facet_vertex(c, lf, 0);
                const auto& lf_v1 = mesh_.cells.facet_vertex(c, lf, 1);
                const auto& lf_v2 = mesh_.cells.facet_vertex(c, lf, 2);
                const auto& lf_v3 = mesh_.cells.facet_vertex(c, lf, 3);
                const auto& lf_p0 = mesh_.vertices.point(lf_v0);
                const auto& lf_p1 = mesh_.vertices.point(lf_v1);
                const auto& lf_p2 = mesh_.vertices.point(lf_v2);
                const auto& lf_p3 = mesh_.vertices.point(lf_v3);

                auto& lf_control_points = hex_facets_control_points[8*c+lf];
                lf_control_points.reserve(INTERNAL_CONTROL_POINTS_NB_PER_FACET_);

                for (GEO::index_t i = 0; i < INTERNAL_CONTROL_POINTS_NB_PER_EDGE_; ++i) {
                    const double ri = node_positions_1D_[i+1];
                    for (GEO::index_t j = 0; j < INTERNAL_CONTROL_POINTS_NB_PER_EDGE_; ++j) {
                        const double rj = node_positions_1D_[j+1];

                        control_node(new_v) = (1-ri)*(1-rj)*lf_p0
                                            + ri*(1-rj)*lf_p3
                                            + (1-ri)*rj*lf_p1
                                            + ri*rj*lf_p2;
                        lf_control_points.push_back(new_v);
                        ++new_v;
                    }
                }

                /* Assign to adjacent cell facet */
                if (const auto& nc = mesh_.cells.adjacent(c, lf);
                    nc != GEO::NO_CELL) {
                    const auto nlf = find_hex_facet(
                        mesh_,
                        nc,
                        mesh_.cells.facet_vertex(c, lf, 2),
                        mesh_.cells.facet_vertex(c, lf, 1),
                        mesh_.cells.facet_vertex(c, lf, 0));
                    assert(nlf != GEO::NO_INDEX);

                    auto& nclf_control_points = hex_facets_control_points[8*nc+nlf];
                    nclf_control_points.reserve(INTERNAL_CONTROL_POINTS_NB_PER_FACET_);

                    assert(mesh_.cells.facet_nb_vertices(nc, nlf) == 4);
                    if (const auto& nclf_v0 = mesh_.cells.facet_vertex(nc, nlf, 0);
                        nclf_v0 == lf_v0
                        ) {
                        for (GEO::index_t i = 0, i_end = order_-1; i < i_end; ++i) {
                            for (GEO::index_t j = 0, j_end = order_-1; j < j_end; ++j)
                                nclf_control_points.push_back(lf_control_points[i+j*(order_-1)]);
                        }
                    }
                    else if (nclf_v0 == lf_v1) {
                        for (GEO::index_t i = 0, i_end = order_-1; i < i_end; ++i) {
                            for (GEO::index_t j = 0, j_end = order_-1; j < j_end; ++j)
                                nclf_control_points.push_back(lf_control_points[(order_-1)*(i+1)-1-j]);
                        }
                    }
                    else if (nclf_v0 == lf_v2) {
                        for (GEO::index_t i = 0, i_end = order_-1; i < i_end; ++i) {
                            for (GEO::index_t j = 0, j_end = order_-1; j < j_end; ++j)
                                nclf_control_points.push_back(lf_control_points[(order_-1)*(order_-1)-1-i-(order_-1)*j]);
                        }
                    }
                    else if (nclf_v0 == lf_v3) {
                        for (GEO::index_t i = 0, i_end = order_-1; i < i_end; ++i) {
                            for (GEO::index_t j = 0, j_end = order_-1; j < j_end; ++j)
                                nclf_control_points.push_back(lf_control_points[(order_-1)*(order_-2-i)+j]);
                        }
                    }
                    else
                        assert(0);
                }
            }
        }

        /* == For cells == */
        std::vector<std::vector<GEO::index_t>> hex_cells_control_points(mesh_.cells.nb()); /*
            [c] -> the idx of control points of this cell
                   from cv0 -> cv1, cv2 -> cv3, ..., cv4 -> cv5, ..., cv6 -> cv7 */
        for (const auto& c : mesh_.cells) {
            auto& c_control_points = hex_cells_control_points[c];
            c_control_points.reserve(INTERNAL_CONTROL_POINTS_NB_PER_CELL_);

            assert(mesh_.cells.nb_vertices(c) == 8);
            const auto& c_p0 = mesh_.cells.point(c, 0);
            const auto& c_p1 = mesh_.cells.point(c, 4);
            const auto& c_p2 = mesh_.cells.point(c, 2);
            const auto& c_p3 = mesh_.cells.point(c, 6);
            const auto& c_p4 = mesh_.cells.point(c, 1);
            const auto& c_p5 = mesh_.cells.point(c, 5);
            const auto& c_p6 = mesh_.cells.point(c, 3);
            const auto& c_p7 = mesh_.cells.point(c, 7);

            for (GEO::index_t i = 0; i < INTERNAL_CONTROL_POINTS_NB_PER_EDGE_; ++i) {
                const double ri = node_positions_1D_[i+1];
                for (GEO::index_t j = 0; j < INTERNAL_CONTROL_POINTS_NB_PER_EDGE_; ++j) {
                    const double rj = node_positions_1D_[j+1];
                    for (GEO::index_t k = 0; k < INTERNAL_CONTROL_POINTS_NB_PER_EDGE_; ++k) {
                        const double rk = node_positions_1D_[k+1];

                        control_node(new_v) = (1-ri)*(1-rj)*(1-rk)*c_p0
                                                + ri*(1-rj)*(1-rk)*c_p1
                                                + (1-ri)*rj*(1-rk)*c_p2
                                                + ri*rj*(1-rk)*c_p3
                                                + (1-ri)*(1-rj)*rk*c_p4
                                                + ri*(1-rj)*rk*c_p5
                                                + (1-ri)*rj*rk*c_p6
                                                + ri*rj*rk*c_p7;
                        c_control_points.push_back(new_v);
                        ++new_v;
                    }
                }
            }
        }

        assert(new_v == control_nodes_nb());

        /* == Create regular index ================================================================================= */
        element_control_nodes_.assign(CONTROL_POINTS_NB_PER_CELL_ * mesh_.cells.nb(), GEO::NO_VERTEX);
        /* [(order+1)^3 * c + lv] -> hex cell c's control vertex lv
            For a hex (0, 1, 2, 3, 4, 5, 6, 7),

              +Z                4-------6        lf-0: (0-2-6-4)     le-0:  (0-1)
              |                /|      /|        lf-1: (3-1-5-7)     le-1:  (1-3)
              o --- +Y        5-------7 |        lf-2: (1-0-4-5)     le-2:  (3-2)
             /                | 0-----|-2        lf-3: (2-3-7-6)     le-3:  (2-0)
            +X                |/      |/         lf-4: (1-3-2-0)     le-4:  (4-5)
                              1-------3          lf-5: (4-6-7-5)     le-5:  (5-7)
                                                                     le-6:  (7-6)
                                                                     le-7:  (6-4)
                                                                     le-8:  (0-4)
                                                                     le-9:  (1-5)
                                                                     le-10: (3-7)
                                                                     le-11: (2-6)

            the arrangement of the control points is:
                dimension 1: cv0 -> cv1, dimension 2: cv0 -> cv2 ,dimension 3: cv0 -> cv4 */

        for (const auto& c : mesh_.cells) {
            const GEO::index_t CELL_BEGIN_IDX = c*CONTROL_POINTS_NB_PER_CELL_;

            /* For vertices */
            for (GEO::index_t lv = 0; lv < 8; ++lv)
                element_control_nodes_[
                    CELL_BEGIN_IDX +
                    cell_vertex_lnd(lv)
                    ] = mesh_.cells.vertex(c, lv);

            /* For edges */
            for (GEO::index_t le = 0; le < 12; ++le) {
                const auto& ev0 = mesh_.cells.edge_vertex(c, le, 0);
                const auto& ev1 = mesh_.cells.edge_vertex(c, le, 1);
                const std::pair<GEO::index_t, GEO::index_t> edge = std::minmax(ev0, ev1);

                assert(hex_edges_control_points.contains(edge));
                const auto& edge_control_points = hex_edges_control_points.at(edge);
                assert(edge_control_points.size() == INTERNAL_CONTROL_POINTS_NB_PER_EDGE_);

                if (ev0 == edge.first) { // do not need to inverse
                    for (GEO::index_t lv = 0; lv < INTERNAL_CONTROL_POINTS_NB_PER_EDGE_; ++lv)
                        element_control_nodes_[
                            CELL_BEGIN_IDX +
                            cell_edge_inner_lnd(le, lv)
                            ] = edge_control_points[lv];
                }
                else { // need to inverse
                    for (GEO::index_t lv = 0; lv < INTERNAL_CONTROL_POINTS_NB_PER_EDGE_; ++lv)
                        element_control_nodes_[
                            CELL_BEGIN_IDX +
                            cell_edge_inner_lnd(le, lv)
                            ] = edge_control_points[INTERNAL_CONTROL_POINTS_NB_PER_EDGE_-1-lv];
                }
            }

            /* For facets */
            for (GEO::index_t lf = 0; lf < 6; ++lf) {
                const auto& facet_control_points = hex_facets_control_points[8*c+lf];
                assert(facet_control_points.size() == INTERNAL_CONTROL_POINTS_NB_PER_FACET_);

                for (GEO::index_t lv1 = 0; lv1 < INTERNAL_CONTROL_POINTS_NB_PER_EDGE_; ++lv1) {
                    for (GEO::index_t lv0 = 0; lv0 < INTERNAL_CONTROL_POINTS_NB_PER_EDGE_; ++lv0)
                        element_control_nodes_[
                            CELL_BEGIN_IDX +
                            cell_facet_inner_lnd(lf, lv0, lv1)
                            ] = facet_control_points[lv1*INTERNAL_CONTROL_POINTS_NB_PER_EDGE_ + lv0];
                }
            }

            /* For cells */
            const auto& cell_control_points = hex_cells_control_points[c];
            assert(cell_control_points.size() == INTERNAL_CONTROL_POINTS_NB_PER_CELL_);

            for (GEO::index_t lv2 = 0; lv2 < INTERNAL_CONTROL_POINTS_NB_PER_EDGE_; ++lv2) {
                for (GEO::index_t lv1 = 0; lv1 < INTERNAL_CONTROL_POINTS_NB_PER_EDGE_; ++lv1) {
                    for (GEO::index_t lv0 = 0; lv0 < INTERNAL_CONTROL_POINTS_NB_PER_EDGE_; ++lv0) {
                        element_control_nodes_[
                            CELL_BEGIN_IDX +
                            cell_inner_lnd(lv0, lv1, lv2)
                            ] = cell_control_points[lv2*INTERNAL_CONTROL_POINTS_NB_PER_FACET_ + lv1*INTERNAL_CONTROL_POINTS_NB_PER_EDGE_ + lv0];
                    }
                }
            }
        }
    }

    void HexControlGrid::append_discretized_high_order_cells_border(
        GEO::Mesh& mesh_out,
        const GEO::index_t resolution,
        GEO::Attribute<GEO::index_t>* mesh_out_v_cell,
        GEO::Attribute<GEO::vec3>* mesh_out_v_uvw,
        GEO::Attribute<GEO::index_t>* mesh_out_f_cell
        ) const {
        if (mesh_out_v_cell != nullptr) {
            assert(mesh_out_v_cell->is_bound());
            assert(mesh_out_v_cell->size() == mesh_out.vertices.nb());
        }
        if (mesh_out_v_uvw != nullptr) {
            assert(mesh_out_v_uvw->is_bound());
            assert(mesh_out_v_uvw->size() == mesh_out.vertices.nb());
        }
        if (mesh_out_f_cell != nullptr) {
            assert(mesh_out_f_cell->is_bound());
            assert(mesh_out_f_cell->size() == mesh_out.facets.nb());
        }

        const GEO::index_t VERTICES_NB_PER_EDGE = resolution+1;

        GEO::index_t new_v = mesh_out.vertices.create_vertices(6*mesh_.cells.nb() * (resolution+1) * (resolution+1));
        GEO::index_t new_f = mesh_out.facets.create_quads(6*mesh_.cells.nb() * resolution * resolution);
        for (const auto& c : mesh_.cells) {
            for (GEO::index_t lf = 0; lf < 6; ++lf) {
                const auto PREV_M_VERTICES = new_v;

                /*
                 * Vertices:
                 * y & j
                 *   |
                 *  ...
                 *   |         |       ...       |         |
                 * (0,1) --- (1,1) --- ... --- (n,1) -- (n+1,1)
                 *   |         |       ...       |         |
                 * (0,0) --- (1,0) --- ... --- (n,0) -- (n+1,0) -> x & i
                 */
                for (GEO::index_t i = 0; i < VERTICES_NB_PER_EDGE; ++i) {
                    const double u = static_cast<double>(i)/resolution;
                    for (GEO::index_t j = 0; j < VERTICES_NB_PER_EDGE; ++j) {
                        const double v = static_cast<double>(j)/resolution;

                        const GEO::vec3 uvw = project_hex_lf_uv_to_uvw(GEO::vec2(u,v), lf);

                        mesh_out.vertices.point(new_v) = compute_cell_uvw_position(c, uvw);

                        if (mesh_out_v_cell != nullptr)
                            (*mesh_out_v_cell)[new_v] = c;
                        if (mesh_out_v_uvw != nullptr)
                            (*mesh_out_v_uvw)[new_v] = uvw;

                        ++new_v;
                    }
                }

                /*
                 * Facets:
                 * ...
                 * +-----+-----+- ... -+-----+      v3 --- v2
                 * | n+1 | n+2 |  ...  |2n+1 |       |     |
                 * +-----+-----+- ... -+-----+      v0 --- v1
                 * |  0  |  1  |  ...  |  n  |
                 * +-----+-----+- ... -+-----+
                 */
                for (GEO::index_t i = 0; i < resolution; ++i) {
                    for (GEO::index_t j = 0; j < resolution; ++j) {
                        const GEO::index_t v0 = VERTICES_NB_PER_EDGE*i+j;
                        const GEO::index_t v1 = v0+VERTICES_NB_PER_EDGE;
                        const GEO::index_t v2 = v1+1;
                        const GEO::index_t v3 = v0+1;
                        assert(v0 < mesh_out.vertices.nb());
                        assert(v1 < mesh_out.vertices.nb());
                        assert(v2 < mesh_out.vertices.nb());
                        assert(v3 < mesh_out.vertices.nb());
                        mesh_out.facets.set_vertex(new_f, 0, PREV_M_VERTICES+v0);
                        mesh_out.facets.set_vertex(new_f, 1, PREV_M_VERTICES+v1);
                        mesh_out.facets.set_vertex(new_f, 2, PREV_M_VERTICES+v2);
                        mesh_out.facets.set_vertex(new_f, 3, PREV_M_VERTICES+v3);

                        if (mesh_out_f_cell != nullptr)
                            (*mesh_out_f_cell)[new_f] = c;

                        ++new_f;
                    }
                }
            }
        }

        mesh_out.facets.connect();
    }

    void HexControlGrid::append_discretized_high_order_cells(
        GEO::Mesh& mesh_out,
        const GEO::index_t resolution,
        GEO::Attribute<GEO::index_t>* mesh_out_v_cell,
        GEO::Attribute<GEO::vec3>* mesh_out_v_uvw,
        GEO::Attribute<GEO::index_t>* mesh_out_c_cell
        ) const {
        if (mesh_out_v_cell != nullptr) {
            assert(mesh_out_v_cell->is_bound());
            assert(mesh_out_v_cell->size() == mesh_out.vertices.nb());
        }
        if (mesh_out_v_uvw != nullptr) {
            assert(mesh_out_v_uvw->is_bound());
            assert(mesh_out_v_uvw->size() == mesh_out.vertices.nb());
        }
        if (mesh_out_c_cell != nullptr) {
            assert(mesh_out_c_cell->is_bound());
            assert(mesh_out_c_cell->size() == mesh_out.cells.nb());
        }

        const GEO::index_t VERTICES_NB_PER_EDGE = resolution+1;
        const GEO::index_t VERTICES_NB_PER_FACET = VERTICES_NB_PER_EDGE * VERTICES_NB_PER_EDGE;
        GEO::index_t new_v = mesh_out.vertices.create_vertices(mesh_.cells.nb() * (resolution+1) * (resolution+1) * (resolution+1));
        GEO::index_t new_c = mesh_out.cells.create_hexes(mesh_.cells.nb() * resolution * resolution * resolution);
        for (const auto& c : mesh_.cells) {
            const auto PREV_M_VERTICES = new_v;

            /* Vertices */
            for (GEO::index_t i = 0; i < VERTICES_NB_PER_EDGE; ++i) {
                const double u = static_cast<double>(i)/resolution;
                for (GEO::index_t j = 0; j < VERTICES_NB_PER_EDGE; ++j) {
                    const double v = static_cast<double>(j)/resolution;
                    for (GEO::index_t k = 0; k < VERTICES_NB_PER_EDGE; ++k) {
                        const double w = static_cast<double>(k)/resolution;
                        const GEO::vec3 uvw(u, v, w);
                        mesh_out.vertices.point(new_v) = compute_cell_uvw_position(c, uvw);

                        if (mesh_out_v_cell != nullptr)
                            (*mesh_out_v_cell)[new_v] = c;
                        if (mesh_out_v_uvw != nullptr)
                            (*mesh_out_v_uvw)[new_v] = uvw;

                        ++new_v;
                    }
                }
            }

            /* Cells
             *    +Z                4-------6
             *    |                /|      /|
             *    o --- +Y        5-------7 |
             *   /                | 0-----|-2
             *  +X                |/      |/
             *                    1-------3
             */
            for (GEO::index_t i = 0; i < resolution; ++i) {
                for (GEO::index_t j = 0; j < resolution; ++j) {
                    for (GEO::index_t k = 0; k < resolution; ++k) {
                        const GEO::index_t v0 = VERTICES_NB_PER_FACET*i+VERTICES_NB_PER_EDGE*j+k;
                        const GEO::index_t v1 = v0+VERTICES_NB_PER_FACET;
                        const GEO::index_t v2 = v0+VERTICES_NB_PER_EDGE;
                        const GEO::index_t v3 = v1+VERTICES_NB_PER_EDGE;
                        const GEO::index_t v4 = v0+1;
                        const GEO::index_t v5 = v4+VERTICES_NB_PER_FACET;
                        const GEO::index_t v6 = v4+VERTICES_NB_PER_EDGE;
                        const GEO::index_t v7 = v5+VERTICES_NB_PER_EDGE;
                        assert(v0 < mesh_out.vertices.nb());
                        assert(v1 < mesh_out.vertices.nb());
                        assert(v2 < mesh_out.vertices.nb());
                        assert(v3 < mesh_out.vertices.nb());
                        assert(v4 < mesh_out.vertices.nb());
                        assert(v5 < mesh_out.vertices.nb());
                        assert(v6 < mesh_out.vertices.nb());
                        assert(v7 < mesh_out.vertices.nb());
                        mesh_out.cells.set_vertex(new_c, 0, PREV_M_VERTICES+v0);
                        mesh_out.cells.set_vertex(new_c, 1, PREV_M_VERTICES+v1);
                        mesh_out.cells.set_vertex(new_c, 2, PREV_M_VERTICES+v2);
                        mesh_out.cells.set_vertex(new_c, 3, PREV_M_VERTICES+v3);
                        mesh_out.cells.set_vertex(new_c, 4, PREV_M_VERTICES+v4);
                        mesh_out.cells.set_vertex(new_c, 5, PREV_M_VERTICES+v5);
                        mesh_out.cells.set_vertex(new_c, 6, PREV_M_VERTICES+v6);
                        mesh_out.cells.set_vertex(new_c, 7, PREV_M_VERTICES+v7);

                        if (mesh_out_c_cell != nullptr)
                            (*mesh_out_c_cell)[new_c] = c;

                        ++new_c;
                    }
                }
            }
        }

        mesh_out.cells.connect();
    }
}
