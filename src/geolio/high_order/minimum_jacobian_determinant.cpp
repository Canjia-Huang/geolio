//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/11.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "minimum_jacobian_determinant.h"
#include <geogram/mesh/mesh_io.h>
#include "control_grid.h"
#include <geolio/mesh/hex_descriptor.h>
#include <unsupported/Eigen/KroneckerProduct>
#include <geolio/common/log.h>

namespace
{
    /**
     * Compute the binomial coefficient "n choose k" as a double.
     *
     * This function computes the combinatorial number C(n, k) using an
     * iterative multiplicative formula to avoid large intermediate
     * factorials. It asserts that 0 <= k <= n.
     *
     * @param[in] n Upper value in the binomial coefficient (non-negative).
     * @param[in] k Lower value in the binomial coefficient (0 <= k <= n).
     * @return The value of C(n, k) represented as a double.
     */
    double combinations(
        const GEO::index_t n,
        const GEO::index_t k
        ) {
        assert(k <= n);
        double res = 1;
        for (GEO::index_t i = 1; i <= k; ++i)
            res = res * static_cast<double>(n-k+i)/i;
        return res;
    }

    /**
     * Evaluate the Bernstein basis polynomial B^n_j(u).
     *
     * The Bernstein basis of degree n and index j at parameter u is
     * defined as: B^n_j(u) = C(n, j) * u^j * (1-u)^(n-j). This helper is
     * used to build the Bezier / Bernstein transform matrix for mapping
     * sampled determinant values to Bernstein coefficients.
     *
     * @param[in] j Basis index (0 <= j <= n).
     * @param[in] n Degree of the Bernstein polynomial (n >= 0).
     * @param[in] u Evaluation parameter in [0,1].
     * @return Value of the Bernstein basis B^n_j(u).
     */
    double bernstein_basis(
        const GEO::index_t j,
        const GEO::index_t n,
        const double u
        ) {
        return combinations(n, j) * std::pow(u, j) * std::pow(1.0-u, n-j);
    }
}

namespace geolio
{
    template<GEO::index_t DIM, typename CONTROL_GRID>
    MinimumJacobianDeterminant<DIM, CONTROL_GRID>::MinimumJacobianDeterminant(
        const CONTROL_GRID& control_grid
        ) : control_grid_(control_grid),
            ORDER_(control_grid.order())
    {
        if constexpr (std::is_same_v<CONTROL_GRID, QuadControlGrid<2>> || std::is_same_v<CONTROL_GRID, QuadControlGrid<3>>) {
            n_ = 2*ORDER_-1;
            N1_ = n_+1;
            N2_ = N1_*N1_;
            CORNER_INDICES_.resize(4);
            CORNER_INDICES_[0] = {0         + 0*N1_      };
            CORNER_INDICES_[1] = {(N1_-1)   + 0*N1_      };
            CORNER_INDICES_[2] = {(N1_-1)   + (N1_-1)*N1_};
            CORNER_INDICES_[3] = {0         + (N1_-1)*N1_};
        }
        else if constexpr (std::is_same_v<CONTROL_GRID, HexControlGrid>) {
            n_ = 3*ORDER_-1;
            N1_ = n_+1;
            N2_ = N1_*N1_;
            N3_ = N1_*N1_*N1_;
            CORNER_INDICES_.resize(8);
            CORNER_INDICES_[0] = {0         + 0*N1_         + 0*N2_};
            CORNER_INDICES_[1] = {(N1_-1)   + 0*N1_         + 0*N2_};
            CORNER_INDICES_[2] = {0         + (N1_-1)*N1_   + 0*N2_};
            CORNER_INDICES_[3] = {(N1_-1)   + (N1_-1)*N1_   + 0*N2_};
            CORNER_INDICES_[4] = {0         + 0*N1_         + (N1_-1)*N2_};
            CORNER_INDICES_[5] = {(N1_-1)   + 0*N1_         + (N1_-1)*N2_};
            CORNER_INDICES_[6] = {0         + (N1_-1)*N1_   + (N1_-1)*N2_};
            CORNER_INDICES_[7] = {(N1_-1)   + (N1_-1)*N1_   + (N1_-1)*N2_};
        }
        else
            static_assert(false);

        samples_1D_.reserve(N1_);
        for (GEO::index_t i = 0; i < N1_; ++i)
            samples_1D_.push_back(static_cast<double>(i)/n_);

        pre_compute_Lagrange_to_Bernstein_matrix();
        pre_compute_subdivision_matrices();
    }

    template<GEO::index_t DIM, typename CONTROL_GRID>
    bool MinimumJacobianDeterminant<DIM, CONTROL_GRID>::contains_inverted_region(
        const GEO::index_t c,
        const double eps
        ) {
        initialize_priority_queue(c);

        while (!pq_.empty()) {
            const auto& B = pq_.top();

            for (unsigned int& i : CORNER_INDICES_) {
                if (B.C[i] < 0) // corner values are exact
                    return true;
            }

            if (B.min_c > 0) {
                /* Do nothing */
                pq_.pop();
            }
            else if (B.max_c < 0)
                return true;
            else if (B.max_c - B.min_c < eps) {
                if (B.min_c < 0)
                    return true;
                return false;
            }
            else {
                /* Subdivide */
                std::vector<Block> sub_blocks;
                subdivide(B, sub_blocks);

                pq_.pop();
                for (const auto& block : sub_blocks)
                    pq_.push(block);
            }
        }

        return false;
    }

    // template<GEO::index_t DIM, typename CONTROL_GRID>
    // double MinimumJacobianDeterminant<DIM, CONTROL_GRID>::compute_upper_bound(
    //     const GEO::index_t c,
    //     const double eps,
    //     std::vector<Block>* travelled_sub_blocks
    //     ) {
    //     // LOG::TRACE(__FUNCTION__);
    //
    //     initialize_priority_queue(c);
    //
    //     double global_upper_bound = std::numeric_limits<double>::max();
    //     while (!pq_.empty()) {
    //         const auto& B = pq_.top();
    //         for (GEO::index_t i = 0; i < 8; ++i)
    //             global_upper_bound = std::min(global_upper_bound, B.C[CORNER_INDICES_[i]]);
    //
    //         if (travelled_sub_blocks != nullptr) // for debug
    //             travelled_sub_blocks->push_back(B);
    //
    //         if (B.min_c > global_upper_bound) {
    //             /* Do nothing */
    //             pq_.pop();
    //         }
    //         else if (B.max_c - B.min_c < eps) {
    //             global_upper_bound = B.min_c;
    //             break;
    //         }
    //         else {
    //             /* Subdivide */
    //             std::vector<Block> sub_blocks;
    //             subdivide(B, sub_blocks);
    //
    //             pq_.pop();
    //             for (GEO::index_t i = 0; i < 8; ++i)
    //                 pq_.push(sub_blocks[i]);
    //         }
    //     }
    //
    //     return global_upper_bound;
    // }
    //
    // template<GEO::index_t DIM, typename CONTROL_GRID>
    // void MinimumJacobianDeterminant<DIM, CONTROL_GRID>::collect_invalid_sub_blocks(
    //     const GEO::index_t c,
    //     std::vector<Block>& invalid_sub_blocks,
    //     const double eps
    //     ) {
    //     // LOG::TRACE(__FUNCTION__);
    //
    //     initialize_priority_queue(c);
    //
    //     while (!pq_.empty()) {
    //         if (const auto& B = pq_.top();
    //             B.min_c > 0
    //             ) {
    //             /* Do nothing */
    //             pq_.pop();
    //         }
    //         else if (B.max_c < 0 || B.max_c - B.min_c < eps) {
    //             invalid_sub_blocks.push_back(B);
    //             pq_.pop();
    //         }
    //         else {
    //             /* Subdivide */
    //             std::vector<Block> sub_blocks;
    //             subdivide(B, sub_blocks);
    //
    //             pq_.pop();
    //             for (GEO::index_t i = 0; i < 8; ++i)
    //                 pq_.push(sub_blocks[i]);
    //         }
    //     }
    // }
    //
    // template<GEO::index_t DIM, typename CONTROL_GRID>
    // void MinimumJacobianDeterminant<DIM, CONTROL_GRID>::append_blocks_to_mesh(
    //     const std::vector<Block>& blocks,
    //     GEO::Mesh& M_out
    //     ) const {
    //     LOG::TRACE("{}({})", __FUNCTION__, blocks.size());
    //
    //     GEO::Attribute<double> M_out_v_mindetJ(M_out.vertices.attributes(), "min_det_J");
    //     const GEO::index_t new_v = M_out.vertices.create_vertices(8*blocks.size());
    //     const GEO::index_t new_e = M_out.edges.create_edges(12*blocks.size());
    //     for (GEO::index_t i = 0, i_end = blocks.size(); i < i_end; ++i) {
    //         const auto& B = blocks[i];
    //         M_out.vertices.point(new_v+8*i)     = GEO::vec3(B.min_u, B.min_v, B.min_w);
    //         M_out.vertices.point(new_v+8*i+1)   = GEO::vec3(B.max_u, B.min_v, B.min_w);
    //         M_out.vertices.point(new_v+8*i+2)   = GEO::vec3(B.min_u, B.max_v, B.min_w);
    //         M_out.vertices.point(new_v+8*i+3)   = GEO::vec3(B.max_u, B.max_v, B.min_w);
    //         M_out.vertices.point(new_v+8*i+4)   = GEO::vec3(B.min_u, B.min_v, B.max_w);
    //         M_out.vertices.point(new_v+8*i+5)   = GEO::vec3(B.max_u, B.min_v, B.max_w);
    //         M_out.vertices.point(new_v+8*i+6)   = GEO::vec3(B.min_u, B.max_v, B.max_w);
    //         M_out.vertices.point(new_v+8*i+7)   = GEO::vec3(B.max_u, B.max_v, B.max_w);
    //         M_out_v_mindetJ[new_v+8*i]      = B.C[CORNER_INDICES_[0]];
    //         M_out_v_mindetJ[new_v+8*i+1]    = B.C[CORNER_INDICES_[1]];
    //         M_out_v_mindetJ[new_v+8*i+2]    = B.C[CORNER_INDICES_[2]];
    //         M_out_v_mindetJ[new_v+8*i+3]    = B.C[CORNER_INDICES_[3]];
    //         M_out_v_mindetJ[new_v+8*i+4]    = B.C[CORNER_INDICES_[4]];
    //         M_out_v_mindetJ[new_v+8*i+5]    = B.C[CORNER_INDICES_[5]];
    //         M_out_v_mindetJ[new_v+8*i+6]    = B.C[CORNER_INDICES_[6]];
    //         M_out_v_mindetJ[new_v+8*i+7]    = B.C[CORNER_INDICES_[7]];
    //         for (GEO::index_t le = 0; le < 12; ++le) {
    //             M_out.edges.set_vertex(new_e+12*i+le, 0, new_v+8*i+geolio::HEX_LE_INCIDENT_LV[le][0]);
    //             M_out.edges.set_vertex(new_e+12*i+le, 1, new_v+8*i+geolio::HEX_LE_INCIDENT_LV[le][1]);
    //         }
    //     }
    // }

    template<GEO::index_t DIM, typename CONTROL_GRID>
    void MinimumJacobianDeterminant<DIM, CONTROL_GRID>::compute_samples_det_J(
        const GEO::index_t c,
        Eigen::VectorXd& det_J
        ) const {
        if constexpr (std::is_same_v<CONTROL_GRID, QuadControlGrid<2>> || std::is_same_v<CONTROL_GRID, QuadControlGrid<3>>) {
            det_J = Eigen::VectorXd::Zero(N2_);

            for (GEO::index_t j = 0; j < N1_; ++j) {
                for (GEO::index_t i = 0; i < N1_; ++i) {
                    const GEO::vec2 uv(samples_1D_[i], samples_1D_[j]);
                    det_J(i+j*N1_) = control_grid_.compute_facet_uv_measure(c, uv, QuadControlGrid<DIM>::MeasureType::DET_JACOBIAN);
                }
            }
        }
        else if constexpr (std::is_same_v<CONTROL_GRID, HexControlGrid>) {
            det_J = Eigen::VectorXd::Zero(N3_);

            for (GEO::index_t k = 0; k < N1_; ++k) {
                for (GEO::index_t j = 0; j < N1_; ++j) {
                    for (GEO::index_t i = 0; i < N1_; ++i) {
                        const GEO::vec3 uvw(samples_1D_[i], samples_1D_[j], samples_1D_[k]);
                        det_J(i+j*N1_+k*N2_) = control_grid_.compute_cell_uvw_measure(c, uvw, HexControlGrid::MeasureType::DET_JACOBIAN);
                    }
                }
            }
        }
        else
            static_assert(false);
    }

    // template<GEO::index_t DIM, typename CONTROL_GRID>
    // void MinimumJacobianDeterminant<DIM, CONTROL_GRID>::compute_samples_grad_det_J(
    //     const GEO::index_t c,
    //     Eigen::MatrixXd& grad_det_J
    //     ) const {
    //     const auto& CONTROL_POINTS_NB_PER_CELL = control_grid_.control_nodes_nb_per_cell();
    //
    //     grad_det_J = Eigen::MatrixXd::Zero(N3_, 3*CONTROL_POINTS_NB_PER_CELL);
    //
    //     for (GEO::index_t k = 0; k < N1_; ++k) {
    //         for (GEO::index_t j = 0; j < N1_; ++j) {
    //             for (GEO::index_t i = 0; i < N1_; ++i) {
    //                 const GEO::vec3 uvw(samples_1D_[i], samples_1D_[j], samples_1D_[k]);
    //                 std::vector<double> grads;
    //                 control_grid_.compute_cell_uvw_detJ_gradient(c, uvw, grads);
    //                 assert(grads.size() == 3*CONTROL_POINTS_NB_PER_CELL);
    //
    //                 const GEO::index_t I = i+j*N1_+k*N2_;
    //                 for (GEO::index_t ii = 0; ii < 3*CONTROL_POINTS_NB_PER_CELL; ++ii)
    //                     grad_det_J(I, ii) = grads[ii];
    //             }
    //         }
    //     }
    // }

    template<GEO::index_t DIM, typename CONTROL_GRID>
    void MinimumJacobianDeterminant<DIM, CONTROL_GRID>::convert_to_bernstein_coeffs(
        const Eigen::VectorXd& J,
        Eigen::VectorXd& C
        ) const {
        if constexpr (std::is_same_v<CONTROL_GRID, QuadControlGrid<2>> || std::is_same_v<CONTROL_GRID, QuadControlGrid<3>>)
            C = M2D_ * J;
        else if constexpr (std::is_same_v<CONTROL_GRID, HexControlGrid>)
            C = M3D_ * J;
        else
            static_assert(false);
    }

    template<GEO::index_t DIM, typename CONTROL_GRID>
    void MinimumJacobianDeterminant<DIM, CONTROL_GRID>::convert_to_bernstein_coeffs(
        const Eigen::MatrixXd& J,
        Eigen::MatrixXd& C
        ) const {
        if constexpr (std::is_same_v<CONTROL_GRID, QuadControlGrid<2>> || std::is_same_v<CONTROL_GRID, QuadControlGrid<3>>)
            C = M2D_ * J;
        else if constexpr (std::is_same_v<CONTROL_GRID, HexControlGrid>)
            C = M3D_ * J;
        else
            static_assert(false);
    }

    template<GEO::index_t DIM, typename CONTROL_GRID>
    void MinimumJacobianDeterminant<DIM, CONTROL_GRID>::pre_compute_Lagrange_to_Bernstein_matrix(
        ) {
        // LOG::TRACE(__FUNCTION__);

        M_ = Eigen::MatrixXd::Zero(N1_, N1_);

        for (GEO::index_t i = 0; i < N1_; ++i) {
            for (GEO::index_t j = 0; j < N1_; ++j)
                M_(i, j) = bernstein_basis(j, n_, samples_1D_[i]);
        }
        M_ = M_.inverse();

        MT_ = M_.transpose();

        M2D_ = Eigen::KroneckerProduct(M_, M_).eval();
        M2D_T_ = M2D_.transpose();

        M3D_ = kroneckerProduct(M_, M2D_).eval();
    }

    template<GEO::index_t DIM, typename CONTROL_GRID>
    void MinimumJacobianDeterminant<DIM, CONTROL_GRID>::pre_compute_subdivision_matrices(
        ) {
        // LOG::TRACE(__FUNCTION__);

        ML_ = Eigen::MatrixXd::Zero(N1_, N1_);
        MR_ = Eigen::MatrixXd::Zero(N1_, N1_);

        std::vector<std::vector<double>> pascal_triangles(N1_, std::vector<double>(N1_, 0.0));
        for (GEO::index_t i = 0; i < N1_; ++i) {
            pascal_triangles[i][0] = 1.0;
            for (GEO::index_t j = 1; j <= i; ++j)
                pascal_triangles[i][j] = pascal_triangles[i-1][j-1] + pascal_triangles[i-1][j];
        }

        for (GEO::index_t i = 0; i < N1_; ++i) {
            const double pow_of_two = std::pow(2.0, i);
            for (GEO::index_t j = 0; j <= i; ++j)
                ML_(i, j) = pascal_triangles[i][j] / pow_of_two;
        }
        for (GEO::index_t i = 0; i < N1_; ++i) {
            const double pow_of_two = std::pow(2.0, n_-i);
            for (GEO::index_t j = i; j < N1_; ++j)
                MR_(i, j) = pascal_triangles[n_-i][j-i] / pow_of_two;
        }
    }

    template<GEO::index_t DIM, typename CONTROL_GRID>
    void MinimumJacobianDeterminant<DIM, CONTROL_GRID>::initialize_priority_queue(
        const GEO::index_t c
        ) {
        /* Clean priority queue */
        std::priority_queue<Block, std::vector<Block>, std::greater<Block>>().swap(pq_);

        /* Compute det(J) values */
        Eigen::VectorXd detJ;
        compute_samples_det_J(c, detJ);

        /* Convert to 3D tensor coefficients */
        Eigen::VectorXd C;
        convert_to_bernstein_coeffs(detJ, C);

        pq_.emplace(C);
    }

    template<GEO::index_t DIM, typename CONTROL_GRID>
    void MinimumJacobianDeterminant<DIM, CONTROL_GRID>::subdivide(
        const Block& block,
        std::vector<Block>& sub_blocks
        ) {
        const auto& C = block.C;

        /* U subdivision */
        auto split_U = [&](const Eigen::MatrixXd& mat) -> std::pair<Eigen::MatrixXd, Eigen::MatrixXd> {
            return {ML_ * mat, MR_ * mat};
        };
        auto [UL, UR] = split_U(Eigen::Map<const Eigen::MatrixXd>(C.data(), N1_, N2_));

        /* V subdivision */
        auto split_V = [&](const Eigen::MatrixXd& mat_u) -> std::pair<Eigen::MatrixXd, Eigen::MatrixXd> {
            Eigen::MatrixXd VL(N1_, N2_), VR(N1_, N2_);
            for(GEO::index_t k = 0; k < N1_; ++k) {
                Eigen::Map<const Eigen::MatrixXd> plane(mat_u.data() + k*N2_, N1_, N1_);
                Eigen::MatrixXd T = plane.transpose();
                VL.block(0, k*N1_, N1_, N1_) = (ML_ * T).transpose();
                VR.block(0, k*N1_, N1_, N1_) = (MR_ * T).transpose();
            }
            return {VL, VR};
        };
        auto [ULVL, ULVR] = split_V(UL);
        auto [URVL, URVR] = split_V(UR);

        std::vector<Eigen::VectorXd> sub_coeffs(CORNER_INDICES_.size());
        if constexpr (std::is_same_v<CONTROL_GRID, QuadControlGrid<2>> || std::is_same_v<CONTROL_GRID, QuadControlGrid<3>>) {
            assert(sub_coeffs.size() == 4);
            sub_coeffs[0] = ULVL;
            sub_coeffs[1] = URVL;
            sub_coeffs[2] = URVR;
            sub_coeffs[3] = ULVR;
        }
        else if constexpr (std::is_same_v<CONTROL_GRID, HexControlGrid>) {
            /* W subdivision */
            auto split_W = [&](const Eigen::MatrixXd& mat_uv) -> std::pair<Eigen::VectorXd, Eigen::VectorXd> {
                const Eigen::Map<const Eigen::MatrixXd> mat_W(mat_uv.data(), N2_, N1_);
                Eigen::MatrixXd resL = mat_W * ML_.transpose();
                Eigen::MatrixXd resR = mat_W * MR_.transpose();
                return {Eigen::Map<Eigen::VectorXd>(resL.data(), N3_),
                        Eigen::Map<Eigen::VectorXd>(resR.data(), N3_)};
            };

            assert(sub_coeffs.size() == 8);
            std::tie(sub_coeffs[0], sub_coeffs[4]) = split_W(ULVL); // 000, 001
            std::tie(sub_coeffs[1], sub_coeffs[5]) = split_W(ULVR); // 010, 011
            std::tie(sub_coeffs[2], sub_coeffs[6]) = split_W(URVL); // 100, 101
            std::tie(sub_coeffs[3], sub_coeffs[7]) = split_W(URVR); // 110, 111
        }
        else
            static_assert(false);

        /* Output */
        sub_blocks.clear();
        sub_blocks.reserve(sub_coeffs.size());
        for (auto & sub_coeff : sub_coeffs)
            sub_blocks.emplace_back(sub_coeff);
        // set uvw
        const double& min_u = block.min_u;
        const double& max_u = block.max_u;
        const double& min_v = block.min_v;
        const double& max_v = block.max_v;
        const double mid_u = 0.5*(min_u+max_u);
        const double mid_v = 0.5*(min_v+max_v);
        if constexpr (std::is_same_v<CONTROL_GRID, QuadControlGrid<2>> || std::is_same_v<CONTROL_GRID, QuadControlGrid<3>>) {
            assert(sub_blocks.size() == 4);
            sub_blocks[0].min_u = min_u;  sub_blocks[0].max_u = mid_u;
            sub_blocks[0].min_v = min_v;  sub_blocks[0].max_v = mid_v;
            sub_blocks[1].min_u = min_u;  sub_blocks[1].max_u = mid_u;
            sub_blocks[1].min_v = mid_v;  sub_blocks[1].max_v = max_v;
            sub_blocks[2].min_u = mid_u;  sub_blocks[2].max_u = max_u;
            sub_blocks[2].min_v = mid_v;  sub_blocks[2].max_v = max_v;
            sub_blocks[3].min_u = mid_u;  sub_blocks[3].max_u = max_u;
            sub_blocks[3].min_v = min_v;  sub_blocks[3].max_v = mid_v;
        }
        else if constexpr (std::is_same_v<CONTROL_GRID, HexControlGrid>) {
            const double& min_w = block.min_w;
            const double& max_w = block.max_w;
            const double mid_w = 0.5*(min_w+max_w);
            assert(sub_blocks.size() == 8);
            sub_blocks[0].min_w = min_w;  sub_blocks[0].max_w = mid_w;
            sub_blocks[1].min_w = min_w;  sub_blocks[1].max_w = mid_w;
            sub_blocks[2].min_w = min_w;  sub_blocks[2].max_w = mid_w;
            sub_blocks[3].min_w = min_w;  sub_blocks[3].max_w = mid_w;
            sub_blocks[4].min_u = min_u;  sub_blocks[4].max_u = mid_u;
            sub_blocks[4].min_v = min_v;  sub_blocks[4].max_v = mid_v;
            sub_blocks[4].min_w = mid_w;  sub_blocks[4].max_w = max_w;
            sub_blocks[5].min_u = min_u;  sub_blocks[5].max_u = mid_u;
            sub_blocks[5].min_v = mid_v;  sub_blocks[5].max_v = max_v;
            sub_blocks[5].min_w = mid_w;  sub_blocks[5].max_w = max_w;
            sub_blocks[6].min_u = mid_u;  sub_blocks[6].max_u = max_u;
            sub_blocks[6].min_v = min_v;  sub_blocks[6].max_v = mid_v;
            sub_blocks[6].min_w = mid_w;  sub_blocks[6].max_w = max_w;
            sub_blocks[7].min_u = mid_u;  sub_blocks[7].max_u = max_u;
            sub_blocks[7].min_v = mid_v;  sub_blocks[7].max_v = max_v;
            sub_blocks[7].min_w = mid_w;  sub_blocks[7].max_w = max_w;
        }
        else
            static_assert(false);
    }

    template class MinimumJacobianDeterminant<2, QuadControlGrid<2>>;
    template class MinimumJacobianDeterminant<3, QuadControlGrid<3>>;
    template class MinimumJacobianDeterminant<3, HexControlGrid>;
}