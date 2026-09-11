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
                for (const auto& sub_block : sub_blocks)
                    pq_.push(sub_block);
            }
        }

        return false;
    }

    template<GEO::index_t DIM, typename CONTROL_GRID>
    double MinimumJacobianDeterminant<DIM, CONTROL_GRID>::compute_lower_bound(
        const GEO::index_t c,
        const double eps,
        std::vector<Block>* travelled_sub_blocks
        ) {
        initialize_priority_queue(c);

        double global_upper_bound = std::numeric_limits<double>::max();
        while (!pq_.empty()) {
            const auto& B = pq_.top();
            for (unsigned int& i : CORNER_INDICES_)
                global_upper_bound = std::min(global_upper_bound, B.C[i]);

            if (travelled_sub_blocks != nullptr) // for debug
                travelled_sub_blocks->push_back(B);

            if (B.min_c > global_upper_bound) {
                /* Do nothing */
                pq_.pop();
            }
            else if (B.max_c - B.min_c < eps) {
                global_upper_bound = B.min_c;
                break;
            }
            else {
                /* Subdivide */
                std::vector<Block> sub_blocks;
                subdivide(B, sub_blocks);

                pq_.pop();
                for (const auto& sub_block : sub_blocks)
                    pq_.push(sub_block);
            }
        }

        return global_upper_bound;
    }

    template<GEO::index_t DIM, typename CONTROL_GRID>
    void MinimumJacobianDeterminant<DIM, CONTROL_GRID>::collect_invalid_sub_blocks(
        const GEO::index_t c,
        std::vector<Block>& invalid_sub_blocks,
        const double eps
        ) {
        initialize_priority_queue(c);

        while (!pq_.empty()) {
            if (const auto& B = pq_.top();
                B.min_c > 0
                ) {
                /* Do nothing */
                pq_.pop();
            }
            else if (B.max_c < 0 || B.max_c - B.min_c < eps) {
                invalid_sub_blocks.push_back(B);
                pq_.pop();
            }
            else {
                /* Subdivide */
                std::vector<Block> sub_blocks;
                subdivide(B, sub_blocks);

                pq_.pop();
                for (const auto& sub_block : sub_blocks)
                    pq_.push(sub_block);
            }
        }
    }

    template<GEO::index_t DIM, typename CONTROL_GRID>
    void MinimumJacobianDeterminant<DIM, CONTROL_GRID>::compute_untangling_funcgrad(
        const GEO::index_t c,
        Eigen::VectorXd& detJ_b_coeffs,
        Eigen::MatrixXd& grad_detJ_b_coeffs
        ) const {
        Eigen::VectorXd detJ;
        compute_samples_det_J(c, detJ);
        convert_to_bernstein_coeffs(detJ, detJ_b_coeffs);

        Eigen::MatrixXd grad_detJ;
        compute_samples_grad_det_J(c, grad_detJ);
        convert_to_bernstein_coeffs(grad_detJ, grad_detJ_b_coeffs);
    }

    template<GEO::index_t DIM, typename CONTROL_GRID>
    void MinimumJacobianDeterminant<DIM, CONTROL_GRID>::append_blocks_to_mesh(
        const std::vector<Block>& blocks,
        GEO::Mesh& mesh_out
        ) const {
        LOG::TRACE("{}({})", __FUNCTION__, blocks.size());

        constexpr GEO::index_t CORNERS_NB = std::is_same_v<CONTROL_GRID, HexControlGrid> ? 8 : 4;
        assert(CORNER_INDICES_.size() == CORNERS_NB);
        constexpr GEO::index_t EDGES_NB = std::is_same_v<CONTROL_GRID, HexControlGrid> ? 12 : 4;

        GEO::Attribute<double> mesh_out_v_mindetJ(mesh_out.vertices.attributes(), "min_det_J");
        const GEO::index_t new_v = mesh_out.vertices.create_vertices(CORNERS_NB*blocks.size());
        const GEO::index_t new_e = mesh_out.edges.create_edges(EDGES_NB*blocks.size());
        for (GEO::index_t i = 0, i_end = blocks.size(); i < i_end; ++i) {
            const auto& B = blocks[i];

            if constexpr (std::is_same_v<CONTROL_GRID, HexControlGrid>) {
                mesh_out.vertices.point(new_v+CORNERS_NB*i)     = GEO::vec3(B.min_u, B.min_v, B.min_w);
                mesh_out.vertices.point(new_v+CORNERS_NB*i+1)   = GEO::vec3(B.max_u, B.min_v, B.min_w);
                mesh_out.vertices.point(new_v+CORNERS_NB*i+2)   = GEO::vec3(B.min_u, B.max_v, B.min_w);
                mesh_out.vertices.point(new_v+CORNERS_NB*i+3)   = GEO::vec3(B.max_u, B.max_v, B.min_w);
                mesh_out.vertices.point(new_v+CORNERS_NB*i+4)   = GEO::vec3(B.min_u, B.min_v, B.max_w);
                mesh_out.vertices.point(new_v+CORNERS_NB*i+5)   = GEO::vec3(B.max_u, B.min_v, B.max_w);
                mesh_out.vertices.point(new_v+CORNERS_NB*i+6)   = GEO::vec3(B.min_u, B.max_v, B.max_w);
                mesh_out.vertices.point(new_v+CORNERS_NB*i+7)   = GEO::vec3(B.max_u, B.max_v, B.max_w);
                mesh_out_v_mindetJ[new_v+CORNERS_NB*i]      = B.C[CORNER_INDICES_[0]];
                mesh_out_v_mindetJ[new_v+CORNERS_NB*i+1]    = B.C[CORNER_INDICES_[1]];
                mesh_out_v_mindetJ[new_v+CORNERS_NB*i+2]    = B.C[CORNER_INDICES_[2]];
                mesh_out_v_mindetJ[new_v+CORNERS_NB*i+3]    = B.C[CORNER_INDICES_[3]];
                mesh_out_v_mindetJ[new_v+CORNERS_NB*i+4]    = B.C[CORNER_INDICES_[4]];
                mesh_out_v_mindetJ[new_v+CORNERS_NB*i+5]    = B.C[CORNER_INDICES_[5]];
                mesh_out_v_mindetJ[new_v+CORNERS_NB*i+6]    = B.C[CORNER_INDICES_[6]];
                mesh_out_v_mindetJ[new_v+CORNERS_NB*i+7]    = B.C[CORNER_INDICES_[7]];
            }
            else {
                mesh_out.vertices.point(new_v+CORNERS_NB*i)     = GEO::vec3(B.min_u, B.min_v, B.min_w);
                mesh_out.vertices.point(new_v+CORNERS_NB*i+1)   = GEO::vec3(B.max_u, B.min_v, B.min_w);
                mesh_out.vertices.point(new_v+CORNERS_NB*i+2)   = GEO::vec3(B.max_u, B.max_v, B.min_w);
                mesh_out.vertices.point(new_v+CORNERS_NB*i+3)   = GEO::vec3(B.min_u, B.max_v, B.min_w);
                mesh_out_v_mindetJ[new_v+CORNERS_NB*i]      = B.C[CORNER_INDICES_[0]];
                mesh_out_v_mindetJ[new_v+CORNERS_NB*i+1]    = B.C[CORNER_INDICES_[1]];
                mesh_out_v_mindetJ[new_v+CORNERS_NB*i+2]    = B.C[CORNER_INDICES_[2]];
                mesh_out_v_mindetJ[new_v+CORNERS_NB*i+3]    = B.C[CORNER_INDICES_[3]];
            }

            if constexpr (std::is_same_v<CONTROL_GRID, HexControlGrid>) {
                for (GEO::index_t le = 0; le < EDGES_NB; ++le) {
                    mesh_out.edges.set_vertex(new_e+EDGES_NB*i+le, 0, new_v+CORNERS_NB*i+geolio::HEX_LE_INCIDENT_LV[le][0]);
                    mesh_out.edges.set_vertex(new_e+EDGES_NB*i+le, 1, new_v+CORNERS_NB*i+geolio::HEX_LE_INCIDENT_LV[le][1]);
                }
            }
            else {
                for (GEO::index_t le = 0; le < EDGES_NB; ++le) {
                    mesh_out.edges.set_vertex(new_e+EDGES_NB*i+le, 0, new_v+CORNERS_NB*i+le);
                    mesh_out.edges.set_vertex(new_e+EDGES_NB*i+le, 1, new_v+CORNERS_NB*i+(le+1)%4);
                }
            }
        }
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

        /* Only the hexahedral path converts (N3_ x N3_) tensor coefficients, and that matrix
         * grows as N1_^6, so it must not be built for the 2D (quad) instantiations. */
        if constexpr (std::is_same_v<CONTROL_GRID, HexControlGrid>)
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
        constexpr bool IS_2D = std::is_same_v<CONTROL_GRID, QuadControlGrid<2>> ||
                               std::is_same_v<CONTROL_GRID, QuadControlGrid<3>>;

        const auto& C = block.C;

        /* The coefficient vector of a 2D block is a single N1_ x N1_ (u, v) plane, while the 3D
         * one stacks N1_ such planes along w. Both are stored with u as the fastest index, so a
         * block is viewed as an N1_ x (N1_*PLANE_NB) matrix whose columns carry the remaining
         * (v, w) index. */
        const GEO::index_t PLANE_NB = IS_2D ? 1 : N1_;

        /* U subdivision */
        auto split_U = [&](const Eigen::MatrixXd& mat) -> std::pair<Eigen::MatrixXd, Eigen::MatrixXd> {
            return {ML_ * mat, MR_ * mat};
        };
        auto [UL, UR] = split_U(Eigen::Map<const Eigen::MatrixXd>(C.data(), N1_, N1_*PLANE_NB));

        /* V subdivision */
        auto split_V = [&](const Eigen::MatrixXd& mat_u) -> std::pair<Eigen::MatrixXd, Eigen::MatrixXd> {
            Eigen::MatrixXd VL(mat_u.rows(), mat_u.cols()), VR(mat_u.rows(), mat_u.cols());
            for(GEO::index_t k = 0; k < PLANE_NB; ++k) {
                Eigen::Map<const Eigen::MatrixXd> plane(mat_u.data() + k*N2_, N1_, N1_);
                Eigen::MatrixXd T = plane.transpose();
                VL.block(0, k*N1_, N1_, N1_) = (ML_ * T).transpose();
                VR.block(0, k*N1_, N1_, N1_) = (MR_ * T).transpose();
            }
            return {VL, VR};
        };
        auto [ULVL, ULVR] = split_V(UL);
        auto [URVL, URVR] = split_V(UR);

        /* Child blocks are indexed by a (u_bit, v_bit, w_bit) triple, the child with a set bit
         * taking the upper half of the parent block along the corresponding axis. The three bits
         * form the flat index u_bit*2 + v_bit + w_bit*4, which is the order in which the
         * coefficients below are assigned. */
        std::vector<Eigen::VectorXd> sub_coeffs(CORNER_INDICES_.size());
        if constexpr (IS_2D) {
            assert(sub_coeffs.size() == 4);
            /* Every child is one N1_ x N1_ plane; flattening it column-major keeps u as the
             * fastest index, which is what split_U / split_V expect on the next subdivision. */
            sub_coeffs[0] = Eigen::Map<const Eigen::VectorXd>(ULVL.data(), N2_);
            sub_coeffs[1] = Eigen::Map<const Eigen::VectorXd>(ULVR.data(), N2_);
            sub_coeffs[2] = Eigen::Map<const Eigen::VectorXd>(URVL.data(), N2_);
            sub_coeffs[3] = Eigen::Map<const Eigen::VectorXd>(URVR.data(), N2_);
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
        /* Set the parametric bounds of every child with the same bit indexing as its
         * coefficients: a clear bit keeps the lower half of the parent block along that axis,
         * a set bit takes the upper half. The w bounds of a 2D block are left untouched. */
        const double lower[3] = {block.min_u, block.min_v, block.min_w};
        const double upper[3] = {block.max_u, block.max_v, block.max_w};
        const double middle[3] = {
            0.5*(block.min_u+block.max_u),
            0.5*(block.min_v+block.max_v),
            0.5*(block.min_w+block.max_w)
        };
        double Block::* const min_bounds[3] = {&Block::min_u, &Block::min_v, &Block::min_w};
        double Block::* const max_bounds[3] = {&Block::max_u, &Block::max_v, &Block::max_w};
        /* A child index is u_bit*2 + v_bit + w_bit*4, so bit 0 carries v and bit 1 carries u. */
        const GEO::index_t axis_of_bit[3] = {1, 0, 2}; // v, u, w
        const GEO::index_t BIT_NB = IS_2D ? 2 : 3;

        for (GEO::index_t i = 0; i < sub_blocks.size(); ++i) {
            for (GEO::index_t b = 0; b < BIT_NB; ++b) {
                const GEO::index_t d = axis_of_bit[b];
                const bool upper_half = ((i >> b) & 1) != 0;
                sub_blocks[i].*min_bounds[d] = upper_half ? middle[d] : lower[d];
                sub_blocks[i].*max_bounds[d] = upper_half ? upper[d]  : middle[d];
            }
        }
    }

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

    template<GEO::index_t DIM, typename CONTROL_GRID>
    void MinimumJacobianDeterminant<DIM, CONTROL_GRID>::compute_samples_grad_det_J(
        const GEO::index_t c,
        Eigen::MatrixXd& grad_det_J
        ) const {
        if constexpr (std::is_same_v<CONTROL_GRID, QuadControlGrid<2>> || std::is_same_v<CONTROL_GRID, QuadControlGrid<3>>) {
            const auto& CONTROL_POINTS_NB = control_grid_.control_nodes_nb_per_facet();
            grad_det_J = Eigen::MatrixXd::Zero(N2_, DIM*CONTROL_POINTS_NB);
            for (GEO::index_t j = 0; j < N1_; ++j) {
                for (GEO::index_t i = 0; i < N1_; ++i) {
                    const GEO::vec2 uv(samples_1D_[i], samples_1D_[j]);
                    std::vector<double> grads;
                    control_grid_.compute_facet_uv_detJ_gradient(c, uv, grads);
                    assert(grads.size() == DIM*CONTROL_POINTS_NB);

                    const GEO::index_t I = i+j*N1_;
                    for (GEO::index_t ii = 0; ii < DIM*CONTROL_POINTS_NB; ++ii)
                        grad_det_J(I, ii) = grads[ii];
                }
            }
        }
        else if constexpr (std::is_same_v<CONTROL_GRID, HexControlGrid>) {
            const auto& CONTROL_POINTS_NB = control_grid_.control_nodes_nb_per_cell();
            grad_det_J = Eigen::MatrixXd::Zero(N3_, 3*CONTROL_POINTS_NB);
            for (GEO::index_t k = 0; k < N1_; ++k) {
                for (GEO::index_t j = 0; j < N1_; ++j) {
                    for (GEO::index_t i = 0; i < N1_; ++i) {
                        const GEO::vec3 uvw(samples_1D_[i], samples_1D_[j], samples_1D_[k]);
                        std::vector<double> grads;
                        control_grid_.compute_cell_uvw_detJ_gradient(c, uvw, grads);
                        assert(grads.size() == 3*CONTROL_POINTS_NB);

                        const GEO::index_t I = i+j*N1_+k*N2_;
                        for (GEO::index_t ii = 0; ii < 3*CONTROL_POINTS_NB; ++ii)
                            grad_det_J(I, ii) = grads[ii];
                    }
                }
            }
        }
        else
            static_assert(false);
    }

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

    template class MinimumJacobianDeterminant<2, QuadControlGrid<2>>;
    template class MinimumJacobianDeterminant<3, QuadControlGrid<3>>;
    template class MinimumJacobianDeterminant<3, HexControlGrid>;
}