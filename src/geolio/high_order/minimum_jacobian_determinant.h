//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/11.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_MINIMUM_JACOBIAN_DETERMINANT_H
#define GEOLIO_MINIMUM_JACOBIAN_DETERMINANT_H
#include <queue>
#include <geogram/mesh/mesh_io.h>
#include <geolio/common/log.h>
#include <geolio/mesh/hex_descriptor.h>
#include <unsupported/Eigen/KroneckerProduct>
#include "control_grid.h"
#include "hex_control_grid.h"
#include "quad_control_grid.h"

namespace geolio
{
    /**
     * Subdivision-based analyzer for the Jacobian determinant of the parametric mapping described by a
     * `ControlGrid`. `CONTROL_GRID` is either a `HexControlGrid`, whose cell is a trivariate (u,v,w)
     * mapping, or a `QuadControlGrid<DIM>`, whose facet is a bivariate (u,v) mapping; for a facet the
     * sampled quantity is `QuadControlGrid::compute_facet_uv_measure(..., MeasureType::DET_JACOBIAN)`,
     * that is the signed area for `DIM == 2` and the non-negative `|du x dv|` for `DIM == 3`.
     *
     * The determinant is sampled on an equispaced tensor-product grid and re-expressed in the Bernstein
     * basis, whose coefficients bound the determinant over a whole parametric block. Blocks are then
     * refined in a best-first order (smallest lower bound first) until each of them can be certified
     * either negative or non-negative, which locates inverted regions without a global minimisation.
     */
    template<typename CONTROL_GRID>
    class MinimumJacobianDeterminant {
        static_assert(isQuadControlGrid<CONTROL_GRID>::value || isHexControlGrid<CONTROL_GRID>::value);
    public:
        /**
         * Create a MinJacobianDet analyzer for the given control grid.
         * @param[in] control_grid Reference to the control grid that defines the polynomial (or spline)
         * coefficients of the mapping. The analyzer holds a reference and does not take ownership.
         */
        explicit MinimumJacobianDeterminant(
            const CONTROL_GRID& control_grid
            ): control_grid_(control_grid),
               ORDER_(control_grid.order())
        {
            if constexpr (isQuadControlGrid<CONTROL_GRID>::value) {
                if (use_absolute_area_)
                    n_ = 4*ORDER_-2;
                else
                    n_ = 2*ORDER_-1;
                N1_ = n_+1;
                N2_ = N1_*N1_;
                CORNER_INDICES_.resize(4);
                CORNER_INDICES_[0] = {0         + 0*N1_      };
                CORNER_INDICES_[1] = {(N1_-1)   + 0*N1_      };
                CORNER_INDICES_[2] = {(N1_-1)   + (N1_-1)*N1_};
                CORNER_INDICES_[3] = {0         + (N1_-1)*N1_};
            }
            else {
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

            samples_1D_.reserve(N1_);
            for (GEO::index_t i = 0; i < N1_; ++i)
                samples_1D_.push_back(static_cast<double>(i)/n_);

            pre_compute_Lagrange_to_Bernstein_matrix();
            pre_compute_subdivision_matrices();
        }

        /**
         * Use the squared absolute area instead of the signed Jacobian determinant for 3D facets.
         *
         * When enabled, subsequent facet evaluations use `|du x dv|^2`. This measure is non-negative,
         * so the analyzer reports zero-area or degenerate facets rather than orientation reversals.
         * This interface is available only when `CONTROL_GRID` is `QuadControlGrid<3>`.
         */
        void use_absolute_area() requires (isQuadControlGrid<CONTROL_GRID>::value && QuadControlGridMeshDim<CONTROL_GRID>::value == 3) { use_absolute_area_ = true; };

        /**
         * Represents an axis-aligned sub-block in the parametric (u,v,w) domain together with interval
         * bounds on the determinant's polynomial coefficients evaluated on that block.
         */
        class Block {
        public:
            /**
             * Construct a block from its Bernstein coefficient vector.
             *
             * @param[in] _C Coefficients of the determinant over the block, flattened with `u` as the
             * fastest index (`i + j*N1_` for a facet grid, `i + j*N1_ + k*N2_` for a hexahedral grid).
             * `min_c` and `max_c` are derived from it and bound the determinant over the block.
             */
            explicit Block(const Eigen::VectorXd& _C) {
                C = _C;
                min_c = _C.minCoeff();
                max_c = _C.maxCoeff();
                if constexpr (isQuadControlGrid<CONTROL_GRID>::value) {
                    min_w = 0;
                    max_w = 0;
                }
            }

            /**
             * Compare blocks by their minimum coefficient bound `min_c` for priority-queue ordering
             * (smaller min_c has higher priority).
             */
            bool operator>(const Block& other) const {
                return min_c > other.min_c;
            }

            double min_u = 0;
            double max_u = 1;
            double min_v = 0;
            double max_v = 1;
            double min_w = 0;
            double max_w = 1;
            double min_c = std::numeric_limits<double>::max();
            double max_c = -std::numeric_limits<double>::max();
            Eigen::VectorXd C;
        };

        /**
         * @brief Test whether the Jacobian determinant of cell / facet `c` is negative somewhere.
         *
         * The parametric domain of `c` is subdivided adaptively and the blocks are examined in a
         * best-first order (smallest lower bound first). The routine returns `true` as soon as inversion
         * is proven: either all coefficients of the leading block are negative (`max_c < 0`), or one of
         * its corner coefficients -- which are exact determinant values -- is negative. It returns
         * `false` once every remaining block is certified non-negative, that is once the determinant is
         * known not to become negative over the whole domain of `c`.
         *
         * @note For a facet grid in 3D the sampled measure `|du x dv|` is non-negative by construction,
         * so a `true` return there means "degenerate / creased facet" rather than an orientation flip.
         *
         * @param[in] c Index of the control-grid cell (hexahedral grid) or facet (quad grid) to test.
         * @param[in] eps Stopping tolerance on the width of a block's coefficient range: a block whose
         * `max_c - min_c` has shrunk below `eps` is no longer subdivided and is decided by the sign of
         * its lower bound. Smaller values keep refining (more accurate, more expensive). Default 1e-10.
         */
        bool contains_inverted_region(
            const GEO::index_t c,
            double eps = 1e-10
            ) {
            initialize_priority_queue(c);

            while (!pq_.empty()) {
                const auto& B = pq_.top();

                for (unsigned int& i : CORNER_INDICES_) {
                    if (B.C[i] < (use_absolute_area_ ? absolute_area_tolerance_ : 0)) // corner values are exact
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

        /**
         * Compute a conservative estimate of the minimum Jacobian determinant over cell `c`.
         *
         * The routine explores the parametric domain through the same subdivision and priority-driven
         * evaluation as `contains_inverted_region()`. The value it returns, `R`, is the smallest lower
         * bound it manages to certify, which brackets the true minimum `m` as `R <= m <= R + eps`: a
         * block is pruned as soon as it cannot beat the best sampled corner value, and the search stops
         * at the first block whose coefficient range has shrunk below `eps`.
         *
         * @param[in] c Index of the control grid cell (hexahedral grid) or facet (quad grid) to analyze.
         * @param[in] eps Stopping tolerance for subdivision (smaller eps -> more accurate bound). Default is 1e-10.
         * @param[out] travelled_sub_blocks Optional pointer to a vector that will be filled with the blocks
         * visited during the search, in the order in which they were examined. Can be nullptr if the caller
         * does not need that information.
         * @return Lower bound `R` on the minimum determinant, tight to `eps`: `R > 0` certifies that the
         * mapping is valid over the whole cell, while `R < -eps` certifies that it is inverted somewhere.
         */
        double compute_lower_bound(
            const GEO::index_t c,
            const double eps = 1e-10,
            std::vector<Block>* travelled_sub_blocks = nullptr
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

        /**
         * Collect sub-blocks of `c` that may contain a negative Jacobian determinant.
         *
         * Adaptive subdivision of the parametric domain of `c` is run and every block that can neither be
         * certified non-negative nor be refined any further is appended to `invalid_sub_blocks`; blocks
         * whose coefficients are all negative are reported directly, without refinement.
         *
         * @param[in] c Index of the control grid cell (hexahedral grid) or facet (quad grid) to examine.
         * @param[out] invalid_sub_blocks Vector that will be filled with the suspect blocks. The caller
         * is responsible for clearing / handling this container.
         * @param[in] eps Stopping tolerance on the width of a block's coefficient range; blocks that
         * cannot be narrowed below it are reported as they are. Larger values are more conservative.
         */
        void collect_invalid_sub_blocks(
            const GEO::index_t c,
            std::vector<Block>& invalid_sub_blocks,
            const double eps = 1e-1
            ) {
            initialize_priority_queue(c);

            while (!pq_.empty()) {
                if (const auto& B = pq_.top();
                    B.min_c > (use_absolute_area_ ? absolute_area_tolerance_ : 0)
                    ) {
                    /* Do nothing */
                    pq_.pop();
                    }
                else if (B.max_c < (use_absolute_area_ ? absolute_area_tolerance_ : 0) || B.max_c - B.min_c < eps) {
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

        /**
         * Compute the Bernstein-basis determinant coefficients and their gradient for cell / facet `c`.
         *
         * `detJ_b_coeffs` stores the determinant coefficients in the flattened `u`-fastest layout used by
         * the rest of this class. `grad_detJ_b_coeffs` stores the gradient samples converted column-wise
         * to the Bernstein basis, with one column per control-point degree of freedom.
         *
         * @param[in] c Index of the control grid cell (hexahedral grid) or facet (quad grid) to analyze.
         * @param[out] detJ_b_coeffs Output vector receiving the determinant coefficients in Bernstein basis.
         * @param[out] grad_detJ_b_coeffs Output matrix receiving the determinant-gradient coefficients in
         * Bernstein basis.
         */
        void compute_untangling_funcgrad(
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

        /**
         * Append block geometry to a Geogram mesh for visualization / debug.
         *
         * Each block is emitted as one element of `mesh_out` whose vertices are its corners in parametric
         * space: eight vertices and twelve edges for a hexahedral grid, four vertices and four edges laid
         * out in the `w = min_w` plane for a facet grid. The determinant at those corners is stored in the
         * `min_det_J` vertex attribute, so that tools can inspect where the suspect sub-blocks are.
         *
         * @param[in] blocks List of blocks to append.
         * @param[in,out] mesh_out Geogram mesh to which block elements will be appended. The mesh is modified in-place.
         */
        void append_blocks_to_mesh(
            const std::vector<Block>& blocks,
            GEO::Mesh& mesh_out
            ) const {
            LOG::TRACE("{}({})", __FUNCTION__, blocks.size());

            constexpr GEO::index_t CORNERS_NB = isQuadControlGrid<CONTROL_GRID>::value ? 4 : 8;
            assert(CORNER_INDICES_.size() == CORNERS_NB);
            constexpr GEO::index_t EDGES_NB = isQuadControlGrid<CONTROL_GRID>::value ? 4 : 12;

            GEO::Attribute<double> mesh_out_v_mindetJ(mesh_out.vertices.attributes(), "min_det_J");
            const GEO::index_t new_v = mesh_out.vertices.create_vertices(CORNERS_NB*blocks.size());
            const GEO::index_t new_e = mesh_out.edges.create_edges(EDGES_NB*blocks.size());
            for (GEO::index_t i = 0, i_end = blocks.size(); i < i_end; ++i) {
                const auto& B = blocks[i];

                if constexpr (isQuadControlGrid<CONTROL_GRID>::value) {
                    mesh_out.vertices.point(new_v+CORNERS_NB*i)     = GEO::vec3(B.min_u, B.min_v, B.min_w);
                    mesh_out.vertices.point(new_v+CORNERS_NB*i+1)   = GEO::vec3(B.max_u, B.min_v, B.min_w);
                    mesh_out.vertices.point(new_v+CORNERS_NB*i+2)   = GEO::vec3(B.max_u, B.max_v, B.min_w);
                    mesh_out.vertices.point(new_v+CORNERS_NB*i+3)   = GEO::vec3(B.min_u, B.max_v, B.min_w);
                    mesh_out_v_mindetJ[new_v+CORNERS_NB*i]      = B.C[CORNER_INDICES_[0]];
                    mesh_out_v_mindetJ[new_v+CORNERS_NB*i+1]    = B.C[CORNER_INDICES_[1]];
                    mesh_out_v_mindetJ[new_v+CORNERS_NB*i+2]    = B.C[CORNER_INDICES_[2]];
                    mesh_out_v_mindetJ[new_v+CORNERS_NB*i+3]    = B.C[CORNER_INDICES_[3]];
                }
                else {
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

                if constexpr (isQuadControlGrid<CONTROL_GRID>::value) {
                    for (GEO::index_t le = 0; le < EDGES_NB; ++le) {
                        mesh_out.edges.set_vertex(new_e+EDGES_NB*i+le, 0, new_v+CORNERS_NB*i+le);
                        mesh_out.edges.set_vertex(new_e+EDGES_NB*i+le, 1, new_v+CORNERS_NB*i+(le+1)%4);
                    }
                }
                else {
                    for (GEO::index_t le = 0; le < EDGES_NB; ++le) {
                        mesh_out.edges.set_vertex(new_e+EDGES_NB*i+le, 0, new_v+CORNERS_NB*i+geolio::HEX_LE_INCIDENT_LV[le][0]);
                        mesh_out.edges.set_vertex(new_e+EDGES_NB*i+le, 1, new_v+CORNERS_NB*i+geolio::HEX_LE_INCIDENT_LV[le][1]);
                    }
                }
            }
        }

    private:
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
        static double combinations(
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
        static double bernstein_basis(
            const GEO::index_t j,
            const GEO::index_t n,
            const double u
            ) {
            return combinations(n, j) * std::pow(u, j) * std::pow(1.0-u, n-j);
        }

        /**
         * Compute and cache the tensor-product Lagrange to Bernstein transforms used by the determinant
         * bounding routines: `M_` (inverse of the Bernstein basis evaluated at `samples_1D_`), the
         * Kronecker products `M2D_ = M_ \otimes M_` and -- for a hexahedral grid only -- `M3D_`, together
         * with their transposes `MT_` and `M2D_T_`.
         */
        void pre_compute_Lagrange_to_Bernstein_matrix(
            ) {
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
            if constexpr (isHexControlGrid<CONTROL_GRID>::value)
                M3D_ = kroneckerProduct(M_, M2D_).eval();
        }

        /**
         * Precompute matrices used to subdivide coefficient representations when splitting blocks.
         *
         * `ML_` and `MR_` map the Bernstein coefficients of a polynomial on [0,1] to the coefficients of
         * its restriction to [0,1/2] and [1/2,1] respectively, along a single axis.
         */
        void pre_compute_subdivision_matrices(
            ) {
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

        /**
         * Reset the search state and seed it with the whole parametric domain of cell / facet `c`: the
         * determinant is sampled (`compute_samples_det_J()`), converted to Bernstein coefficients
         * (`convert_to_bernstein_coeffs()`) and pushed as the single root `Block` of the priority queue.
         *
         * @param[in] c Index of the control grid cell (hexahedral grid) or facet (quad grid).
         */
        void initialize_priority_queue(
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

        /**
         * Subdivide `block` into its axis-aligned children and fill `sub_blocks` with them; internal helper
         * used by the subdivision search.
         *
         * A block produces four children for a facet grid and eight for a hexahedral grid, indexed by the
         * (u_bit, v_bit, w_bit) triple of the half they take along each axis, flattened as
         * `u_bit*2 + v_bit + w_bit*4`. `sub_blocks[i]` therefore carries both the coefficients and the
         * parametric bounds of the same child.
         *
         * @param[in] block Parent block to subdivide.
         * @param[out] sub_blocks Vector replaced by the children (four or eight of them).
         */
        void subdivide(
            const Block& block, std::vector<Block>& sub_blocks
            ) {
            const auto& C = block.C;

            /* The coefficient vector of a 2D block is a single N1_ x N1_ (u, v) plane, while the 3D
             * one stacks N1_ such planes along w. Both are stored with u as the fastest index, so a
             * block is viewed as an N1_ x (N1_*PLANE_NB) matrix whose columns carry the remaining
             * (v, w) index. */
            const GEO::index_t PLANE_NB = isQuadControlGrid<CONTROL_GRID>::value ? 1 : N1_;

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
            if constexpr (isQuadControlGrid<CONTROL_GRID>::value) {
                assert(sub_coeffs.size() == 4);
                /* Every child is one N1_ x N1_ plane; flattening it column-major keeps u as the
                 * fastest index, which is what split_U / split_V expect on the next subdivision. */
                sub_coeffs[0] = Eigen::Map<const Eigen::VectorXd>(ULVL.data(), N2_);
                sub_coeffs[1] = Eigen::Map<const Eigen::VectorXd>(ULVR.data(), N2_);
                sub_coeffs[2] = Eigen::Map<const Eigen::VectorXd>(URVL.data(), N2_);
                sub_coeffs[3] = Eigen::Map<const Eigen::VectorXd>(URVR.data(), N2_);
            }
            else {
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
            const GEO::index_t BIT_NB = isQuadControlGrid<CONTROL_GRID>::value ? 2 : 3;

            for (GEO::index_t i = 0; i < sub_blocks.size(); ++i) {
                for (GEO::index_t b = 0; b < BIT_NB; ++b) {
                    const GEO::index_t d = axis_of_bit[b];
                    const bool upper_half = ((i >> b) & 1) != 0;
                    sub_blocks[i].*min_bounds[d] = upper_half ? middle[d] : lower[d];
                    sub_blocks[i].*max_bounds[d] = upper_half ? upper[d]  : middle[d];
                }
            }
        }

        /**
         * Sample the Jacobian determinant on the tensor-product sample grid of cell / facet `c`.
         *
         * The samples are taken at the equispaced one-dimensional nodes `i / n_` and stored with `u` as
         * the fastest index: the linear index is `i + j * N1_` for a facet grid (N2_ entries) and
         * `i + j * N1_ + k * N2_` for a hexahedral grid (N3_ entries).
         *
         * @param[in] c Index of the control grid cell (hexahedral grid) or facet (quad grid) to evaluate.
         * @param[out] det_J Output vector of sampled determinant values.
         */
        void compute_samples_det_J(
            const GEO::index_t c,
            Eigen::VectorXd& det_J
            ) const {
            if constexpr (isQuadControlGrid<CONTROL_GRID>::value) {
                det_J = Eigen::VectorXd::Zero(N2_);

                for (GEO::index_t j = 0; j < N1_; ++j) {
                    for (GEO::index_t i = 0; i < N1_; ++i) {
                        const GEO::vec2 uv(samples_1D_[i], samples_1D_[j]);
                        if (use_absolute_area_)
                            det_J(i+j*N1_) = control_grid_.compute_facet_uv_measure(c, uv, CONTROL_GRID::MeasureType::ABSOLUTE_SQ_AREA);
                        else
                            det_J(i+j*N1_) = control_grid_.compute_facet_uv_measure(c, uv, CONTROL_GRID::MeasureType::DET_JACOBIAN);
                    }
                }
            }
            else {
                det_J = Eigen::VectorXd::Zero(N3_);

                for (GEO::index_t k = 0; k < N1_; ++k) {
                    for (GEO::index_t j = 0; j < N1_; ++j) {
                        for (GEO::index_t i = 0; i < N1_; ++i) {
                            const GEO::vec3 uvw(samples_1D_[i], samples_1D_[j], samples_1D_[k]);
                            det_J(i+j*N1_+k*N2_) = control_grid_.compute_cell_uvw_measure(c, uvw, CONTROL_GRID::MeasureType::DET_JACOBIAN);
                        }
                    }
                }
            }
        }

        /**
         * Sample the Jacobian-determinant gradient on the tensor-product sample grid of cell / facet `c`.
         *
         * Each row corresponds to one sampled parametric point, flattened with `u` as the fastest index
         * (`i + j * N1_` for a facet, `i + j * N1_ + k * N2_` for a hexahedral cell). Each column
         * corresponds to one control-point degree of freedom in the flattened layout used by
         * `ControlGrid`, so the matrix has `DIM * control_points_nb_per_facet()` columns for a facet grid
         * and `3 * control_points_nb_per_cell()` columns for a hexahedral grid.
         *
         * @note The definition of this member is currently commented out in
         * minimum_jacobian_determinant.cpp, so calling it does not link.
         *
         * @param[in] c Index of the control grid cell (hexahedral grid) or facet (quad grid) to evaluate.
         * @param[out] grad_det_J Output matrix of sampled determinant gradients.
         */
        void compute_samples_grad_det_J(
            const GEO::index_t c,
            Eigen::MatrixXd& grad_det_J
            ) const {
            if constexpr (isQuadControlGrid<CONTROL_GRID>::value) {
                constexpr GEO::index_t DIM = isQuadControlGrid<CONTROL_GRID>::value ? 2 : 3;

                const auto& CONTROL_POINTS_NB = control_grid_.control_nodes_nb_per_facet();
                grad_det_J = Eigen::MatrixXd::Zero(N2_, DIM*CONTROL_POINTS_NB);
                for (GEO::index_t j = 0; j < N1_; ++j) {
                    for (GEO::index_t i = 0; i < N1_; ++i) {
                        const GEO::vec2 uv(samples_1D_[i], samples_1D_[j]);
                        std::vector<double> grads;
                        if (use_absolute_area_)
                            control_grid_.compute_facet_uv_absolute_area_sq_gradient(c, uv, grads);
                        else
                            control_grid_.compute_facet_uv_detJ_gradient(c, uv, grads);
                        assert(grads.size() == DIM*CONTROL_POINTS_NB);

                        const GEO::index_t I = i+j*N1_;
                        for (GEO::index_t ii = 0; ii < DIM*CONTROL_POINTS_NB; ++ii)
                            grad_det_J(I, ii) = grads[ii];
                    }
                }
            }
            else {
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
        }

        /**
         * Convert sampled values / coefficients from the tensor-product Lagrange basis to the Bernstein basis.
         *
         * The transform is `M2D_` for a facet grid and `M3D_` for a hexahedral grid, so `J` and `C` hold
         * N2_ and N3_ entries respectively, in the same `u`-fastest flattened layout as
         * `compute_samples_det_J()`.
         *
         * @param[in] J Input vector in the Lagrange basis.
         * @param[out] C Output vector in the Bernstein basis.
         */
        void convert_to_bernstein_coeffs(
            const Eigen::VectorXd& J,
            Eigen::VectorXd& C
            ) const {
            if constexpr (isQuadControlGrid<CONTROL_GRID>::value)
                C = M2D_ * J;
            else
                C = M3D_ * J;
        }

        /**
         * Convert sampled values / coefficients from the tensor-product Lagrange basis to the Bernstein basis.
         *
         * This overload applies the same transform to every column of `J` at once (`C = M2D_ * J` for a
         * facet grid, `C = M3D_ * J` for a hexahedral grid), for callers that keep several sample vectors
         * side by side.
         *
         * @param[in] J Input matrix in the Lagrange basis.
         * @param[out] C Output matrix in the Bernstein basis.
         */
        void convert_to_bernstein_coeffs(
            const Eigen::MatrixXd& J,
            Eigen::MatrixXd& C
            ) const {
            if constexpr (isQuadControlGrid<CONTROL_GRID>::value)
                C = M2D_ * J;
            else
                C = M3D_ * J;
        }

        bool use_absolute_area_ = false;
        const double absolute_area_tolerance_ = 1e-8;

        const CONTROL_GRID& control_grid_;

        const GEO::index_t ORDER_;
        GEO::index_t n_; // the order of the det(J) == 2/3*ORDER-1
        GEO::index_t N1_ = GEO::NO_INDEX; // n+1
        GEO::index_t N2_ = GEO::NO_INDEX; // N1^2
        GEO::index_t N3_ = GEO::NO_INDEX; // N1^3

        std::vector<GEO::index_t> CORNER_INDICES_; // the corner indices of the flat vector (size == (n+1)^2/3)

        std::vector<double> samples_1D_; // isometric sampling nodes in 1D

        Eigen::MatrixXd M_; // Lagrange to Bernstein matrix (N1*N1)
        Eigen::MatrixXd MT_; // M.transport
        Eigen::MatrixXd M2D_; // M \otimes M (N2*N2)
        Eigen::MatrixXd M2D_T_; // (M \otimes M).transpose
        Eigen::MatrixXd M3D_; // M \otimes M \otimes M (N3*N3)

        Eigen::MatrixXd ML_; // subdivision matrix (N1*N1)
        Eigen::MatrixXd MR_; // subdivision matrix (N1*N1)

        std::priority_queue<Block, std::vector<Block>, std::greater<Block>> pq_;
    };
}

#endif //GEOLIO_MINIMUM_JACOBIAN_DETERMINANT_H
