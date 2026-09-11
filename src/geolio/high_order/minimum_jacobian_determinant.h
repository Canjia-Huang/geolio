//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/11.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_MINIMUM_JACOBIAN_DETERMINANT_H
#define GEOLIO_MINIMUM_JACOBIAN_DETERMINANT_H
#include "control_grid.h"
#include <Eigen/Dense>
#include "hex_control_grid.h"
#include "quad_control_grid.h"

namespace geolio
{
    /**
     * Utility to estimate and certify lower bounds for the Jacobian determinant of a trivariate (hexahedral)
     * parametric mapping defined by a `ControlGrid`. The class uses interval / subdivision-based techniques to
     * find sub-blocks of the parametric domain where the determinant may drop below a tolerance and to compute
     * conservative upper bounds on the minimum determinant.
     */
    template<GEO::index_t DIM, typename CONTROL_GRID>
    class MinimumJacobianDeterminant {
    public:
        /**
         * Create a MinJacobianDet analyzer for the given control grid.
         * @param[in] control_grid Reference to the control grid that defines the polynomial (or spline)
         * coefficients of the mapping. The analyzer holds a reference and does not take ownership.
         */
        explicit MinimumJacobianDeterminant(const CONTROL_GRID& control_grid);

        /**
         * Represents an axis-aligned sub-block in the parametric (u,v,w) domain together with interval
         * bounds on the determinant's polynomial coefficients evaluated on that block.
         */
        class Block {
        public:
            /**
             * Construct a block with the given coefficient vector.
             * @param[in] _C Polynomial / coefficient vector associated with the block (used to bound
             * the determinant over the block).
             */
            explicit Block(const Eigen::VectorXd& _C) {
                C = _C;
                min_c = _C.minCoeff();
                max_c = _C.maxCoeff();
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
         * @brief Fast conservative validity test for the Jacobian determinant on a cell.
         *
         * Perform a low-cost, conservative check to decide whether the Jacobian determinant
         * of the parametric mapping associated with control-grid cell `c` is strictly greater
         * than `eps` over the entire parametric domain of the cell. This method is intended
         * as an early-out verifier: a return value of `true` guarantees that the cell is safe
         * (no determinant values <= eps), while `false` indicates that the test could not
         * certify positivity and that a more accurate (and potentially expensive) check is
         * required (for example `compute_upper_bound()` or adaptive subdivision).
         *
         * @param[in] c Index of the control-grid facet/cell to test.
         * @param[in] eps Determinant threshold. Values <= eps are considered invalid. Default is 1e-10.
         * @return `true` if the test certifies det(J) > eps everywhere on the cell; `false` otherwise.
         */
        bool check_inverse(GEO::index_t c, double eps = 1e-10);

        /**
         * Compute an estimated conservative upper bound on the minimum Jacobian determinant over cell `c`.
         *
         * The routine explores the parametric domain (typically via subdivision and priority-driven evaluation)
         * to produce an upper bound on the minimum determinant. The returned value can be used to quickly accept
         * or reject a cell wrt. a positivity tolerance.
         *
         * @param[in] c Index of the control grid cell to analyze.
         * @param[in] eps Stopping tolerance for subdivision (smaller eps -> more accurate bound). Default is 1e-10.
         * @param[out] travelled_sub_blocks Optional pointer to a vector that will be filled with the blocks
         * visited during the search. Can be nullptr if the caller does not need that information.
         * @return Estimated (conservative) upper bound on the minimum Jacobian determinant over the cell.
         * A value <= eps indicates the cell may contain inverted or near-degenerate mappings.
         */
        double compute_upper_bound(GEO::index_t c, double eps = 1e-10, std::vector<Block>* travelled_sub_blocks = nullptr);

        /**
         * Collect sub-blocks of cell `c` where the Jacobian determinant may be below `eps`.
         *
         * This function performs adaptive subdivision of the parametric domain for cell `c` and pushes blocks that
         * are potentially invalid (their conservative lower bound is <= eps) into `invalid_sub_blocks`.
         *
         * @param[in] c Index of the control grid cell to examine.
         * @param[out] invalid_sub_blocks Vector that will be filled with the blocks that could contain determinant
         * values <= eps. The caller is responsible for clearing / handling this container.
         * @param[in] eps Determinant threshold below which a block is considered invalid (default 1e-1).
         * Larger values yield more conservative detection.
         */
        void collect_invalid_sub_blocks(GEO::index_t c, std::vector<Block>& invalid_sub_blocks, double eps = 1e-1);

        /**
         * Append block geometry to a Geogram mesh for visualization / debug.
         * Each block is converted to a hexahedral element in `M_out` so that tools can inspect the locations of
         * invalid sub-blocks.
         *
         * @param[in] blocks List of blocks to append.
         * @param[in,out] M_out Geogram mesh to which block elements will be appended. The mesh is modified in-place.
         */
        void append_blocks_to_mesh(const std::vector<Block>& blocks, GEO::Mesh& M_out) const;

        /**
         * Sample the Jacobian determinant on the pre-defined 3D tensor-product sample grid of cell `c`.
         *
         * The output vector is indexed in flattened (i, j, k) order, where the linear index is
         * `i + j * N1_ + k * N2_`.
         *
         * @param[in] c Index of the control grid cell to evaluate.
         * @param[out] det_J Output vector of sampled determinant values.
         */
        void compute_samples_det_J(GEO::index_t c, Eigen::VectorXd& det_J) const;

        /**
         * Sample the Jacobian-determinant gradient on the pre-defined 3D tensor-product sample grid of cell `c`.
         *
         * Each row corresponds to one sampled parametric point, flattened in `(i, j, k)` order as
         * `i + j * N1_ + k * N2_`. Each column corresponds to one control-point degree of freedom
         * in the flattened `(x, y, z)` layout used by `ControlGrid`.
         *
         * @param[in] c Index of the control grid cell to evaluate.
         * @param[out] grad_det_J Output matrix of sampled determinant gradients.
         *                        have `samples_nb` rows and `3 * control_grid_.control_points_nb_per_cell()` columns.
         */
        void compute_samples_grad_det_J(GEO::index_t c, Eigen::MatrixXd& grad_det_J) const;

        /**
         * Convert sampled values / coefficients from the tensor-product Lagrange basis to the Bernstein basis.
         *
         * @param[in] J Input vector in the Lagrange basis.
         * @param[out] C Output vector in the Bernstein basis.
         */
        void convert_to_bernstein_coeffs(const Eigen::VectorXd& J, Eigen::VectorXd& C) const;

        /**
         * Convert sampled values / coefficients from the tensor-product Lagrange basis to the Bernstein basis.
         *
         * This overload performs the conversion column-wise for matrix data.
         *
         * @param[in] J Input matrix in the Lagrange basis.
         * @param[out] C Output matrix in the Bernstein basis.
         */
        void convert_to_bernstein_coeffs(const Eigen::MatrixXd& J, Eigen::MatrixXd& C) const;

        /**
         * Get the number of 3D sampling nodes per axis used by the Jacobian-detector.
         */
        [[nodiscard]] auto samples_nb() const { return N3_; }

    private:
        /**
         * Compute and cache the matrix that converts tensor-product Lagrange coefficients to the Bernstein basis.
         * This is an internal precomputation step used by the determinant bounding routines.
         */
        void pre_compute_Lagrange_to_Bernstein_matrix();

        /**
         * Precompute matrices used to subdivide coefficient representations when splitting blocks.
         *
         * The resulting matrices are used to split a block's Bernstein coefficients into the coefficients of
         * its child blocks during adaptive subdivision.
         */
        void pre_compute_subdivision_matrices();

        /**
         * Initialize internal state for analyzing cell `c` (e.g. build the initial block, push it to the priority
         * queue, compute coefficient vectors).
         * @param[in] c Index of the control grid cell.
         */
        void initialize_priority_queue(GEO::index_t c);

        /**
         * Subdivide `block` into smaller axis-aligned child blocks and fill `sub_blocks` with the resulting children.
         * Internal helper used by the subdivision search.
         *
         * @param[in] block Parent block to subdivide.
         * @param[out] sub_blocks Vector to which the child blocks will be
         * appended.
         */
        void subdivide(
            const Block& block,
            std::vector<Block>& sub_blocks);

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

    extern template class MinimumJacobianDeterminant<2, QuadControlGrid<2>>;
    extern template class MinimumJacobianDeterminant<3, QuadControlGrid<3>>;
    extern template class MinimumJacobianDeterminant<3, HexControlGrid>;
}

#endif //GEOLIO_MINIMUM_JACOBIAN_DETERMINANT_H
