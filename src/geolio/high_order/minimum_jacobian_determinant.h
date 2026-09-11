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
                if constexpr (std::is_same_v<CONTROL_GRID, QuadControlGrid<2>> || std::is_same_v<CONTROL_GRID, QuadControlGrid<3>>) {
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
        bool contains_inverted_region(GEO::index_t c, double eps = 1e-10);

        /**
         * Compute an estimated conservative upper bound on the minimum Jacobian determinant over cell `c`.
         *
         * The routine explores the parametric domain through the same subdivision and priority-driven
         * evaluation as `contains_inverted_region()` and returns the smallest lower bound it manages to
         * certify, so a negative return value proves that the mapping is inverted somewhere.
         *
         * @note The definition of this member is currently commented out in
         * minimum_jacobian_determinant.cpp, so calling it does not link.
         *
         * @param[in] c Index of the control grid cell (hexahedral grid) or facet (quad grid) to analyze.
         * @param[in] eps Stopping tolerance for subdivision (smaller eps -> more accurate bound). Default is 1e-10.
         * @param[out] travelled_sub_blocks Optional pointer to a vector that will be filled with the blocks
         * visited during the search. Can be nullptr if the caller does not need that information.
         * @return Conservative upper bound on the minimum Jacobian determinant over the cell: a negative
         * value proves inversion, a non-negative value cannot certify positivity on its own.
         */
        double compute_upper_bound(GEO::index_t c, double eps = 1e-10, std::vector<Block>* travelled_sub_blocks = nullptr);

        /**
         * Collect sub-blocks of `c` that may contain a negative Jacobian determinant.
         *
         * Adaptive subdivision of the parametric domain of `c` is run and every block that can neither be
         * certified non-negative nor be refined any further is appended to `invalid_sub_blocks`; blocks
         * whose coefficients are all negative are reported directly, without refinement.
         *
         * @note The definition of this member is currently commented out in
         * minimum_jacobian_determinant.cpp, so calling it does not link.
         *
         * @param[in] c Index of the control grid cell (hexahedral grid) or facet (quad grid) to examine.
         * @param[out] invalid_sub_blocks Vector that will be filled with the suspect blocks. The caller
         * is responsible for clearing / handling this container.
         * @param[in] eps Stopping tolerance on the width of a block's coefficient range; blocks that
         * cannot be narrowed below it are reported as they are. Larger values are more conservative.
         */
        void collect_invalid_sub_blocks(GEO::index_t c, std::vector<Block>& invalid_sub_blocks, double eps = 1e-1);

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
        void compute_samples_det_J(GEO::index_t c, Eigen::VectorXd& det_J) const;

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
        void compute_samples_grad_det_J(GEO::index_t c, Eigen::MatrixXd& grad_det_J) const;

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
        void convert_to_bernstein_coeffs(const Eigen::VectorXd& J, Eigen::VectorXd& C) const;

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
        void convert_to_bernstein_coeffs(const Eigen::MatrixXd& J, Eigen::MatrixXd& C) const;

        /**
         * Get the number of sampling nodes of the Jacobian-detector.
         *
         * @return N1_^3 for a hexahedral grid. Note that for a facet (quad) grid the sampled vector only
         * holds N1_^2 entries, so prefer the size of the vector filled by `compute_samples_det_J()` there.
         */
        [[nodiscard]] auto samples_nb() const { return N3_; }

        /**
         * Append block geometry to a Geogram mesh for visualization / debug.
         *
         * Each block is emitted as one element of `M_out` (eight vertices and twelve edges, following the
         * hexahedral layout of HEX_LE_INCIDENT_LV) whose vertices are the block corners in parametric
         * space, and the determinant at those corners is stored in the `min_det_J` vertex attribute, so
         * that tools can inspect where the invalid sub-blocks are.
         *
         * @note The definition of this member is currently commented out in
         * minimum_jacobian_determinant.cpp, so calling it does not link. Only the hexahedral layout is
         * implemented there; a facet grid would need the four-vertex quad layout instead.
         *
         * @param[in] blocks List of blocks to append.
         * @param[in,out] mesh_out Geogram mesh to which block elements will be appended. The mesh is modified in-place.
         */
        void append_blocks_to_mesh(const std::vector<Block>& blocks, GEO::Mesh& mesh_out) const;

    private:
        /**
         * Compute and cache the tensor-product Lagrange to Bernstein transforms used by the determinant
         * bounding routines: `M_` (inverse of the Bernstein basis evaluated at `samples_1D_`), the
         * Kronecker products `M2D_ = M_ \otimes M_` and -- for a hexahedral grid only -- `M3D_`, together
         * with their transposes `MT_` and `M2D_T_`.
         */
        void pre_compute_Lagrange_to_Bernstein_matrix();

        /**
         * Precompute matrices used to subdivide coefficient representations when splitting blocks.
         *
         * `ML_` and `MR_` map the Bernstein coefficients of a polynomial on [0,1] to the coefficients of
         * its restriction to [0,1/2] and [1/2,1] respectively, along a single axis.
         */
        void pre_compute_subdivision_matrices();

        /**
         * Reset the search state and seed it with the whole parametric domain of cell / facet `c`: the
         * determinant is sampled (`compute_samples_det_J()`), converted to Bernstein coefficients
         * (`convert_to_bernstein_coeffs()`) and pushed as the single root `Block` of the priority queue.
         *
         * @param[in] c Index of the control grid cell (hexahedral grid) or facet (quad grid).
         */
        void initialize_priority_queue(GEO::index_t c);

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
        void subdivide(const Block& block, std::vector<Block>& sub_blocks);

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
