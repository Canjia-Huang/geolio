//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/11.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
// Tests of MinimumJacobianDeterminant, on both families of control grids it supports: a pair of
// hexahedral cells and a pair of quadrilateral facets (2D and 3D). Every case holds one element
// with an inverted region and one valid element, and checks the three analysis entry points
// (contains_inverted_region, compute_lower_bound, collect_invalid_sub_blocks) against each other
// and against determinants evaluated directly on the control grid. The visited and suspect
// sub-blocks are dumped as .geogram artifacts named after the running test.
//
// compute_untangling_funcgrad is checked separately: the Bernstein coefficients it returns for the
// determinant and for its gradient must reproduce the determinant and the gradient sampled on the
// control grid, both at the corners of the element -- where a Bernstein coefficient is exactly the
// value of the represented function -- and on random parametric points, through an independent
// Bernstein basis implementation.
//
#include <geolio/high_order/minimum_jacobian_determinant.h>
#include <gtest/gtest.h>
#include "control_grid_test_utils.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

namespace geolio::test
{
    namespace
    {
        /* == shared data ====================================================================================== */

        /** Resolution of the parametric grids used to sample the determinant for reference. */
        constexpr GEO::index_t SAMPLE_RESOLUTION = 10;

        /** Stopping tolerance of the half-width of a block's coefficient range. */
        constexpr double LOWER_BOUND_EPS = 1e-10;

        /**
         * Stopping tolerance used when collecting suspect sub-blocks.
         *
         * A coarse value keeps the reported blocks, and the artifacts dumped from them, small:
         * refining further multiplies the block count by orders of magnitude.
         */
        constexpr double INVALID_BLOCK_EPS = 1.0;

        /**
         * Tolerance on the determinant stored on the corners of the dumped sub-blocks.
         *
         * The corner values are Bernstein coefficients, obtained from the sampled determinant
         * through dense Lagrange-to-Bernstein transforms, so they carry the rounding error of those
         * products (of the order of 1e-7 for a hexahedral grid, much less for a facet).
         */
        constexpr double BLOCK_CORNER_TOL = 1e-6;

        /** Slack of the "the certified bound never exceeds a sampled determinant" check. */
        constexpr double BOUND_SLACK = 1e-9;

        /** Number of random parametric points used to reconstruct the Bernstein representations. */
        constexpr GEO::index_t RECONSTRUCTION_SAMPLE_NB = 20;

        /**
         * Tolerance on a determinant-gradient coefficient checked against the gradient sampled
         * directly on the control grid.
         *
         * The corner coefficient of the Bernstein representation of one gradient component is
         * exactly that component at the corner: only the off-diagonal entries of the inverse
         * Lagrange-to-Bernstein matrices contribute to it, and they carry the rounding of those
         * transforms, so the measured error stays around machine precision.
         */
        constexpr double BERNSTEIN_GRADIENT_TOL = 1e-9;

        /**
         * Tolerance on the determinant / gradient reconstructed from the Bernstein coefficients at
         * an arbitrary parametric point.
         *
         * The reconstruction runs the Lagrange-to-Bernstein transforms of the analyzer backwards,
         * which amplifies, by the conditioning of those matrices, the rounding of the sampled
         * values. The measured error is of the order of 1e-7 for both families of grids, so this
         * bound leaves one order of magnitude of headroom while still catching a wrong coefficient.
         */
        constexpr double BERNSTEIN_RECONSTRUCTION_TOL = 1e-5;

        /**
         * Tolerance on the determinant / gradient reconstructed from the Bernstein coefficients of
         * the squared absolute area.
         *
         * That measure has degree 4*order - 2, so the analyzer samples it at 4*order - 1 nodes per
         * direction, where the equispaced Lagrange-to-Bernstein transform is far more ill-
         * conditioned: the coefficients then carry an error of the order of 1e-4, which the
         * reconstruction reproduces.
         */
        constexpr double ABSOLUTE_AREA_RECONSTRUCTION_TOL = 1e-3;

        /* == shared helpers =================================================================================== */

        /**
         * @brief Parametric point carried by the corner of a dumped sub-block.
         * @tparam Sample Parametric sample type, GEO::vec2 or GEO::vec3.
         * @param[in] point Corner of a dumped block, as stored in the mesh (facet blocks have z = 0).
         * @return The parametric point of that corner.
         */
        template <typename Sample>
        [[nodiscard]] Sample parametric_point_of(const GEO::vec3& point) {
            if constexpr (std::is_same_v<Sample, GEO::vec2>)
                return Sample(point.x, point.y);
            else
                return Sample(point.x, point.y, point.z);
        }

        /**
         * @brief Dump a list of sub-blocks as an artifact.
         * @tparam Grid Control-grid type.
         * @param[in] mjd Analyzer that owns the block geometry.
         * @param[in] blocks Blocks to dump; nothing is written when the list is empty.
         * @param[in] suffix Suffix of the artifact file name.
         */
        template <typename Grid>
        void save_blocks(
            const MinimumJacobianDeterminant<Grid>& mjd,
            const std::vector<typename MinimumJacobianDeterminant<Grid>::Block>& blocks,
            const std::string_view suffix
            ) {
            if (blocks.empty())
                return;

            GEO::Mesh mesh_out;
            mjd.append_blocks_to_mesh(blocks, mesh_out);
            EXPECT_TRUE(mesh_out.save(artifact_path(suffix)));
        }

        /**
         * @brief Check the determinant stored on the corners of a list of sub-blocks.
         *
         * `append_blocks_to_mesh` stores the Bernstein coefficient of every block corner, which is
         * an exact value of the determinant at that parametric point: it must match a direct
         * evaluation of the control grid.
         *
         * @tparam Grid Control-grid type.
         * @tparam Sample Parametric sample type, GEO::vec2 or GEO::vec3.
         * @param[in] mjd Analyzer that owns the block geometry.
         * @param[in] blocks Blocks to check.
         * @param[in] det_jacobian_at Evaluates the determinant at one parametric point.
         */
        template <typename Grid, typename Sample, typename EvaluateFn>
        void expect_block_corners_are_exact_determinants(
            const MinimumJacobianDeterminant<Grid>& mjd,
            const std::vector<typename MinimumJacobianDeterminant<Grid>::Block>& blocks,
            EvaluateFn&& det_jacobian_at
            ) {
            if (blocks.empty())
                return;

            GEO::Mesh mesh_out;
            mjd.append_blocks_to_mesh(blocks, mesh_out);

            GEO::Attribute<double> min_det_J(mesh_out.vertices.attributes(), "min_det_J");
            ASSERT_TRUE(min_det_J.is_bound());

            for (const auto v : mesh_out.vertices) {
                const auto point = parametric_point_of<Sample>(mesh_out.vertices.point(v));
                EXPECT_NEAR(min_det_J[v], det_jacobian_at(point), BLOCK_CORNER_TOL)
                    << "block corner at " << parametric_context("p", point);
            }
        }

        /**
         * @brief Check the analysis of one element through all three entry points, and dump it.
         *
         * The three routines explore the same subdivision, so they must conclude alike: on an
         * inverted element the certified lower bound is negative and suspect sub-blocks are
         * reported, whereas on a valid element the bound is positive and none is reported. The
         * bound must also stay below every determinant sampled directly on the element, since it
         * brackets the true minimum from below.
         *
         * @tparam Grid Control-grid type.
         * @tparam Sample Parametric sample type, GEO::vec2 for a facet and GEO::vec3 for a hexahedron.
         * @param[in,out] mjd Analyzer under test.
         * @param[in] element Cell (hexahedral grid) or facet (quad grid) index.
         * @param[in] expect_inverted Whether the element is known to contain an inverted region.
         * @param[in] det_jacobian_at Evaluates the determinant at one parametric point.
         * @param[in] artifact_prefix Prefix of the dumped artifact file names, e.g. "c0", which
         *            yields "<test name>_c0_travelled_blocks.geogram".
         */
        template <typename Grid, typename Sample, typename EvaluateFn>
        void expect_analysis_consistent(
            MinimumJacobianDeterminant<Grid>& mjd,
            const GEO::index_t element,
            const bool expect_inverted,
            EvaluateFn&& det_jacobian_at,
            const std::string_view artifact_prefix
            ) {
            using Block = typename MinimumJacobianDeterminant<Grid>::Block;

            std::vector<Block> travelled_blocks;
            const double lower_bound = mjd.compute_lower_bound(element, LOWER_BOUND_EPS, &travelled_blocks);

            std::vector<Block> invalid_blocks;
            mjd.collect_invalid_sub_blocks(element, invalid_blocks, INVALID_BLOCK_EPS);

            SCOPED_TRACE(::testing::Message() << "element " << element);

            EXPECT_EQ(mjd.contains_inverted_region(element), expect_inverted);
            EXPECT_FALSE(travelled_blocks.empty());

            if (expect_inverted) {
                EXPECT_LT(lower_bound, 0.0);
                EXPECT_FALSE(invalid_blocks.empty());
            }
            else {
                EXPECT_GT(lower_bound, 0.0);
                EXPECT_TRUE(invalid_blocks.empty());
            }

            double sampled_min = std::numeric_limits<double>::max();
            if constexpr (std::is_same_v<Sample, GEO::vec2>) {
                for (const auto& point : grid_unit_samples_2d(SAMPLE_RESOLUTION))
                    sampled_min = std::min(sampled_min, det_jacobian_at(point));
            }
            else {
                for (const auto& point : grid_unit_samples_3d(SAMPLE_RESOLUTION))
                    sampled_min = std::min(sampled_min, det_jacobian_at(point));
            }
            EXPECT_LE(lower_bound, sampled_min + BOUND_SLACK);

            expect_block_corners_are_exact_determinants<Grid, Sample>(mjd, travelled_blocks, det_jacobian_at);
            expect_block_corners_are_exact_determinants<Grid, Sample>(mjd, invalid_blocks, det_jacobian_at);

            save_blocks(mjd, travelled_blocks, "_" + std::string(artifact_prefix) + "_travelled_blocks.geogram");
            save_blocks(mjd, invalid_blocks, "_" + std::string(artifact_prefix) + "_invalid_blocks.geogram");
        }

        /* == untangling function gradient ===================================================================== */

        /**
         * @brief Degree of the sampled determinant along one parametric direction.
         * @tparam Grid Control-grid type.
         * @param[in] order Polynomial order of the control grid.
         * @param[in] absolute_area Whether the sampled measure is the squared absolute area.
         * @return 2*order - 1 for a facet grid under the signed determinant and 3*order - 1 for a
         *         hexahedral grid; 4*order - 2 for a facet grid under the squared absolute area.
         */
        template <typename Grid>
        [[nodiscard]] constexpr GEO::index_t determinant_degree(const GEO::index_t order, const bool absolute_area = false) {
            if (absolute_area)
                return 4*order - 2;
            return (std::is_same_v<Grid, QuadControlGrid> ? 2 : 3)*order - 1;
        }

        /**
         * @brief Evaluate the Bernstein basis polynomial B^n_j(u).
         *
         * Independent reference implementation, used to rebuild the analyzed polynomial from the
         * coefficients returned by `compute_untangling_funcgrad`.
         *
         * @param[in] j Basis index (0 <= j <= n).
         * @param[in] n Degree of the polynomial.
         * @param[in] u Evaluation parameter in [0, 1].
         * @return B^n_j(u) = C(n, j) * u^j * (1-u)^(n-j).
         */
        [[nodiscard]] inline double bernstein_basis(const GEO::index_t j, const GEO::index_t n, const double u) {
            double binomial = 1.0;
            for (GEO::index_t i = 1; i <= j; ++i)
                binomial *= static_cast<double>(n-j+i)/static_cast<double>(i);
            return binomial*std::pow(u, static_cast<double>(j))*std::pow(1.0-u, static_cast<double>(n-j));
        }

        /**
         * @brief Evaluate a tensor-product Bernstein representation at one parametric point.
         *
         * The coefficients are addressed in the flattened `u`-fastest layout of the analyzer,
         * `i + j*N1 + k*N1*N1`, where `k` is always zero for a bivariate representation.
         *
         * @tparam Sample Parametric sample type, GEO::vec2 or GEO::vec3.
         * @tparam CoeffAt Callable returning the coefficient of the (i, j, k)-th control point.
         * @param[in] n Bernstein degree along each parametric direction.
         * @param[in] point Parametric point the coefficients belong to.
         * @param[in] coeff_at Coefficient accessor.
         * @return Value of the represented function at @p point.
         */
        template <typename Sample, typename CoeffAt>
        [[nodiscard]] double evaluate_bernstein_tensor_product(
            const GEO::index_t n,
            const Sample& point,
            CoeffAt&& coeff_at
            ) {
            const GEO::index_t N1 = n+1;
            std::vector<double> Bu(N1), Bv(N1), Bw(N1, 0.0);
            for (GEO::index_t i = 0; i < N1; ++i) {
                Bu[i] = bernstein_basis(i, n, point[0]);
                Bv[i] = bernstein_basis(i, n, point[1]);
                if constexpr (Sample::dim == 3)
                    Bw[i] = bernstein_basis(i, n, point[2]);
            }
            const GEO::index_t PLANE_NB = (Sample::dim == 2) ? 1 : N1;

            double value = 0;
            for (GEO::index_t k = 0; k < PLANE_NB; ++k) {
                for (GEO::index_t j = 0; j < N1; ++j) {
                    for (GEO::index_t i = 0; i < N1; ++i) {
                        if constexpr (Sample::dim == 2)
                            value += Bu[i]*Bv[j]*coeff_at(i, j, 0);
                        else
                            value += Bu[i]*Bv[j]*Bw[k]*coeff_at(i, j, k);
                    }
                }
            }
            return value;
        }

        /**
         * @brief Generate random parametric points of an element.
         * @tparam Sample Parametric sample type, GEO::vec2 or GEO::vec3.
         * @param[in] sample_nb Number of samples.
         * @return @p sample_nb points uniformly distributed in the parametric domain of an element.
         */
        template <typename Sample>
        [[nodiscard]] std::vector<Sample> random_unit_samples(const GEO::index_t sample_nb) {
            if constexpr (Sample::dim == 2)
                return random_unit_samples_2d(sample_nb);
            else
                return random_unit_samples_3d(sample_nb);
        }

        /**
         * @brief Flat coefficient index and parametric point of every corner of an element.
         *
         * For a Bernstein polynomial the coefficient of a corner of the domain is exactly the value
         * of the polynomial there, so these pairs let the returned coefficients be checked without
         * evaluating any basis function.
         *
         * @tparam Sample Parametric sample type, GEO::vec2 or GEO::vec3.
         * @param[in] N1 Number of coefficients along one parametric direction.
         * @return One (flat index, parametric point) pair per corner of the element.
         */
        template <typename Sample>
        [[nodiscard]] std::vector<std::pair<GEO::index_t, Sample>> element_corners(const GEO::index_t N1) {
            const GEO::index_t last = N1-1;
            const GEO::index_t K_NB = (Sample::dim == 2) ? 1 : 2;

            std::vector<std::pair<GEO::index_t, Sample>> corners;
            corners.reserve(K_NB*4);
            for (GEO::index_t kh = 0; kh < K_NB; ++kh) {
                for (GEO::index_t jh = 0; jh < 2; ++jh) {
                    for (GEO::index_t ih = 0; ih < 2; ++ih) {
                        const GEO::index_t i = ih*last;
                        const GEO::index_t j = jh*last;
                        const GEO::index_t k = kh*last;
                        const double u = static_cast<double>(i)/static_cast<double>(last);
                        const double v = static_cast<double>(j)/static_cast<double>(last);
                        const double w = static_cast<double>(k)/static_cast<double>(last);
                        if constexpr (Sample::dim == 2)
                            corners.emplace_back(i + j*N1, Sample(u, v));
                        else
                            corners.emplace_back(i + j*N1 + k*N1*N1, Sample(u, v, w));
                    }
                }
            }
            return corners;
        }

        /**
         * @brief Check `compute_untangling_funcgrad` against the determinant and its gradient.
         *
         * The routine must return the Bernstein coefficients of the sampled determinant and of its
         * gradient with respect to the local control points of the element; three properties are
         * checked:
         *  - the coefficient of a corner of the element is exactly the determinant (resp. the
         *    gradient) evaluated at that corner, a Bernstein representation being interpolatory
         *    there;
         *  - reconstructing the determinant from the returned coefficients at random parametric
         *    points, with an independent Bernstein basis, reproduces the determinant sampled
         *    directly on the control grid;
         *  - reconstructing every column of the gradient coefficients the same way reproduces the
         *    gradient returned by the control grid.
         *
         * @tparam Grid Control-grid type.
         * @tparam Sample Parametric sample type, GEO::vec2 for a facet and GEO::vec3 for a hexahedron.
         * @param[in] mjd Analyzer under test.
         * @param[in] element Cell (hexahedral grid) or facet (quad grid) index.
         * @param[in] order Polynomial order of the control grid.
         * @param[in] det_jacobian_at Evaluates the determinant at one parametric point.
         * @param[in] gradient_at Fills the gradient of the determinant with respect to the local
         *            control points of the element at one parametric point.
         * @param[in] absolute_area Whether the analyzer is expected to sample the squared absolute
         *            area instead of the signed determinant.
         */
        template <typename Grid, typename Sample, typename DetFn, typename GradFn>
        void expect_untangling_funcgrad_is_correct(
            const MinimumJacobianDeterminant<Grid>& mjd,
            const GEO::index_t element,
            const GEO::index_t order,
            DetFn&& det_jacobian_at,
            GradFn&& gradient_at,
            const bool absolute_area = false
            ) {
            Eigen::VectorXd det_J_coeffs;
            Eigen::MatrixXd grad_det_J_coeffs;
            mjd.compute_untangling_funcgrad(element, det_J_coeffs, grad_det_J_coeffs);

            SCOPED_TRACE(::testing::Message() << "element " << element);

            const GEO::index_t n = determinant_degree<Grid>(order, absolute_area);
            const GEO::index_t N1 = n+1;
            const GEO::index_t coefficient_nb = (Sample::dim == 2) ? N1*N1 : N1*N1*N1;

            ASSERT_EQ(det_J_coeffs.size(), coefficient_nb);
            ASSERT_EQ(grad_det_J_coeffs.rows(), coefficient_nb);

            /* The number of differentiated degrees of freedom is the one the control grid reports. */
            std::vector<double> gradient;
            gradient_at(Sample(), gradient);
            const GEO::index_t dof_nb = gradient.size();
            ASSERT_GT(dof_nb, 0u);
            ASSERT_EQ(grad_det_J_coeffs.cols(), dof_nb);

            /* Corner coefficients are exact values of the determinant / of its gradient. */
            for (const auto& [index, point] : element_corners<Sample>(N1)) {
                SCOPED_TRACE(parametric_context("corner", point));
                EXPECT_NEAR(det_J_coeffs[index], det_jacobian_at(point), BLOCK_CORNER_TOL);

                gradient_at(point, gradient);
                ASSERT_EQ(gradient.size(), dof_nb);
                for (GEO::index_t d = 0; d < dof_nb; ++d)
                    EXPECT_NEAR(grad_det_J_coeffs(index, d), gradient[d], BERNSTEIN_GRADIENT_TOL)
                        << "control-point degree of freedom " << d;
            }

            /* Reconstruction at random points must reproduce the analyzed function. */
            const double reconstruction_tol = absolute_area ? ABSOLUTE_AREA_RECONSTRUCTION_TOL : BERNSTEIN_RECONSTRUCTION_TOL;
            for (const auto& point : random_unit_samples<Sample>(RECONSTRUCTION_SAMPLE_NB)) {
                SCOPED_TRACE(parametric_context("sample", point));

                const double reconstructed_det_J = evaluate_bernstein_tensor_product<Sample>(n, point,
                    [&](const GEO::index_t i, const GEO::index_t j, const GEO::index_t k) {
                        return det_J_coeffs[i + j*N1 + k*N1*N1];
                    });
                EXPECT_NEAR(reconstructed_det_J, det_jacobian_at(point), reconstruction_tol);

                gradient_at(point, gradient);
                for (GEO::index_t d = 0; d < dof_nb; ++d) {
                    const double reconstructed_gradient = evaluate_bernstein_tensor_product<Sample>(n, point,
                        [&](const GEO::index_t i, const GEO::index_t j, const GEO::index_t k) {
                            return grad_det_J_coeffs(i + j*N1 + k*N1*N1, d);
                        });
                    EXPECT_NEAR(reconstructed_gradient, gradient[d], reconstruction_tol)
                        << "control-point degree of freedom " << d;
                }
            }
        }
    }

    /* == hexahedral grid ====================================================================================== */

    /**
     * @brief Fixture of the tests running on the mesh made of two hexahedral cells sharing a facet.
     *
     * Cell 0 is the unit cube with one of its interior control nodes displaced, which inverts it
     * locally; cell 1 is the unit cube translated by one unit along y and stays valid.
     */
    class HexMinimumJacobianDeterminantTest : public ::testing::Test {
    protected:
        using Grid = HexControlGrid;
        static constexpr GEO::index_t ORDER = 4;

        void SetUp() override {
            mesh.vertices.create_vertices(12);
            mesh.vertices.point(0) = GEO::vec3(0, 0, 0);
            mesh.vertices.point(1) = GEO::vec3(1, 0, 0);
            mesh.vertices.point(2) = GEO::vec3(0, 1, 0);
            mesh.vertices.point(3) = GEO::vec3(1, 1, 0);
            mesh.vertices.point(4) = GEO::vec3(0, 0, 1);
            mesh.vertices.point(5) = GEO::vec3(1, 0, 1);
            mesh.vertices.point(6) = GEO::vec3(0, 1, 1);
            mesh.vertices.point(7) = GEO::vec3(1, 1, 1);
            mesh.vertices.point(8) = GEO::vec3(0, 2, 0);
            mesh.vertices.point(9) = GEO::vec3(1, 2, 0);
            mesh.vertices.point(10) = GEO::vec3(0, 2, 1);
            mesh.vertices.point(11) = GEO::vec3(1, 2, 1);
            mesh.cells.create_hex(0, 1, 2, 3, 4, 5, 6, 7);
            mesh.cells.create_hex(2, 8, 6, 10, 3, 9, 7, 11);
            mesh.cells.connect();

            control_grid = std::make_unique<Grid>(mesh, ORDER);
            control_grid->control_nodes.point<3>(control_grid->cell_nd(0, 2, 3, 1)) += GEO::vec3(-0.2, -0.4, 0.1);

            mjd = std::make_unique<MinimumJacobianDeterminant<Grid>>(*control_grid);
        }

        /** @return The determinant of cell @p c at one parametric point. */
        [[nodiscard]] double det_jacobian(const GEO::index_t c, const GEO::vec3& uvw) const {
            return control_grid->compute_cell_uvw_measure(c, uvw, Grid::MeasureType::DET_JACOBIAN);
        }

        GEO::Mesh mesh;
        std::unique_ptr<Grid> control_grid;
        std::unique_ptr<MinimumJacobianDeterminant<Grid>> mjd;
    };

    TEST_F(HexMinimumJacobianDeterminantTest, finds_inverted_cell) {
        expect_analysis_consistent<Grid, GEO::vec3>(*mjd, 0, true,
            [this](const GEO::vec3& uvw) { return det_jacobian(0, uvw); }, "c0");
        expect_analysis_consistent<Grid, GEO::vec3>(*mjd, 1, false,
            [this](const GEO::vec3& uvw) { return det_jacobian(1, uvw); }, "c1");
    }

    TEST_F(HexMinimumJacobianDeterminantTest, computes_untangling_funcgrad) {
        for (const GEO::index_t c : {GEO::index_t(0), GEO::index_t(1)}) {
            expect_untangling_funcgrad_is_correct<Grid, GEO::vec3>(*mjd, c, ORDER,
                [this, c](const GEO::vec3& uvw) { return det_jacobian(c, uvw); },
                [this, c](const GEO::vec3& uvw, std::vector<double>& gradient) {
                    control_grid->compute_cell_uvw_detJ_gradient(c, uvw, gradient);
                });
        }
    }

    /* == quadrilateral grids ================================================================================== */

    /**
     * @brief Fixture of the tests running on the mesh made of two quadrilateral facets sharing an edge.
     *
     * Facet 1 holds a displaced interior control node, which inverts it locally; facet 0 is the unit
     * square and stays valid.
     *
     * @tparam DimType Wrapper carrying the physical dimension, as in DimTypes.
     */
    template <typename DimType>
    class QuadMinimumJacobianDeterminantTest : public ::testing::Test {
    protected:
        static constexpr GEO::index_t MESH_DIM = DimType::value;
        static constexpr GEO::index_t ORDER = 5;
        static constexpr GEO::index_t DISCRETIZATION_RESOLUTION = 20;

        void SetUp() override {
            mesh.vertices.set_dimension(MESH_DIM);
            mesh.vertices.create_vertices(6);
            if constexpr (MESH_DIM == 2) {
                mesh.vertices.template point<2>(0) = GEO::vec2(0, 0);
                mesh.vertices.template point<2>(1) = GEO::vec2(1, 0);
                mesh.vertices.template point<2>(2) = GEO::vec2(1, 1);
                mesh.vertices.template point<2>(3) = GEO::vec2(0, 1);
                mesh.vertices.template point<2>(4) = GEO::vec2(2, 0);
                mesh.vertices.template point<2>(5) = GEO::vec2(2, 1);
            }
            else {
                mesh.vertices.point(0) = GEO::vec3(0, 0, 0);
                mesh.vertices.point(1) = GEO::vec3(1, 0, 0);
                mesh.vertices.point(2) = GEO::vec3(1, 1, 0);
                mesh.vertices.point(3) = GEO::vec3(0, 1, 0);
                mesh.vertices.point(4) = GEO::vec3(2, 0, 0);
                mesh.vertices.point(5) = GEO::vec3(2, 1, 0);
            }
            mesh.facets.create_quad(0, 1, 2, 3);
            mesh.facets.create_quad(5, 2, 1, 4);
            mesh.facets.connect();

            control_grid = std::make_unique<QuadControlGrid>(mesh, ORDER);
            if constexpr (MESH_DIM == 2)
                control_grid->control_nodes.point<2>(control_grid->facet_inner_nd(1, 3, 2)) += GEO::vec2(-0.4, 0.1);
            else
                control_grid->control_nodes.point<3>(control_grid->facet_inner_nd(1, 3, 2)) += GEO::vec3(0.2, -0.4, 0.1);

            mjd = std::make_unique<MinimumJacobianDeterminant<QuadControlGrid>>(*control_grid);
        }

        /** @return The determinant of facet @p f at one parametric point. */
        [[nodiscard]] double det_jacobian(const GEO::index_t f, const GEO::vec2& uv) const {
            return control_grid->compute_facet_uv_measure(f, uv, QuadControlGrid::MeasureType::DET_JACOBIAN);
        }

        /** @return The quality measures of facet @p f at one parametric point. */
        [[nodiscard]] Quality evaluate_quality(const GEO::index_t f, const GEO::vec2& uv) const {
            return {
                .det_jacobian = control_grid->compute_facet_uv_measure(f, uv, QuadControlGrid::MeasureType::DET_JACOBIAN),
                .absolute_sq_area = control_grid->compute_facet_uv_measure(f, uv, QuadControlGrid::MeasureType::ABSOLUTE_SQ_AREA),
                .scaled_jacobian = control_grid->compute_facet_uv_measure(f, uv, QuadControlGrid::MeasureType::SCALED_JACOBIAN),
                .inverse_mean_ratio = control_grid->compute_facet_uv_measure(f, uv, QuadControlGrid::MeasureType::INVERSE_MEAN_RATIO),
                .MIPS = control_grid->compute_facet_uv_measure(f, uv, QuadControlGrid::MeasureType::MIPS)
            };
        }

        /**
         * @brief Dump the discretized high-order facets, with the quality measures of every vertex.
         * @param[in] suffix Suffix of the artifact file name.
         */
        void save_high_order_mesh_facets(const std::string_view suffix = "_ho_mesh.geogram") const {
            GEO::Mesh mesh_out(MESH_DIM);
            GEO::Attribute<GEO::index_t> mesh_out_v_facet(mesh_out.vertices.attributes(), "facet");
            GEO::Attribute<GEO::vec2> mesh_out_v_uv(mesh_out.vertices.attributes(), "uv");

            control_grid->append_discretized_high_order_facets<MESH_DIM>(
                mesh_out,
                DISCRETIZATION_RESOLUTION,
                &mesh_out_v_facet,
                &mesh_out_v_uv);

            MeshQualityAttributes quality(mesh_out);
            for (const auto v : mesh_out.vertices)
                quality.set(v, evaluate_quality(mesh_out_v_facet[v], mesh_out_v_uv[v]));

            EXPECT_TRUE(mesh_out.save(artifact_path(suffix)));
        }

        GEO::Mesh mesh;
        std::unique_ptr<QuadControlGrid> control_grid;
        std::unique_ptr<MinimumJacobianDeterminant<QuadControlGrid>> mjd;
    };

    TYPED_TEST_SUITE(QuadMinimumJacobianDeterminantTest, DimTypes);

    TYPED_TEST(QuadMinimumJacobianDeterminantTest, finds_inverted_facet) {
        this->save_high_order_mesh_facets();

        expect_analysis_consistent<QuadControlGrid, GEO::vec2>(*this->mjd, 0, false,
            [this](const GEO::vec2& uv) { return this->det_jacobian(0, uv); }, "f0");
        expect_analysis_consistent<QuadControlGrid, GEO::vec2>(*this->mjd, 1, true,
            [this](const GEO::vec2& uv) { return this->det_jacobian(1, uv); }, "f1");
    }

    TYPED_TEST(QuadMinimumJacobianDeterminantTest, computes_untangling_funcgrad) {
        for (const GEO::index_t f : {GEO::index_t(0), GEO::index_t(1)}) {
            expect_untangling_funcgrad_is_correct<QuadControlGrid, GEO::vec2>(*this->mjd, f, this->ORDER,
                [this, f](const GEO::vec2& uv) { return this->det_jacobian(f, uv); },
                [this, f](const GEO::vec2& uv, std::vector<double>& gradient) {
                    this->control_grid->compute_facet_uv_detJ_gradient(f, uv, gradient);
                });
        }
    }

    TYPED_TEST(QuadMinimumJacobianDeterminantTest, computes_untangling_funcgrad_absolute_area) {
        if (this->MESH_DIM != 3)
            GTEST_SKIP() << "The absolute-area measure is defined for 3D facets only.";

        this->mjd->use_absolute_area();

        for (const GEO::index_t f : {GEO::index_t(0), GEO::index_t(1)}) {
            expect_untangling_funcgrad_is_correct<QuadControlGrid, GEO::vec2>(*this->mjd, f, this->ORDER,
                [this, f](const GEO::vec2& uv) {
                    return this->control_grid->compute_facet_uv_measure(f, uv, QuadControlGrid::MeasureType::ABSOLUTE_SQ_AREA);
                },
                [this, f](const GEO::vec2& uv, std::vector<double>& gradient) {
                    this->control_grid->compute_facet_uv_absolute_area_sq_gradient(f, uv, gradient);
                },
                true);
        }
    }
}
