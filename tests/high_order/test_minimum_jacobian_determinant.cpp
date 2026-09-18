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
#include <geolio/high_order/minimum_jacobian_determinant.h>
#include <gtest/gtest.h>
#include "control_grid_test_utils.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <string_view>
#include <type_traits>
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

            constexpr GEO::index_t ORDER = 4;
            control_grid = std::make_unique<HexControlGrid>(mesh, ORDER);
            control_grid->control_node(control_grid->cell_nd(0, 2, 3, 1)) += GEO::vec3(-0.2, -0.4, 0.1);

            mjd = std::make_unique<MinimumJacobianDeterminant<HexControlGrid>>(*control_grid);
        }

        /** @return The determinant of cell @p c at one parametric point. */
        [[nodiscard]] double det_jacobian(const GEO::index_t c, const GEO::vec3& uvw) const {
            return control_grid->compute_cell_uvw_measure(c, uvw, HexControlGrid::MeasureType::DET_JACOBIAN);
        }

        GEO::Mesh mesh;
        std::unique_ptr<HexControlGrid> control_grid;
        std::unique_ptr<MinimumJacobianDeterminant<HexControlGrid>> mjd;
    };

    TEST_F(HexMinimumJacobianDeterminantTest, finds_inverted_cell) {
        expect_analysis_consistent<HexControlGrid, GEO::vec3>(*mjd, 0, true,
            [this](const GEO::vec3& uvw) { return det_jacobian(0, uvw); }, "c0");
        expect_analysis_consistent<HexControlGrid, GEO::vec3>(*mjd, 1, false,
            [this](const GEO::vec3& uvw) { return det_jacobian(1, uvw); }, "c1");
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
        static constexpr GEO::index_t DIM = DimType::value;
        static constexpr GEO::index_t ORDER = 5;
        static constexpr GEO::index_t DISCRETIZATION_RESOLUTION = 20;

        void SetUp() override {
            mesh.vertices.set_dimension(DIM);
            mesh.vertices.create_vertices(6);
            if constexpr (DIM == 2) {
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

            control_grid = std::make_unique<QuadControlGrid<DIM>>(mesh, ORDER);
            if constexpr (DIM == 2)
                control_grid->control_node(control_grid->facet_inner_nd(1, 3, 2)) += GEO::vec2(-0.4, 0.1);
            else
                control_grid->control_node(control_grid->facet_inner_nd(1, 3, 2)) += GEO::vec3(0.2, -0.4, 0.1);

            mjd = std::make_unique<MinimumJacobianDeterminant<QuadControlGrid<DIM>>>(*control_grid);
        }

        /** @return The determinant of facet @p f at one parametric point. */
        [[nodiscard]] double det_jacobian(const GEO::index_t f, const GEO::vec2& uv) const {
            return control_grid->compute_facet_uv_measure(f, uv, QuadControlGrid<DIM>::MeasureType::DET_JACOBIAN);
        }

        /** @return The quality measures of facet @p f at one parametric point. */
        [[nodiscard]] Quality evaluate_quality(const GEO::index_t f, const GEO::vec2& uv) const {
            return {
                .det_jacobian = control_grid->compute_facet_uv_measure(f, uv, QuadControlGrid<DIM>::MeasureType::DET_JACOBIAN),
                .absolute_sq_area = control_grid->compute_facet_uv_measure(f, uv, QuadControlGrid<DIM>::MeasureType::ABSOLUTE_SQ_AREA),
                .scaled_jacobian = control_grid->compute_facet_uv_measure(f, uv, QuadControlGrid<DIM>::MeasureType::SCALED_JACOBIAN),
                .inverse_mean_ratio = control_grid->compute_facet_uv_measure(f, uv, QuadControlGrid<DIM>::MeasureType::INVERSE_MEAN_RATIO),
                .MIPS = control_grid->compute_facet_uv_measure(f, uv, QuadControlGrid<DIM>::MeasureType::MIPS)
            };
        }

        /**
         * @brief Dump the discretized high-order facets, with the quality measures of every vertex.
         * @param[in] suffix Suffix of the artifact file name.
         */
        void save_high_order_mesh_facets(const std::string_view suffix = "_ho_mesh.geogram") const {
            GEO::Mesh mesh_out(DIM);
            GEO::Attribute<GEO::index_t> mesh_out_v_facet(mesh_out.vertices.attributes(), "facet");
            GEO::Attribute<GEO::vec2> mesh_out_v_uv(mesh_out.vertices.attributes(), "uv");

            control_grid->append_discretized_high_order_facets(
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
        std::unique_ptr<QuadControlGrid<DIM>> control_grid;
        std::unique_ptr<MinimumJacobianDeterminant<QuadControlGrid<DIM>>> mjd;
    };

    TYPED_TEST_SUITE(QuadMinimumJacobianDeterminantTest, DimTypes);

    TYPED_TEST(QuadMinimumJacobianDeterminantTest, finds_inverted_facet) {
        this->save_high_order_mesh_facets();

        expect_analysis_consistent<QuadControlGrid<TypeParam::value>, GEO::vec2>(*this->mjd, 0, false,
            [this](const GEO::vec2& uv) { return this->det_jacobian(0, uv); }, "f0");
        expect_analysis_consistent<QuadControlGrid<TypeParam::value>, GEO::vec2>(*this->mjd, 1, true,
            [this](const GEO::vec2& uv) { return this->det_jacobian(1, uv); }, "f1");
    }
}
