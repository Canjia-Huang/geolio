//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/4.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
// Tests of HexControlGrid, in two parts:
//   * the parametric projection helpers, checked as properties of the reference unit cube
//     (HexVertexProjectionTest / HexEdgeProjectionTest / HexFacetProjectionTest);
//   * the control grid itself: high-order mapping, parametric derivatives, facet normals,
//     control-node quantities and cell quality measures, on a single hex (order 5) and on a
//     two-hex mesh (order 6).
// Every geometric case is also dumped as a .geogram artifact named after the running test
// ("test_<suite>_<test>*.geogram" in the working directory) for visual inspection in geobox.
//
#include <geolio/high_order/hex_control_grid.h>
#include <gtest/gtest.h>
#include "control_grid_test_utils.h"

#include <algorithm>
#include <array>
#include <memory>
#include <numeric>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace geolio::test
{
    namespace
    {
        /* == shared data ====================================================================================== */

        /** Local topology of a hexahedron. */
        constexpr GEO::index_t HEX_VERTEX_NB = 8;
        constexpr GEO::index_t HEX_EDGE_NB = 12;
        constexpr GEO::index_t HEX_FACET_NB = 6;

        /** Index of the reference cell of the test meshes. */
        constexpr GEO::index_t CELL = 0;

        /** Resolution of the parametric grids used by the quality-measure checks. */
        constexpr GEO::index_t SAMPLE_RESOLUTION = 10;

        /** Resolution of the discretized high-order cells dumped as artifacts. */
        constexpr GEO::index_t DISCRETIZATION_RESOLUTION = 20;

        /** Resolution of the finer background mesh dumped behind the derivative/normal arrows. */
        constexpr GEO::index_t BACKGROUND_RESOLUTION = 30;

        /** Resolution of the inner sample grid of the finite-difference derivative checks. */
        constexpr GEO::index_t FINITE_DIFF_RESOLUTION = 5;

        /** Amplitude of the random jitter applied to the control nodes of the "random" cases. */
        constexpr double JITTER_AMPLITUDE = 0.1;

        /** Step of the central finite differences used to check the analytic derivatives. */
        constexpr double FINITE_DIFF_STEP = 1e-6;

        /** Tolerance on the squared distance between analytic and finite-difference derivatives. */
        constexpr double FINITE_DIFF_DIST2_TOL = 1e-16;

        /** Tolerance on the cosine between a facet normal and a tangent of that facet. */
        constexpr double NORMAL_ORTHOGONALITY_TOL = 1e-8;

        /** Corners of the reference cell (local vertex i -> corner i). */
        const std::array<GEO::vec3, HEX_VERTEX_NB> UNIT_HEX_CORNERS = {
            GEO::vec3(0.0, 0.0, 0.0), GEO::vec3(1.0, 0.0, 0.0),
            GEO::vec3(0.0, 1.0, 0.0), GEO::vec3(1.0, 1.0, 0.0),
            GEO::vec3(0.0, 0.0, 1.0), GEO::vec3(1.0, 0.0, 1.0),
            GEO::vec3(0.0, 1.0, 1.0), GEO::vec3(1.0, 1.0, 1.0)
        };

        /** Corners of the two-hex mesh: the unit cube, plus its translate by one unit along y. */
        const std::array<GEO::vec3, 12> TWO_HEX_CORNERS = {
            GEO::vec3(0.0, 0.0, 0.0), GEO::vec3(1.0, 0.0, 0.0),
            GEO::vec3(0.0, 1.0, 0.0), GEO::vec3(1.0, 1.0, 0.0),
            GEO::vec3(0.0, 0.0, 1.0), GEO::vec3(1.0, 0.0, 1.0),
            GEO::vec3(0.0, 1.0, 1.0), GEO::vec3(1.0, 1.0, 1.0),
            GEO::vec3(0.0, 2.0, 0.0), GEO::vec3(1.0, 2.0, 0.0),
            GEO::vec3(0.0, 2.0, 1.0), GEO::vec3(1.0, 2.0, 1.0)
        };

        /* == shared helpers =================================================================================== */

        /**
         * @brief Sample the two facet tangents of a cell facet by central finite differences.
         *
         * The tangents are obtained from the cell mapping, evaluated at the parametric points of
         * the facet that surround @p uv, so they span the tangent plane of the facet.
         *
         * @param[in] grid Control grid owning the cell.
         * @param[in] c Cell index.
         * @param[in] lf Local facet index.
         * @param[in] uv Facet-local parametric point, in [h, 1-h]^2.
         * @return The two tangents.
         */
        template <typename Grid>
        [[nodiscard]] std::pair<GEO::vec3, GEO::vec3> finite_difference_facet_tangents(
            const Grid& grid,
            const GEO::index_t c,
            const GEO::index_t lf,
            const GEO::vec2& uv
            ) {
            const auto position = [&](const double du, const double dv) {
                return grid.compute_cell_uvw_position(c, project_hex_lf_uv_to_uvw(GEO::vec2(uv.x + du, uv.y + dv), lf));
            };
            return {
                (position(FINITE_DIFF_STEP, 0.0) - position(-FINITE_DIFF_STEP, 0.0)) / (2.0*FINITE_DIFF_STEP),
                (position(0.0, FINITE_DIFF_STEP) - position(0.0, -FINITE_DIFF_STEP)) / (2.0*FINITE_DIFF_STEP)
            };
        }
    }

    /* == parametric projections =============================================================================== */

    /**
     * @brief Reference cell of the projection tests.
     *
     * The mesh is the unit cube [0,1]^3, whose local vertex i sits exactly on the corner
     * project_hex_lv_to_uvw(i).
     */
    class UnitHexTest : public ::testing::Test {
    protected:
        void SetUp() override {
            mesh.vertices.create_vertices(HEX_VERTEX_NB);
            for (GEO::index_t lv = 0; lv < HEX_VERTEX_NB; ++lv)
                mesh.vertices.point(lv) = UNIT_HEX_CORNERS[lv];
            mesh.cells.create_hex(0, 1, 2, 3, 4, 5, 6, 7);
        }

        /**
         * @brief Endpoints of a local edge of the reference cell.
         * @param[in] le Local edge index, 0,1,...,11.
         * @return The endpoints, oriented along the edge parameter.
         */
        [[nodiscard]] std::pair<GEO::vec3, GEO::vec3> cell_edge(const GEO::index_t le) const {
            return {
                mesh.vertices.point(mesh.cells.edge_vertex(CELL, le, 0)),
                mesh.vertices.point(mesh.cells.edge_vertex(CELL, le, 1))
            };
        }

        /**
         * @brief Corners of a local facet of the reference cell.
         * @param[in] lf Local facet index, 0,1,...,5.
         * @return The corners, ordered as the facet's vertices: (0,0), (1,0), (1,1), (0,1).
         */
        [[nodiscard]] std::array<GEO::vec3, 4> cell_facet_corners(const GEO::index_t lf) const {
            std::array<GEO::vec3, 4> corners;
            for (GEO::index_t k = 0; k < 4; ++k)
                corners[k] = mesh.vertices.point(mesh.cells.facet_vertex(CELL, lf, k));
            return corners;
        }

        /**
         * @brief Bilinear interpolation of the corners of a local facet.
         * @param[in] corners Corners ordered as in cell_facet_corners().
         * @param[in] uv Facet-local parametric point in [0,1]^2.
         * @return The interpolated point.
         */
        [[nodiscard]] static GEO::vec3 facet_bilinear(
            const std::array<GEO::vec3, 4>& corners,
            const GEO::vec2& uv
            ) {
            return (1.0 - uv.x)*(1.0 - uv.y)*corners[0] + uv.x*(1.0 - uv.y)*corners[1]
                 + (1.0 - uv.x)*uv.y*corners[3] + uv.x*uv.y*corners[2];
        }

        GEO::Mesh mesh;
    };

    /** Fixture parametrized by the local vertex index of the reference cell. */
    class HexVertexProjectionTest : public UnitHexTest, public ::testing::WithParamInterface<GEO::index_t> {
    protected:
        void SetUp() override {
            UnitHexTest::SetUp();
            lv = GetParam();
            ASSERT_LT(lv, HEX_VERTEX_NB);
        }

        GEO::index_t lv = 0;
    };

    TEST_P(HexVertexProjectionTest, projects_vertex_to_parametric_corner) {
        const auto uvw = project_hex_lv_to_uvw(lv);
        const auto& p = mesh.vertices.point(mesh.cells.vertex(CELL, lv));

        EXPECT_NEAR(GEO::distance2(p, uvw), 0.0, EXACT_DIST2_TOL);
        EXPECT_NEAR(GEO::distance2(uvw, UNIT_HEX_CORNERS[lv]), 0.0, EXACT_DIST2_TOL);
    }

    TEST_P(HexVertexProjectionTest, projects_corner_to_edge_endpoints) {
        const auto uvw = project_hex_lv_to_uvw(lv);

        // A corner of the cube projects onto one of the two endpoints of every edge.
        for (GEO::index_t le = 0; le < HEX_EDGE_NB; ++le) {
            SCOPED_TRACE(::testing::Message() << "le = " << le);
            const double t = project_uvw_to_hex_le_t(uvw, le);

            EXPECT_GE(t, 0.0);
            EXPECT_LE(t, 1.0);
            EXPECT_NEAR(t*(1.0 - t), 0.0, EXACT_DIST2_TOL) << "t = " << t;
        }
    }

    INSTANTIATE_TEST_SUITE_P(
        AllLocalVertices,
        HexVertexProjectionTest,
        ::testing::Range<GEO::index_t>(0, HEX_VERTEX_NB)
        );

    /** Fixture parametrized by the local edge index of the reference cell. */
    class HexEdgeProjectionTest : public UnitHexTest, public ::testing::WithParamInterface<GEO::index_t> {
    protected:
        void SetUp() override {
            UnitHexTest::SetUp();
            le = GetParam();
            ASSERT_LT(le, HEX_EDGE_NB);
            uvws = random_unit_samples_3d(RANDOM_SAMPLE_NB);
            ts = random_unit_samples_1d(RANDOM_SAMPLE_NB);
        }

        GEO::index_t le = 0;
        std::vector<GEO::vec3> uvws;
        std::vector<double> ts;
    };

    TEST_P(HexEdgeProjectionTest, projects_uvw_to_closest_edge_parameter) {
        const auto [ep0, ep1] = cell_edge(le);
        const auto edge = ep1 - ep0;
        const double edge_length2 = GEO::dot(edge, edge);

        for (const auto& uvw : uvws) {
            const double t = project_uvw_to_hex_le_t(uvw, le);
            EXPECT_GE(t, 0.0);
            EXPECT_LE(t, 1.0);

            // The projected point is the foot of the perpendicular dropped from uvw on the edge.
            EXPECT_NEAR(GEO::dot(edge, uvw - ((1.0 - t)*ep0 + t*ep1)), 0.0, REFERENCE_TOL);

            // ... which is the closest point of the segment, recomputed independently.
            const double t_closest = std::clamp(GEO::dot(uvw - ep0, edge) / edge_length2, 0.0, 1.0);
            EXPECT_NEAR(t, t_closest, REFERENCE_TOL);
        }
    }

    TEST_P(HexEdgeProjectionTest, projects_edge_parameter_to_uvw) {
        const auto [ep0, ep1] = cell_edge(le);

        for (const double t : ts) {
            const auto uvw = project_hex_le_t_to_uvw(t, le);
            EXPECT_GE(uvw.x, 0.0);
            EXPECT_LE(uvw.x, 1.0);
            EXPECT_GE(uvw.y, 0.0);
            EXPECT_LE(uvw.y, 1.0);
            EXPECT_GE(uvw.z, 0.0);
            EXPECT_LE(uvw.z, 1.0);

            // The parametric point lies on the edge ...
            EXPECT_NEAR(GEO::distance2(uvw, (1.0 - t)*ep0 + t*ep1), 0.0, EXACT_DIST2_TOL);

            // ... and the two projections are inverse of each other on the edge.
            EXPECT_NEAR(project_uvw_to_hex_le_t(uvw, le), t, REFERENCE_TOL);
        }
    }

    INSTANTIATE_TEST_SUITE_P(
        AllLocalEdges,
        HexEdgeProjectionTest,
        ::testing::Range<GEO::index_t>(0, HEX_EDGE_NB)
        );

    /** Fixture parametrized by the local facet index of the reference cell. */
    class HexFacetProjectionTest : public UnitHexTest, public ::testing::WithParamInterface<GEO::index_t> {
    protected:
        void SetUp() override {
            UnitHexTest::SetUp();
            lf = GetParam();
            ASSERT_LT(lf, HEX_FACET_NB);
            uvws = random_unit_samples_3d(RANDOM_SAMPLE_NB);
            uvs = random_unit_samples_2d(RANDOM_SAMPLE_NB);
        }

        GEO::index_t lf = 0;
        std::vector<GEO::vec3> uvws;
        std::vector<GEO::vec2> uvs;
    };

    TEST_P(HexFacetProjectionTest, projects_uvw_to_facet_uv) {
        const auto corners = cell_facet_corners(lf);

        for (const auto& uvw : uvws) {
            const auto uv = project_uvw_to_hex_lf_uv(uvw, lf);
            EXPECT_GE(uv.x, 0.0);
            EXPECT_LE(uv.x, 1.0);
            EXPECT_GE(uv.y, 0.0);
            EXPECT_LE(uv.y, 1.0);

            // The projected parameters locate the orthogonal projection of the point on the facet:
            // the residual is orthogonal to the two corner edges spanning the facet.
            const auto p = facet_bilinear(corners, uv);
            EXPECT_NEAR(GEO::dot(corners[1] - corners[0], uvw - p), 0.0, REFERENCE_TOL);
            EXPECT_NEAR(GEO::dot(corners[3] - corners[0], uvw - p), 0.0, REFERENCE_TOL);
        }
    }

    TEST_P(HexFacetProjectionTest, projects_facet_uv_to_uvw) {
        const auto corners = cell_facet_corners(lf);

        for (const auto& uv : uvs) {
            const auto uvw = project_hex_lf_uv_to_uvw(uv, lf);
            EXPECT_GE(uvw.x, 0.0);
            EXPECT_LE(uvw.x, 1.0);
            EXPECT_GE(uvw.y, 0.0);
            EXPECT_LE(uvw.y, 1.0);
            EXPECT_GE(uvw.z, 0.0);
            EXPECT_LE(uvw.z, 1.0);

            // The parametric point lies on the facet ...
            EXPECT_NEAR(GEO::distance2(uvw, facet_bilinear(corners, uv)), 0.0, EXACT_DIST2_TOL);

            // ... and the two projections are inverse of each other on the facet.
            const auto uv_back = project_uvw_to_hex_lf_uv(uvw, lf);
            EXPECT_NEAR(GEO::distance2(uv_back, uv), 0.0, EXACT_DIST2_TOL);
        }
    }

    INSTANTIATE_TEST_SUITE_P(
        AllLocalFacets,
        HexFacetProjectionTest,
        ::testing::Range<GEO::index_t>(0, HEX_FACET_NB)
        );

    TEST_F(UnitHexTest, edge_endpoints_match_cell_vertices) {
        for (GEO::index_t le = 0; le < HEX_EDGE_NB; ++le) {
            SCOPED_TRACE(::testing::Message() << "le = " << le);
            const auto [ep0, ep1] = cell_edge(le);

            EXPECT_NEAR(GEO::distance2(project_hex_le_t_to_uvw(0.0, le), ep0), 0.0, EXACT_DIST2_TOL);
            EXPECT_NEAR(GEO::distance2(project_hex_le_t_to_uvw(1.0, le), ep1), 0.0, EXACT_DIST2_TOL);
            EXPECT_NEAR(project_uvw_to_hex_le_t(ep0, le), 0.0, EXACT_DIST2_TOL);
            EXPECT_NEAR(project_uvw_to_hex_le_t(ep1, le), 1.0, EXACT_DIST2_TOL);
        }
    }

    TEST_F(UnitHexTest, facet_corners_match_facet_uv) {
        const std::array<GEO::vec2, 4> CORNER_UV = {
            GEO::vec2(0.0, 0.0), GEO::vec2(1.0, 0.0), GEO::vec2(1.0, 1.0), GEO::vec2(0.0, 1.0)
        };

        for (GEO::index_t lf = 0; lf < HEX_FACET_NB; ++lf) {
            const auto corners = cell_facet_corners(lf);

            for (GEO::index_t k = 0; k < 4; ++k) {
                SCOPED_TRACE(::testing::Message() << "lf = " << lf << ", corner " << k);

                // The k-th facet corner projects onto the k-th corner of the facet domain ...
                EXPECT_NEAR(GEO::distance2(project_uvw_to_hex_lf_uv(corners[k], lf), CORNER_UV[k]), 0.0, EXACT_DIST2_TOL);
                // ... and the inverse projection maps it back onto that corner.
                EXPECT_NEAR(GEO::distance2(project_hex_lf_uv_to_uvw(CORNER_UV[k], lf), corners[k]), 0.0, EXACT_DIST2_TOL);
            }
        }
    }

    /* == control grid ========================================================================================= */

    /**
     * @brief Common helpers of the HexControlGrid tests.
     *
     * The hexahedral control grid is three-dimensional, so unlike its quadrilateral counterpart
     * it is not parametrized by the physical dimension.
     */
    class HexControlGridTest : public ::testing::Test {
    protected:
        using Grid = HexControlGrid;

        /* == meshes =========================================================================================== */

        /**
         * @brief Create the vertices of the reference mesh.
         * @param[in] positions One position per vertex.
         */
        void create_vertices(const std::span<const GEO::vec3> positions) {
            mesh.vertices.create_vertices(static_cast<GEO::index_t>(positions.size()));
            for (GEO::index_t v = 0; v < positions.size(); ++v)
                mesh.vertices.point(v) = positions[v];
        }

        /* == perturbations ==================================================================================== */

        /**
         * @brief Add a random jitter to one control node.
         *
         * The jitter makes the tested configuration generic: its amplitude is small enough for the
         * cell to keep its orientation, which the quality invariants of the tests verify.
         *
         * @param[in] nd Global index of the control node.
         */
        void jiggle_control_node(const GEO::index_t nd) {
            control_grid->control_node(nd) += random_offset<3>(JITTER_AMPLITUDE);
        }

        /* == evaluations ====================================================================================== */

        /**
         * @brief Evaluate every quality measure of a cell at one parametric point.
         *
         * The hexahedral grid does not define an absolute-area measure, so the corresponding field
         * of the result is left at zero.
         *
         * @param[in] c Cell index.
         * @param[in] uvw Parametric point in [0,1]^3.
         * @return The measure values.
         */
        [[nodiscard]] Quality evaluate_quality(const GEO::index_t c, const GEO::vec3& uvw) const {
            return {
                .det_jacobian = control_grid->compute_cell_uvw_measure(c, uvw, Grid::MeasureType::DET_JACOBIAN),
                .scaled_jacobian = control_grid->compute_cell_uvw_measure(c, uvw, Grid::MeasureType::SCALED_JACOBIAN),
                .inverse_mean_ratio = control_grid->compute_cell_uvw_measure(c, uvw, Grid::MeasureType::INVERSE_MEAN_RATIO),
                .MIPS = control_grid->compute_cell_uvw_measure(c, uvw, Grid::MeasureType::MIPS)
            };
        }

        /** First-order parametric data of a cell mapping, at one parametric point. */
        struct Derivatives {
            GEO::vec3 du;
            GEO::vec3 dv;
            GEO::vec3 dw;
            std::vector<double> Bu;
            std::vector<double> Bv;
            std::vector<double> Bw;
            std::vector<double> dBu;
            std::vector<double> dBv;
            std::vector<double> dBw;
        };

        /**
         * @brief Evaluate the parametric derivatives of a cell mapping.
         * @param[in] c Cell index.
         * @param[in] uvw Parametric point in [0,1]^3.
         * @return The tangents and the 1D basis values/derivatives used to build them.
         */
        [[nodiscard]] Derivatives evaluate_derivatives(const GEO::index_t c, const GEO::vec3& uvw) const {
            Derivatives derivatives;
            control_grid->compute_cell_uvw_dudvdw(
                c, uvw,
                derivatives.du, derivatives.dv, derivatives.dw,
                derivatives.Bu, derivatives.Bv, derivatives.Bw,
                derivatives.dBu, derivatives.dBv, derivatives.dBw);
            return derivatives;
        }

        /**
         * @brief Centre of a cell, used as the inner reference point of the facet normals.
         * @param[in] c Cell index.
         * @return The average of the eight cell corner control nodes.
         */
        [[nodiscard]] GEO::vec3 cell_centre(const GEO::index_t c) const {
            GEO::vec3 centre;
            for (GEO::index_t lv = 0; lv < HEX_VERTEX_NB; ++lv)
                centre += control_grid->control_node(control_grid->cell_vertex_nd(c, lv));
            return (1.0/HEX_VERTEX_NB)*centre;
        }

        /* == checks =========================================================================================== */

        /**
         * @brief Check the quality-measure invariants on a regular grid of samples.
         * @param[in] c Cell index.
         * @param[in] resolution Number of subdivisions of each parametric direction.
         */
        void expect_quality_invariants_over_grid(const GEO::index_t c, const GEO::index_t resolution) const {
            for (const auto& uvw : grid_unit_samples_3d(resolution))
                expect_quality_invariants(evaluate_quality(c, uvw), parametric_context("uvw", uvw));
        }

        /**
         * @brief Check that the mapping interpolates the control nodes of a cell.
         *
         * This is the Lagrange delta property: at the parametric position of the local control
         * node (i, j, k), the mapping reduces exactly to that node's position, whatever the
         * (possibly perturbed) control-node coordinates are.
         *
         * @param[in] c Cell index.
         */
        void expect_mapping_interpolates_control_nodes(const GEO::index_t c) const {
            const auto& node_positions = control_grid->node_positions_1D();
            for (GEO::index_t i = 0, i_end = control_grid->order(); i <= i_end; ++i) {
                for (GEO::index_t j = 0, j_end = control_grid->order(); j <= j_end; ++j) {
                    for (GEO::index_t k = 0, k_end = control_grid->order(); k <= k_end; ++k) {
                        SCOPED_TRACE(::testing::Message() << "control node (" << i << ", " << j << ", " << k << ")");

                        const GEO::vec3 uvw(node_positions[i], node_positions[j], node_positions[k]);
                        const auto& nd = control_grid->cell_nd(c, i, j, k);
                        EXPECT_NEAR(
                            GEO::distance2(control_grid->compute_cell_uvw_position(c, uvw), control_grid->control_node(nd)),
                            0.0,
                            EXACT_DIST2_TOL
                            );
                    }
                }
            }
        }

        /**
         * @brief Check that the mapping of an unperturbed axis-aligned unit cube is the identity.
         * @param[in] c Cell index of a cell whose corners are the unit cube corners.
         */
        void expect_identity_mapping(const GEO::index_t c) const {
            for (const auto& uvw : grid_unit_samples_3d(SAMPLE_RESOLUTION))
                EXPECT_NEAR(GEO::distance2(control_grid->compute_cell_uvw_position(c, uvw), uvw), 0.0, EXACT_DIST2_TOL);
        }

        /**
         * @brief Check that the mapping has moved at the parametric position of a perturbed node.
         *
         * The reference (unperturbed) configuration of the test meshes is the identity mapping, so
         * the mapping at the node position must no longer be the node's original corner.
         *
         * @param[in] c Cell index.
         * @param[in] i First local control-node index of the perturbed node.
         * @param[in] j Second local control-node index of the perturbed node.
         * @param[in] k Third local control-node index of the perturbed node.
         */
        void expect_mapping_moved_at_node(
            const GEO::index_t c,
            const GEO::index_t i,
            const GEO::index_t j,
            const GEO::index_t k
            ) const {
            const auto& node_positions = control_grid->node_positions_1D();
            const GEO::vec3 uvw(node_positions[i], node_positions[j], node_positions[k]);

            EXPECT_GT(GEO::distance2(control_grid->compute_cell_uvw_position(c, uvw), uvw), 0.0)
                << "the jitter of control node (" << i << ", " << j << ", " << k << ") must move the mapping";
        }

        /**
         * @brief Check the analytic derivatives against central finite differences, and the 1D bases.
         *
         * The finite differences are evaluated on the interior of the parametric domain, so that
         * uvw +- h stays in [0, 1]. The 1D Lagrange basis values must sum to one (partition of
         * unity) and their derivatives to zero.
         *
         * @param[in] c Cell index.
         */
        void expect_derivatives_match_finite_differences(const GEO::index_t c) const {
            for (GEO::index_t i = 1; i < FINITE_DIFF_RESOLUTION; ++i) {
                for (GEO::index_t j = 1; j < FINITE_DIFF_RESOLUTION; ++j) {
                    for (GEO::index_t k = 1; k < FINITE_DIFF_RESOLUTION; ++k) {
                        SCOPED_TRACE(::testing::Message() << "sample (" << i << ", " << j << ", " << k << ")");

                        const double u = static_cast<double>(i)/FINITE_DIFF_RESOLUTION;
                        const double v = static_cast<double>(j)/FINITE_DIFF_RESOLUTION;
                        const double w = static_cast<double>(k)/FINITE_DIFF_RESOLUTION;
                        const auto derivatives = evaluate_derivatives(c, GEO::vec3(u, v, w));

                        const auto difference = [&](const GEO::index_t axis) {
                            GEO::vec3 plus(u, v, w), minus(u, v, w);
                            plus[axis] += FINITE_DIFF_STEP;
                            minus[axis] -= FINITE_DIFF_STEP;
                            return (
                                control_grid->compute_cell_uvw_position(c, plus) -
                                control_grid->compute_cell_uvw_position(c, minus)
                                ) / (2.0*FINITE_DIFF_STEP);
                        };

                        EXPECT_NEAR(GEO::distance2(derivatives.du, difference(0)), 0.0, FINITE_DIFF_DIST2_TOL);
                        EXPECT_NEAR(GEO::distance2(derivatives.dv, difference(1)), 0.0, FINITE_DIFF_DIST2_TOL);
                        EXPECT_NEAR(GEO::distance2(derivatives.dw, difference(2)), 0.0, FINITE_DIFF_DIST2_TOL);

                        // The measure must be the mixed product of the three tangents.
                        const double det_jacobian = control_grid->compute_cell_uvw_measure(
                            c, GEO::vec3(u, v, w), Grid::MeasureType::DET_JACOBIAN);
                        EXPECT_NEAR(
                            det_jacobian,
                            GEO::dot(derivatives.dw, GEO::cross(derivatives.du, derivatives.dv)),
                            REFERENCE_TOL
                            );

                        EXPECT_NEAR(std::accumulate(derivatives.Bu.begin(), derivatives.Bu.end(), 0.0), 1.0, REFERENCE_TOL);
                        EXPECT_NEAR(std::accumulate(derivatives.Bv.begin(), derivatives.Bv.end(), 0.0), 1.0, REFERENCE_TOL);
                        EXPECT_NEAR(std::accumulate(derivatives.Bw.begin(), derivatives.Bw.end(), 0.0), 1.0, REFERENCE_TOL);
                        EXPECT_NEAR(std::accumulate(derivatives.dBu.begin(), derivatives.dBu.end(), 0.0), 0.0, REFERENCE_TOL);
                        EXPECT_NEAR(std::accumulate(derivatives.dBv.begin(), derivatives.dBv.end(), 0.0), 0.0, REFERENCE_TOL);
                        EXPECT_NEAR(std::accumulate(derivatives.dBw.begin(), derivatives.dBw.end(), 0.0), 0.0, REFERENCE_TOL);
                    }
                }
            }
        }

        /**
         * @brief Check that two cells sharing a global vertex also share its control node.
         * @param[in] c0 First cell index.
         * @param[in] c1 Second cell index.
         */
        void expect_shared_vertices_are_conforming(const GEO::index_t c0, const GEO::index_t c1) const {
            for (GEO::index_t lv0 = 0; lv0 < HEX_VERTEX_NB; ++lv0) {
                for (GEO::index_t lv1 = 0; lv1 < HEX_VERTEX_NB; ++lv1) {
                    if (mesh.cells.vertex(c0, lv0) != mesh.cells.vertex(c1, lv1))
                        continue;

                    SCOPED_TRACE(::testing::Message() << "global vertex " << mesh.cells.vertex(c0, lv0));
                    EXPECT_EQ(control_grid->cell_vertex_nd(c0, lv0), control_grid->cell_vertex_nd(c1, lv1));
                }
            }
        }

        /**
         * @brief Check that two cells sharing a global facet also share its control nodes.
         *
         * The control-node positions of both facets are compared as sorted point sets, so that the
         * check is independent of the way the two cells order the corners of the shared facet.
         *
         * @param[in] c0 First cell index.
         * @param[in] c1 Second cell index.
         */
        void expect_shared_facets_are_conforming(const GEO::index_t c0, const GEO::index_t c1) const {
            for (GEO::index_t lf0 = 0; lf0 < HEX_FACET_NB; ++lf0) {
                for (GEO::index_t lf1 = 0; lf1 < HEX_FACET_NB; ++lf1) {
                    if (facet_vertices(c0, lf0) != facet_vertices(c1, lf1))
                        continue;

                    SCOPED_TRACE(::testing::Message() << "shared facet " << lf0 << " / " << lf1);
                    const auto positions0 = facet_control_node_positions(c0, lf0);
                    const auto positions1 = facet_control_node_positions(c1, lf1);
                    ASSERT_EQ(positions0.size(), positions1.size());

                    for (std::size_t i = 0; i < positions0.size(); ++i)
                        EXPECT_NEAR(GEO::distance2(positions0[i], positions1[i]), 0.0, EXACT_DIST2_TOL);
                }
            }
        }

        /* == artifacts ======================================================================================== */

        /**
         * @brief Dump the control nodes as a point set, with their quantities when they are defined.
         * @param[in] suffix Suffix of the artifact file name.
         */
        void save_control_nodes(const std::string_view suffix = "_nodes.geogram") const {
            ASSERT_NE(control_grid, nullptr);

            GEO::Mesh mesh_out;
            GEO::Attribute<GEO::index_t> mesh_out_v_idx(mesh_out.vertices.attributes(), "idx");

            mesh_out.vertices.create_vertices(control_grid->control_nodes_nb());
            for (const auto v : mesh_out.vertices) {
                mesh_out.vertices.point(v) = control_grid->control_node(v);
                mesh_out_v_idx[v] = v;
            }

            if (const auto& v_quantities = control_grid->control_nodes_quantities();
                v_quantities.is_bound()
                ) {
                const auto dim = v_quantities.dimension();
                GEO::Attribute<double> mesh_out_v_quantities;
                mesh_out_v_quantities.create_vector_attribute(mesh_out.vertices.attributes(), "quantities", dim);
                for (GEO::index_t v = 0, v_end = control_grid->control_nodes_nb(); v < v_end; ++v) {
                    for (GEO::index_t d = 0; d < dim; ++d)
                        mesh_out_v_quantities[dim*v+d] = v_quantities[dim*v+d];
                }
            }

            EXPECT_TRUE(mesh_out.save(artifact_path(suffix)));
        }

        /**
         * @brief Dump the discretized high-order cell borders of the mesh.
         *
         * The quality measures are stored as per-vertex attributes, together with the source cell
         * and the parametric coordinate of each vertex. The control-node quantities are
         * interpolated on the vertices when they are defined.
         *
         * @param[in] suffix Suffix of the artifact file name.
         */
        void save_high_order_mesh_border(const std::string_view suffix = "_border.geogram") const {
            ASSERT_NE(control_grid, nullptr);

            GEO::Mesh mesh_out;
            GEO::Attribute<GEO::index_t> mesh_out_v_cell(mesh_out.vertices.attributes(), "cell");
            GEO::Attribute<GEO::vec3> mesh_out_v_uvw(mesh_out.vertices.attributes(), "uvw");
            GEO::Attribute<GEO::index_t> mesh_out_f_cell(mesh_out.facets.attributes(), "cell");

            control_grid->append_discretized_high_order_cells_border(
                mesh_out,
                DISCRETIZATION_RESOLUTION,
                &mesh_out_v_cell,
                &mesh_out_v_uvw,
                &mesh_out_f_cell);

            save_vertices_quality(mesh_out, mesh_out_v_cell, mesh_out_v_uvw);

            EXPECT_TRUE(mesh_out.save(artifact_path(suffix)));
        }

        /**
         * @brief Dump the discretized high-order cells of the mesh.
         *
         * The quality measures are stored as per-vertex attributes, together with the source cell
         * and the parametric coordinate of each vertex. The control-node quantities are
         * interpolated on the vertices when they are defined.
         *
         * @param[in] suffix Suffix of the artifact file name.
         */
        void save_high_order_mesh_cells(const std::string_view suffix = "_cells.geogram") const {
            ASSERT_NE(control_grid, nullptr);

            GEO::Mesh mesh_out;
            GEO::Attribute<GEO::index_t> mesh_out_v_cell(mesh_out.vertices.attributes(), "cell");
            GEO::Attribute<GEO::vec3> mesh_out_v_uvw(mesh_out.vertices.attributes(), "uvw");
            GEO::Attribute<GEO::index_t> mesh_out_c_cell(mesh_out.cells.attributes(), "cell");

            control_grid->append_discretized_high_order_cells(
                mesh_out,
                DISCRETIZATION_RESOLUTION,
                &mesh_out_v_cell,
                &mesh_out_v_uvw,
                &mesh_out_c_cell);

            save_vertices_quality(mesh_out, mesh_out_v_cell, mesh_out_v_uvw);

            EXPECT_TRUE(mesh_out.save(artifact_path(suffix)));
        }

        /* == data ============================================================================================= */

        GEO::Mesh mesh;
        std::unique_ptr<Grid> control_grid;

    private:
        /* == helpers of the conformity and artifact checks ==================================================== */

        /**
         * @brief Global vertices of a local facet of a cell.
         * @param[in] c Cell index.
         * @param[in] lf Local facet index.
         * @return The four vertex indices, sorted.
         */
        [[nodiscard]] std::array<GEO::index_t, 4> facet_vertices(const GEO::index_t c, const GEO::index_t lf) const {
            std::array<GEO::index_t, 4> vertices{};
            for (GEO::index_t k = 0; k < 4; ++k)
                vertices[k] = mesh.cells.facet_vertex(c, lf, k);
            std::ranges::sort(vertices);
            return vertices;
        }

        /**
         * @brief Positions of the control nodes of a local facet of a cell, sorted lexicographically.
         * @param[in] c Cell index.
         * @param[in] lf Local facet index.
         * @return The node positions, sorted along (x, y, z).
         */
        [[nodiscard]] std::vector<GEO::vec3> facet_control_node_positions(const GEO::index_t c, const GEO::index_t lf) const {
            std::vector<GEO::vec3> positions;
            positions.reserve((control_grid->order()+1)*(control_grid->order()+1));
            for (GEO::index_t i = 0, order = control_grid->order(); i <= order; ++i)
                for (GEO::index_t j = 0; j <= order; ++j)
                    positions.push_back(control_grid->control_node(control_grid->cell_facet_nd(c, lf, i, j)));

            std::ranges::sort(positions, [](const GEO::vec3& p, const GEO::vec3& q) {
                if (p.x != q.x) return p.x < q.x;
                if (p.y != q.y) return p.y < q.y;
                return p.z < q.z;
            });
            return positions;
        }

        /**
         * @brief Store every quality measure of a discretized mesh as per-vertex attributes.
         * @param[in] mesh_out Discretized mesh, already annotated with "cell" and "uvw".
         * @param[in] mesh_out_v_cell Source cell of each vertex.
         * @param[in] mesh_out_v_uvw Parametric coordinate of each vertex.
         */
        void save_vertices_quality(
            GEO::Mesh& mesh_out,
            const GEO::Attribute<GEO::index_t>& mesh_out_v_cell,
            const GEO::Attribute<GEO::vec3>& mesh_out_v_uvw
            ) const {
            MeshQualityAttributes quality(mesh_out, false); // no absolute-area measure for hexes
            for (const auto v : mesh_out.vertices)
                quality.set(v, evaluate_quality(mesh_out_v_cell[v], mesh_out_v_uvw[v]));

            if (const auto& v_quantities = control_grid->control_nodes_quantities();
                v_quantities.is_bound()
                ) {
                const auto dim = v_quantities.dimension();
                GEO::Attribute<double> mesh_out_v_quantities;
                mesh_out_v_quantities.create_vector_attribute(mesh_out.vertices.attributes(), "quantities", dim);
                for (const auto v : mesh_out.vertices) {
                    control_grid->compute_cell_uvw_quantities(
                        mesh_out_v_cell[v],
                        mesh_out_v_uvw[v],
                        &mesh_out_v_quantities[dim*v]);
                }
            }
        }
    };

    /**
     * @brief Fixture of the tests running on a mesh made of a single axis-aligned unit hex.
     */
    class SingleHexControlGridTest : public HexControlGridTest {
    protected:
        static constexpr GEO::index_t ORDER = 5;

        void SetUp() override {
            create_vertices(UNIT_HEX_CORNERS);
            mesh.cells.create_hex(0, 1, 2, 3, 4, 5, 6, 7);

            control_grid = std::make_unique<Grid>(mesh, ORDER);
        }
    };

    TEST_F(SingleHexControlGridTest, regular) {
        // The axis-aligned unit hex is the ideal element: the mapping is the identity and every
        // quality measure sits at its optimum.
        expect_mapping_interpolates_control_nodes(CELL);
        expect_identity_mapping(CELL);

        for (const auto& uvw : grid_unit_samples_3d(SAMPLE_RESOLUTION)) {
            const auto quality = evaluate_quality(CELL, uvw);
            expect_quality_invariants(quality, parametric_context("uvw", uvw));

            EXPECT_NEAR(quality.det_jacobian, 1.0, REFERENCE_TOL);
            EXPECT_NEAR(quality.scaled_jacobian, 1.0, REFERENCE_TOL);
            EXPECT_NEAR(quality.inverse_mean_ratio, 1.0, REFERENCE_TOL);
            EXPECT_NEAR(quality.MIPS, 1.0, REFERENCE_TOL);
        }

        save_control_nodes();
        save_high_order_mesh_border();
        save_high_order_mesh_cells();
    }

    TEST_F(SingleHexControlGridTest, random) {
        jiggle_control_node(control_grid->cell_edge_nd(CELL, 1, 2));
        jiggle_control_node(control_grid->cell_facet_nd(CELL, 2, 2, 3));
        jiggle_control_node(control_grid->cell_nd(CELL, 1, 2, 3));

        expect_mapping_interpolates_control_nodes(CELL);
        expect_mapping_moved_at_node(CELL, 1, 2, 3);
        expect_quality_invariants_over_grid(CELL, SAMPLE_RESOLUTION);

        save_control_nodes();
        save_high_order_mesh_border();
        save_high_order_mesh_cells();
    }

    TEST_F(SingleHexControlGridTest, quantities) {
        constexpr GEO::index_t QUANTITY_NB = 6;

        jiggle_control_node(control_grid->cell_edge_nd(CELL, 1, 2));
        jiggle_control_node(control_grid->cell_facet_nd(CELL, 2, 2, 3));

        control_grid->create_control_node_quantities(QUANTITY_NB);
        EXPECT_EQ(control_grid->control_node_quantities_dimension(), QUANTITY_NB);

        auto& quantities = control_grid->control_nodes_quantities();
        ASSERT_TRUE(quantities.is_bound());
        for (GEO::index_t v = 0, v_end = control_grid->control_nodes_nb(); v < v_end; ++v) {
            for (GEO::index_t d = 0; d < QUANTITY_NB; ++d)
                quantities[QUANTITY_NB*v+d] = static_cast<double>(d+1)*GEO::Numeric::random_float32();
        }

        // At the parametric position of a control node, the interpolation reduces to the nodal value.
        const auto& node_positions = control_grid->node_positions_1D();
        for (GEO::index_t i = 0, i_end = control_grid->order(); i <= i_end; ++i) {
            for (GEO::index_t j = 0, j_end = control_grid->order(); j <= j_end; ++j) {
                for (GEO::index_t k = 0, k_end = control_grid->order(); k <= k_end; ++k) {
                    const GEO::vec3 uvw(node_positions[i], node_positions[j], node_positions[k]);
                    const auto nd = control_grid->cell_nd(CELL, i, j, k);

                    for (GEO::index_t d = 0; d < QUANTITY_NB; ++d) {
                        SCOPED_TRACE(::testing::Message()
                            << "control node (" << i << ", " << j << ", " << k << "), component " << d);
                        EXPECT_NEAR(
                            control_grid->compute_cell_uvw_quantity(CELL, uvw, d),
                            quantities[QUANTITY_NB*nd+d],
                            REFERENCE_TOL
                            );
                    }
                }
            }
        }

        // The single-component and all-components evaluation must agree.
        constexpr GEO::index_t CHECK_RESOLUTION = 3;
        for (const auto& uvw : grid_unit_samples_3d(CHECK_RESOLUTION)) {
            std::vector<double> q(QUANTITY_NB);
            control_grid->compute_cell_uvw_quantities(CELL, uvw, q.data());

            for (GEO::index_t d = 0; d < QUANTITY_NB; ++d) {
                EXPECT_NEAR(q[d], control_grid->compute_cell_uvw_quantity(CELL, uvw, d), REFERENCE_TOL)
                    << "uvw = (" << uvw.x << ", " << uvw.y << ", " << uvw.z << "), component " << d;
            }
        }

        save_control_nodes();
        save_high_order_mesh_border();
        save_high_order_mesh_cells();
    }

    TEST_F(SingleHexControlGridTest, facet_normal) {
        constexpr GEO::index_t POINTS_NB = 100;
        constexpr GEO::index_t LF = 1;
        constexpr GEO::index_t LF_SAMPLE_RESOLUTION = 5;
        constexpr double ARROW_LENGTH = 0.2;

        // Bend the sampled facet by moving two of its control nodes out of the facet plane.
        jiggle_control_node(control_grid->cell_facet_nd(CELL, LF, 2, 3));
        jiggle_control_node(control_grid->cell_facet_nd(CELL, LF, 3, 1));

        const auto centre = cell_centre(CELL);

        GEO::Mesh mesh_out;
        GEO::index_t new_v = mesh_out.vertices.create_vertices(2*POINTS_NB);
        GEO::index_t new_e = mesh_out.edges.create_edges(POINTS_NB);
        for (GEO::index_t i = 0; i < POINTS_NB; ++i) {
            const GEO::vec2 uv(GEO::Numeric::random_float32(), GEO::Numeric::random_float32());
            const auto position = control_grid->compute_cell_uvw_position(CELL, project_hex_lf_uv_to_uvw(uv, LF));
            const auto normal = control_grid->compute_cell_facet_uv_normal(CELL, LF, uv);

            mesh_out.vertices.point(new_v) = position;
            mesh_out.vertices.point(new_v+1) = position + ARROW_LENGTH*normal;
            mesh_out.edges.set_vertex(new_e, 0, new_v);
            mesh_out.edges.set_vertex(new_e, 1, new_v+1);

            new_v += 2;
            ++new_e;
        }

        // The normal of a facet must be orthogonal to the facet, non-degenerate, and point outward.
        for (GEO::index_t lf = 0; lf < HEX_FACET_NB; ++lf) {
            for (GEO::index_t i = 1; i < LF_SAMPLE_RESOLUTION; ++i) {
                for (GEO::index_t j = 1; j < LF_SAMPLE_RESOLUTION; ++j) {
                    SCOPED_TRACE(::testing::Message() << "lf = " << lf << ", sample (" << i << ", " << j << ")");

                    const GEO::vec2 uv(
                        static_cast<double>(i)/LF_SAMPLE_RESOLUTION,
                        static_cast<double>(j)/LF_SAMPLE_RESOLUTION);
                    const auto normal = control_grid->compute_cell_facet_uv_normal(CELL, lf, uv);
                    const auto [tu, tv] = finite_difference_facet_tangents(*control_grid, CELL, lf, uv);

                    EXPECT_GT(GEO::length(normal), 0.0);
                    EXPECT_NEAR(GEO::dot(GEO::normalize(normal), GEO::normalize(tu)), 0.0, NORMAL_ORTHOGONALITY_TOL);
                    EXPECT_NEAR(GEO::dot(GEO::normalize(normal), GEO::normalize(tv)), 0.0, NORMAL_ORTHOGONALITY_TOL);

                    const auto position = control_grid->compute_cell_uvw_position(CELL, project_hex_lf_uv_to_uvw(uv, lf));
                    EXPECT_GT(GEO::dot(GEO::normalize(normal), GEO::normalize(position - centre)), 0.0);
                }
            }
        }

        control_grid->append_discretized_high_order_cells_border(mesh_out, BACKGROUND_RESOLUTION);
        EXPECT_TRUE(mesh_out.save(artifact_path(".geogram")));
    }

    TEST_F(SingleHexControlGridTest, dudvdw) {
        constexpr GEO::index_t RESOLUTION = 5;
        constexpr GEO::index_t POINTS_NB = (RESOLUTION+1)*(RESOLUTION+1)*(RESOLUTION+1);
        constexpr double ARROW_LENGTH = 0.05;

        jiggle_control_node(control_grid->cell_nd(CELL, 1, 2, 3));
        jiggle_control_node(control_grid->cell_nd(CELL, 3, 0, 2));

        expect_derivatives_match_finite_differences(CELL);

        // Dump the three tangent vectors as arrows, over a regular grid of parametric samples.
        GEO::Mesh mesh_out;
        GEO::Attribute<GEO::index_t> mesh_out_e_axis(mesh_out.edges.attributes(), "axis");
        GEO::index_t new_v = mesh_out.vertices.create_vertices(4*POINTS_NB);
        GEO::index_t new_e = mesh_out.edges.create_edges(3*POINTS_NB);
        for (const auto& uvw : grid_unit_samples_3d(RESOLUTION)) {
            const auto derivatives = evaluate_derivatives(CELL, uvw);
            const auto position = control_grid->compute_cell_uvw_position(CELL, uvw);

            mesh_out.vertices.point(new_v) = position;
            mesh_out.vertices.point(new_v+1) = position + ARROW_LENGTH*derivatives.du;
            mesh_out.vertices.point(new_v+2) = position + ARROW_LENGTH*derivatives.dv;
            mesh_out.vertices.point(new_v+3) = position + ARROW_LENGTH*derivatives.dw;
            mesh_out.edges.set_vertex(new_e, 0, new_v);
            mesh_out.edges.set_vertex(new_e, 1, new_v+1);
            mesh_out.edges.set_vertex(new_e+1, 0, new_v);
            mesh_out.edges.set_vertex(new_e+1, 1, new_v+2);
            mesh_out.edges.set_vertex(new_e+2, 0, new_v);
            mesh_out.edges.set_vertex(new_e+2, 1, new_v+3);
            mesh_out_e_axis[new_e] = 0;
            mesh_out_e_axis[new_e+1] = 1;
            mesh_out_e_axis[new_e+2] = 2;

            new_v += 4;
            new_e += 3;
        }

        control_grid->append_discretized_high_order_cells(mesh_out, BACKGROUND_RESOLUTION);
        EXPECT_TRUE(mesh_out.save(artifact_path(".geogram")));
    }

    /**
     * @brief Fixture of the tests running on the mesh made of two unit hexes sharing a facet.
     */
    class TwoHexControlGridTest : public HexControlGridTest {
    protected:
        static constexpr GEO::index_t ORDER = 6;

        void SetUp() override {
            create_vertices(TWO_HEX_CORNERS);
            mesh.cells.create_hex(0, 1, 2, 3, 4, 5, 6, 7);
            mesh.cells.create_hex(2, 8, 6, 10, 3, 9, 7, 11);
            mesh.cells.connect();

            control_grid = std::make_unique<Grid>(mesh, ORDER);
        }
    };

    TEST_F(TwoHexControlGridTest, regular) {
        for (const GEO::index_t c : {0, 1}) {
            SCOPED_TRACE(::testing::Message() << "cell " << c);
            expect_mapping_interpolates_control_nodes(c);
            expect_quality_invariants_over_grid(c, SAMPLE_RESOLUTION);
        }
        expect_identity_mapping(0);

        // The cells are conforming: they share the control nodes of their common vertex and facet.
        expect_shared_vertices_are_conforming(0, 1);
        expect_shared_facets_are_conforming(0, 1);

        save_control_nodes();
        save_high_order_mesh_border();
        save_high_order_mesh_cells();
    }

    TEST_F(TwoHexControlGridTest, random) {
        jiggle_control_node(control_grid->cell_edge_nd(0, 2, 3));
        jiggle_control_node(control_grid->cell_facet_nd(1, 0, 2, 4));
        jiggle_control_node(control_grid->cell_nd(1, 2, 4, 1));

        for (const GEO::index_t c : {0, 1}) {
            SCOPED_TRACE(::testing::Message() << "cell " << c);
            expect_mapping_interpolates_control_nodes(c);
            expect_quality_invariants_over_grid(c, SAMPLE_RESOLUTION);
        }

        // Jittering a node of the shared facet must keep the two cells conforming.
        expect_shared_facets_are_conforming(0, 1);

        save_control_nodes();
        save_high_order_mesh_border();
        save_high_order_mesh_cells();
    }
}
