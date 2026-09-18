//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/4.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
// Tests of QuadControlGrid<DIM>, in two parts:
//   * the parametric projection helpers, checked as properties of the reference unit quad
//     (QuadVertexProjectionTest / QuadEdgeProjectionTest);
//   * the control grid itself: high-order mapping, parametric derivatives, facet normals,
//     control-node quantities and facet quality measures, on a single quad (order 4) and on a
//     two-quad mesh (order 5).
// Every geometric case is also dumped as a .geogram artifact named after the running test
// ("test_<suite>_<test>*.geogram" in the working directory) for visual inspection in geobox.
//
#include <geolio/high_order/quad_control_grid.h>
#include <gtest/gtest.h>
#include "control_grid_test_utils.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
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

        /** Local topology of a quadrilateral. */
        constexpr GEO::index_t QUAD_VERTEX_NB = 4;
        constexpr GEO::index_t QUAD_EDGE_NB = 4;

        /** Index of the reference facet of the test meshes (their first facet is the unit quad). */
        constexpr GEO::index_t FACET = 0;

        /** Resolution of the parametric grids used by the quality-measure checks. */
        constexpr GEO::index_t SAMPLE_RESOLUTION = 20;

        /** Resolution of the discretized high-order facets dumped as artifacts. */
        constexpr GEO::index_t DISCRETIZATION_RESOLUTION = 20;

        /** Resolution of the finer background mesh dumped behind the derivative/normal arrows. */
        constexpr GEO::index_t BACKGROUND_RESOLUTION = 30;

        /** Amplitude of the random jitter applied to the control nodes of the "random" cases. */
        constexpr double JITTER_AMPLITUDE = 0.1;

        /**
         * Largest Jacobian determinant accepted for a degenerate configuration.
         *
         * The Jacobian of the reference element is 1, so a degenerate mapping must get far below
         * that value (it vanishes exactly at the degeneracy, but a sample grid only gets close).
         */
        constexpr double DEGENERATE_DET_JACOBIAN_TOL = 1e-2;

        /** Step of the central finite differences used to check the analytic derivatives. */
        constexpr double FINITE_DIFF_STEP = 1e-6;

        /** Tolerance on the squared distance between analytic and finite-difference derivatives. */
        constexpr double FINITE_DIFF_DIST2_TOL = 1e-16;

        /** Corners of the reference quad (counter-clockwise: local vertex i -> corner i). */
        const std::array<GEO::vec2, QUAD_VERTEX_NB> UNIT_QUAD_CORNERS = {
            GEO::vec2(0.0, 0.0), GEO::vec2(1.0, 0.0), GEO::vec2(1.0, 1.0), GEO::vec2(0.0, 1.0)
        };

        /** Corners of the two-quad mesh: the unit square, plus the unit square (1,0)-(2,1). */
        const std::array<GEO::vec2, 6> TWO_QUAD_CORNERS = {
            GEO::vec2(0.0, 0.0), GEO::vec2(1.0, 0.0), GEO::vec2(1.0, 1.0),
            GEO::vec2(0.0, 1.0), GEO::vec2(2.0, 0.0), GEO::vec2(2.0, 1.0)
        };

        /* == shared helpers =================================================================================== */

        /**
         * @brief Offset of a control-node line out of the facet plane.
         *
         * A 3D facet can bulge out of its plane, in the z direction. A 2D facet has no such
         * direction, so the equivalent profile is sheared along v instead: both preserve the
         * orientation of the mapping, which is what the "not inverted" cases rely on.
         *
         * @param[in] amplitude Signed displacement of the line.
         * @return The offset to add to every control node of the line.
         */
        template <GEO::index_t DIM>
        [[nodiscard]] GEO::vecng<DIM, double> bulge_offset(const double amplitude) {
            GEO::vecng<DIM, double> offset;
            if constexpr (DIM == 3)
                offset[2] = amplitude;
            else
                offset[1] = amplitude;
            return offset;
        }
    }

    /* == parametric projections =============================================================================== */

    /**
     * @brief Reference quad of the projection tests.
     *
     * The mesh is the unit square [0,1]^2, whose local vertex i sits exactly on the corner
     * project_quad_lv_to_uv(i) and whose local edge i joins the local vertices i and i+1.
     */
    class UnitQuadTest : public ::testing::Test {
    protected:
        void SetUp() override {
            mesh.vertices.set_dimension(2);
            mesh.vertices.create_vertices(QUAD_VERTEX_NB);
            for (GEO::index_t lv = 0; lv < QUAD_VERTEX_NB; ++lv)
                mesh.vertices.point<2>(lv) = UNIT_QUAD_CORNERS[lv];
            mesh.facets.create_quad(0, 1, 2, 3);
        }

        /**
         * @brief Endpoints of a local edge of the reference facet.
         * @param[in] le Local edge index, 0,1,2,3.
         * @return The endpoints, oriented from local vertex \p le to local vertex \p le + 1.
         */
        [[nodiscard]] std::pair<GEO::vec2, GEO::vec2> facet_edge(const GEO::index_t le) const {
            return {
                mesh.facets.point<2>(FACET, le),
                mesh.facets.point<2>(FACET, (le + 1) % QUAD_EDGE_NB)
            };
        }

        GEO::Mesh mesh;
    };

    /** Fixture parametrized by the local vertex index of the reference quad. */
    class QuadVertexProjectionTest : public UnitQuadTest, public ::testing::WithParamInterface<GEO::index_t> {
    protected:
        void SetUp() override {
            UnitQuadTest::SetUp();
            lv = GetParam();
            ASSERT_LT(lv, QUAD_VERTEX_NB);
        }

        GEO::index_t lv = 0;
    };

    TEST_P(QuadVertexProjectionTest, projects_vertex_to_parametric_corner) {
        const auto uv = project_quad_lv_to_uv(lv);
        const auto& p = mesh.vertices.point<2>(mesh.facets.vertex(FACET, lv));

        EXPECT_NEAR(GEO::distance2(p, uv), 0.0, EXACT_DIST2_TOL);
        EXPECT_NEAR(GEO::distance2(uv, UNIT_QUAD_CORNERS[lv]), 0.0, EXACT_DIST2_TOL);
    }

    TEST_P(QuadVertexProjectionTest, projects_corner_to_edge_endpoints) {
        const auto uv = project_quad_lv_to_uv(lv);

        // A corner lies on the supporting line of every edge, at one of its two endpoints.
        for (GEO::index_t le = 0; le < QUAD_EDGE_NB; ++le) {
            SCOPED_TRACE(::testing::Message() << "le = " << le);
            const double t = project_uv_to_quad_le_t(uv, le);

            EXPECT_GE(t, 0.0);
            EXPECT_LE(t, 1.0);
            EXPECT_NEAR(t*(1.0 - t), 0.0, EXACT_DIST2_TOL) << "t = " << t;
        }
    }

    INSTANTIATE_TEST_SUITE_P(
        AllLocalVertices,
        QuadVertexProjectionTest,
        ::testing::Range<GEO::index_t>(0, QUAD_VERTEX_NB)
        );

    /** Fixture parametrized by the local edge index of the reference quad. */
    class QuadEdgeProjectionTest : public UnitQuadTest, public ::testing::WithParamInterface<GEO::index_t> {
    protected:
        void SetUp() override {
            UnitQuadTest::SetUp();
            le = GetParam();
            ASSERT_LT(le, QUAD_EDGE_NB);
            uvs = random_unit_samples_2d(RANDOM_SAMPLE_NB);
            ts = random_unit_samples_1d(RANDOM_SAMPLE_NB);
        }

        GEO::index_t le = 0;
        std::vector<GEO::vec2> uvs;
        std::vector<double> ts;
    };

    TEST_P(QuadEdgeProjectionTest, projects_uv_to_closest_edge_parameter) {
        const auto [ep0, ep1] = facet_edge(le);
        const auto edge = ep1 - ep0;
        const double edge_length2 = GEO::dot(edge, edge);

        for (const auto& uv : uvs) {
            const double t = project_uv_to_quad_le_t(uv, le);
            EXPECT_GE(t, 0.0);
            EXPECT_LE(t, 1.0);

            // The projected point is the foot of the perpendicular dropped from uv on the edge.
            EXPECT_NEAR(GEO::dot(edge, uv - ((1.0 - t)*ep0 + t*ep1)), 0.0, REFERENCE_TOL);

            // ... which is the closest point of the segment, recomputed independently.
            const double t_closest = std::clamp(GEO::dot(uv - ep0, edge) / edge_length2, 0.0, 1.0);
            EXPECT_NEAR(t, t_closest, REFERENCE_TOL);
        }
    }

    TEST_P(QuadEdgeProjectionTest, projects_edge_parameter_to_uv) {
        const auto [ep0, ep1] = facet_edge(le);

        for (const double t : ts) {
            const auto uv = project_quad_le_t_to_uv(t, le);
            EXPECT_GE(uv.x, 0.0);
            EXPECT_LE(uv.x, 1.0);
            EXPECT_GE(uv.y, 0.0);
            EXPECT_LE(uv.y, 1.0);

            // The parametric point lies on the edge ...
            EXPECT_NEAR(GEO::distance2(uv, (1.0 - t)*ep0 + t*ep1), 0.0, EXACT_DIST2_TOL);

            // ... and the two projections are inverse of each other on the edge.
            EXPECT_NEAR(project_uv_to_quad_le_t(uv, le), t, REFERENCE_TOL);
        }
    }

    INSTANTIATE_TEST_SUITE_P(
        AllLocalEdges,
        QuadEdgeProjectionTest,
        ::testing::Range<GEO::index_t>(0, QUAD_EDGE_NB)
        );

    TEST_F(UnitQuadTest, edge_endpoints_match_facet_vertices) {
        for (GEO::index_t le = 0; le < QUAD_EDGE_NB; ++le) {
            SCOPED_TRACE(::testing::Message() << "le = " << le);
            const auto [ep0, ep1] = facet_edge(le);

            EXPECT_NEAR(GEO::distance2(project_quad_le_t_to_uv(0.0, le), ep0), 0.0, EXACT_DIST2_TOL);
            EXPECT_NEAR(GEO::distance2(project_quad_le_t_to_uv(1.0, le), ep1), 0.0, EXACT_DIST2_TOL);
            EXPECT_NEAR(project_uv_to_quad_le_t(ep0, le), 0.0, EXACT_DIST2_TOL);
            EXPECT_NEAR(project_uv_to_quad_le_t(ep1, le), 1.0, EXACT_DIST2_TOL);
        }
    }

    /* == control grid ========================================================================================= */

    /**
     * @brief Common helpers of the QuadControlGrid tests.
     * @tparam DIM Physical dimension of the reference mesh, 2 or 3.
     */
    template <GEO::index_t DIM>
    class QuadControlGridTest : public ::testing::Test {
    protected:
        using Grid = QuadControlGrid<DIM>;
        using Vec = GEO::vecng<DIM, double>;

        /* == meshes =========================================================================================== */

        /**
         * @brief Create the vertices of the reference mesh from planar positions.
         *
         * The positions are used as (x, y), the z coordinate being zero when DIM == 3.
         * @param[in] planar_positions One position per vertex.
         */
        void create_vertices(const std::span<const GEO::vec2> planar_positions) {
            mesh.vertices.set_dimension(DIM);
            mesh.vertices.create_vertices(static_cast<GEO::index_t>(planar_positions.size()));
            for (GEO::index_t v = 0; v < planar_positions.size(); ++v) {
                if constexpr (DIM == 2)
                    mesh.vertices.point<2>(v) = planar_positions[v];
                else
                    mesh.vertices.point<3>(v) = GEO::vec3(planar_positions[v].x, planar_positions[v].y, 0.0);
            }
        }

        /* == perturbations ==================================================================================== */

        /**
         * @brief Add a random jitter to one control node.
         *
         * The jitter makes the tested configuration generic. For DIM == 3 an additional
         * deterministic displacement can be applied out of the facet plane, so that the node is
         * guaranteed to have moved.
         *
         * @param[in] nd Global index of the control node.
         * @param[in] out_of_plane_bias Signed displacement along z (ignored when DIM == 2).
         */
        void jiggle_control_node(const GEO::index_t nd, const double out_of_plane_bias = 0.0) {
            auto& p = control_grid->control_node(nd);
            for (GEO::index_t d = 0; d < DIM; ++d)
                p[d] += JITTER_AMPLITUDE*GEO::Numeric::random_float32();
            if constexpr (DIM == 3)
                p[2] += out_of_plane_bias;
        }

        /* == evaluations ====================================================================================== */

        /**
         * @brief Evaluate every quality measure of a facet at one parametric point.
         * @param[in] f Facet index.
         * @param[in] uv Parametric point in [0,1]^2.
         * @return The measure values.
         */
        [[nodiscard]] Quality evaluate_quality(const GEO::index_t f, const GEO::vec2& uv) const {
            return {
                .det_jacobian = control_grid->compute_facet_uv_measure(f, uv, Grid::MeasureType::DET_JACOBIAN),
                .absolute_sq_area = control_grid->compute_facet_uv_measure(f, uv, Grid::MeasureType::ABSOLUTE_SQ_AREA),
                .scaled_jacobian = control_grid->compute_facet_uv_measure(f, uv, Grid::MeasureType::SCALED_JACOBIAN),
                .inverse_mean_ratio = control_grid->compute_facet_uv_measure(f, uv, Grid::MeasureType::INVERSE_MEAN_RATIO),
                .MIPS = control_grid->compute_facet_uv_measure(f, uv, Grid::MeasureType::MIPS)
            };
        }

        /** First-order parametric data of a facet mapping, at one parametric point. */
        struct Derivatives {
            Vec du;
            Vec dv;
            std::vector<double> Bu;
            std::vector<double> Bv;
            std::vector<double> dBu;
            std::vector<double> dBv;
        };

        /**
         * @brief Evaluate the parametric derivatives of a facet mapping.
         * @param[in] f Facet index.
         * @param[in] uv Parametric point in [0,1]^2.
         * @return The tangents and the 1D basis values/derivatives used to build them.
         */
        [[nodiscard]] Derivatives evaluate_derivatives(const GEO::index_t f, const GEO::vec2& uv) const {
            Derivatives derivatives;
            control_grid->compute_facet_uv_dudv(
                f, uv,
                derivatives.du, derivatives.dv,
                derivatives.Bu, derivatives.Bv,
                derivatives.dBu, derivatives.dBv);
            return derivatives;
        }

        /* == checks =========================================================================================== */

        /**
         * @brief Check the quality-measure invariants on a regular grid of samples.
         * @param[in] f Facet index.
         * @param[in] resolution Number of subdivisions of each parametric direction.
         */
        void expect_quality_invariants_over_grid(const GEO::index_t f, const GEO::index_t resolution) const {
            for (const auto& uv : grid_unit_samples_2d(resolution))
                expect_quality_invariants(evaluate_quality(f, uv), parametric_context("uv", uv));
        }

        /**
         * @brief Check that the mapping interpolates the control nodes of a facet.
         *
         * This is the Lagrange delta property: at the parametric position of the local control
         * node (i, j), the mapping reduces exactly to that node's position, whatever the
         * (possibly perturbed) control-node coordinates are.
         *
         * @param[in] f Facet index.
         */
        void expect_mapping_interpolates_control_nodes(const GEO::index_t f) const {
            const auto& node_positions = control_grid->node_positions_1D();
            for (GEO::index_t i = 0, i_end = control_grid->order(); i <= i_end; ++i) {
                for (GEO::index_t j = 0, j_end = control_grid->order(); j <= j_end; ++j) {
                    SCOPED_TRACE(::testing::Message() << "control node (" << i << ", " << j << ")");

                    const GEO::vec2 uv(node_positions[i], node_positions[j]);
                    const auto& nd = control_grid->facet_nd(f, i, j);
                    EXPECT_NEAR(
                        GEO::distance2(control_grid->compute_facet_uv_position(f, uv), control_grid->control_node(nd)),
                        0.0,
                        EXACT_DIST2_TOL
                        );
                }
            }
        }

        /**
         * @brief Check that the mapping of an unperturbed axis-aligned unit quad is the identity.
         * @param[in] f Facet index of a facet whose corners are the unit quad corners.
         */
        void expect_identity_mapping(const GEO::index_t f) const {
            for (const auto& uv : grid_unit_samples_2d(SAMPLE_RESOLUTION)) {
                Vec reference;
                reference[0] = uv.x;
                reference[1] = uv.y;
                EXPECT_NEAR(GEO::distance2(control_grid->compute_facet_uv_position(f, uv), reference), 0.0, EXACT_DIST2_TOL);
            }
        }

        /**
         * @brief Check that the mapping has moved at the parametric position of a perturbed node.
         *
         * The reference (unperturbed) configuration of the test meshes is the identity mapping,
         * so the mapping at the node position must no longer be the node's original corner.
         *
         * @param[in] f Facet index.
         * @param[in] i First local control-node index of the perturbed node.
         * @param[in] j Second local control-node index of the perturbed node.
         */
        void expect_mapping_moved_at_node(const GEO::index_t f, const GEO::index_t i, const GEO::index_t j) const {
            const auto& node_positions = control_grid->node_positions_1D();
            const GEO::vec2 uv(node_positions[i], node_positions[j]);

            Vec reference;
            reference[0] = uv.x;
            reference[1] = uv.y;

            EXPECT_GT(GEO::distance2(control_grid->compute_facet_uv_position(f, uv), reference), 0.0)
                << "the jitter of control node (" << i << ", " << j << ") must move the mapping";
        }

        /**
         * @brief Check the analytic derivatives against central finite differences, and the 1D bases.
         *
         * The finite differences are evaluated on the interior of the parametric domain, so that
         * uv +- h stays in [0, 1]. The 1D Lagrange basis values must sum to one (partition of
         * unity) and their derivatives to zero.
         *
         * @param[in] f Facet index.
         */
        void expect_derivatives_match_finite_differences(const GEO::index_t f) const {
            constexpr GEO::index_t RESOLUTION = 10;

            for (GEO::index_t i = 1; i < RESOLUTION; ++i) {
                for (GEO::index_t j = 1; j < RESOLUTION; ++j) {
                    SCOPED_TRACE(::testing::Message() << "sample (" << i << ", " << j << ")");

                    const double u = static_cast<double>(i)/RESOLUTION;
                    const double v = static_cast<double>(j)/RESOLUTION;
                    const auto derivatives = evaluate_derivatives(f, GEO::vec2(u, v));

                    const Vec du_fd = (
                        control_grid->compute_facet_uv_position(f, GEO::vec2(u + FINITE_DIFF_STEP, v)) -
                        control_grid->compute_facet_uv_position(f, GEO::vec2(u - FINITE_DIFF_STEP, v))
                        ) / (2.0*FINITE_DIFF_STEP);
                    const Vec dv_fd = (
                        control_grid->compute_facet_uv_position(f, GEO::vec2(u, v + FINITE_DIFF_STEP)) -
                        control_grid->compute_facet_uv_position(f, GEO::vec2(u, v - FINITE_DIFF_STEP))
                        ) / (2.0*FINITE_DIFF_STEP);

                    EXPECT_NEAR(GEO::distance2(derivatives.du, du_fd), 0.0, FINITE_DIFF_DIST2_TOL);
                    EXPECT_NEAR(GEO::distance2(derivatives.dv, dv_fd), 0.0, FINITE_DIFF_DIST2_TOL);

                    EXPECT_NEAR(std::accumulate(derivatives.Bu.begin(), derivatives.Bu.end(), 0.0), 1.0, REFERENCE_TOL);
                    EXPECT_NEAR(std::accumulate(derivatives.Bv.begin(), derivatives.Bv.end(), 0.0), 1.0, REFERENCE_TOL);
                    EXPECT_NEAR(std::accumulate(derivatives.dBu.begin(), derivatives.dBu.end(), 0.0), 0.0, REFERENCE_TOL);
                    EXPECT_NEAR(std::accumulate(derivatives.dBv.begin(), derivatives.dBv.end(), 0.0), 0.0, REFERENCE_TOL);
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

            GEO::Mesh mesh_out(DIM);
            GEO::Attribute<GEO::index_t> mesh_out_v_idx(mesh_out.vertices.attributes(), "idx");

            mesh_out.vertices.create_vertices(control_grid->control_nodes_nb());
            for (const auto v : mesh_out.vertices) {
                mesh_out.vertices.point<DIM>(v) = control_grid->control_node(v);
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
         * @brief Dump the discretized high-order facets of the mesh.
         *
         * The quality measures are stored as per-vertex attributes, together with the source
         * facet and the parametric coordinate of each vertex. The control-node quantities are
         * interpolated on the vertices when they are defined.
         *
         * @param[in] suffix Suffix of the artifact file name.
         */
        void save_high_order_mesh_facets(const std::string_view suffix = "_facets.geogram") const {
            ASSERT_NE(control_grid, nullptr);

            GEO::Mesh mesh_out(DIM);
            GEO::Attribute<GEO::index_t> mesh_out_v_facet(mesh_out.vertices.attributes(), "facet");
            GEO::Attribute<GEO::vec2> mesh_out_v_uv(mesh_out.vertices.attributes(), "uv");
            GEO::Attribute<GEO::index_t> mesh_out_f_facet(mesh_out.facets.attributes(), "facet");

            control_grid->append_discretized_high_order_facets(
                mesh_out,
                DISCRETIZATION_RESOLUTION,
                &mesh_out_v_facet,
                &mesh_out_v_uv,
                &mesh_out_f_facet);

            evaluate_vertices_quality(mesh_out, mesh_out_v_facet, mesh_out_v_uv);

            if (const auto& v_quantities = control_grid->control_nodes_quantities();
                v_quantities.is_bound()
                ) {
                const auto dim = v_quantities.dimension();
                GEO::Attribute<double> mesh_out_v_quantities;
                mesh_out_v_quantities.create_vector_attribute(mesh_out.vertices.attributes(), "quantities", dim);
                for (const auto v : mesh_out.vertices) {
                    control_grid->compute_facet_uv_quantities(
                        mesh_out_v_facet[v],
                        mesh_out_v_uv[v],
                        &mesh_out_v_quantities[dim*v]);
                }
            }

            EXPECT_TRUE(mesh_out.save(artifact_path(suffix)));
        }

        /* == data ============================================================================================= */

        GEO::Mesh mesh;
        std::unique_ptr<Grid> control_grid;

    private:
        /**
         * @brief Store every quality measure of a discretized mesh as per-vertex attributes.
         * @param[in] mesh_out Discretized mesh, already annotated with "facet" and "uv".
         * @param[in] mesh_out_v_facet Source facet of each vertex.
         * @param[in] mesh_out_v_uv Parametric coordinate of each vertex.
         */
        void evaluate_vertices_quality(
            GEO::Mesh& mesh_out,
            const GEO::Attribute<GEO::index_t>& mesh_out_v_facet,
            const GEO::Attribute<GEO::vec2>& mesh_out_v_uv
            ) const {
            MeshQualityAttributes quality(mesh_out);
            for (const auto v : mesh_out.vertices)
                quality.set(v, evaluate_quality(mesh_out_v_facet[v], mesh_out_v_uv[v]));
        }
    };

    /**
     * @brief Fixture of the tests running on a mesh made of a single axis-aligned unit quad.
     * @tparam DimType Wrapper carrying the physical dimension, as in DimTypes.
     */
    template <typename DimType>
    class SingleQuadControlGridTest : public QuadControlGridTest<DimType::value> {
    protected:
        static constexpr GEO::index_t DIM = DimType::value;
        static constexpr GEO::index_t ORDER = 4;

        void SetUp() override {
            this->create_vertices(UNIT_QUAD_CORNERS);
            this->mesh.facets.create_quad(0, 1, 2, 3);

            this->control_grid = std::make_unique<QuadControlGrid<DIM>>(this->mesh, ORDER);
        }

        /**
         * @brief Add an offset to every control node of one tensor-product line of the facet.
         * @param[in] i Index of the line along u, 0,1,...,order.
         * @param[in] offset Offset added to each node of the line.
         */
        void offset_line(const GEO::index_t i, const GEO::vecng<DIM, double>& offset) {
            for (GEO::index_t j = 0; j < this->control_grid->control_nodes_nb_per_edge(); ++j)
                this->control_grid->control_node(this->control_grid->facet_nd(FACET, i, j)) += offset;
        }
    };

    TYPED_TEST_SUITE(SingleQuadControlGridTest, DimTypes);

    TYPED_TEST(SingleQuadControlGridTest, regular) {
        // The axis-aligned unit quad is the ideal element: the mapping is the identity and every
        // quality measure sits at its optimum.
        this->expect_mapping_interpolates_control_nodes(FACET);
        this->expect_identity_mapping(FACET);

        for (const auto& uv : grid_unit_samples_2d(SAMPLE_RESOLUTION)) {
            const auto quality = this->evaluate_quality(FACET, uv);
            expect_quality_invariants(quality, parametric_context("uv", uv));

            EXPECT_NEAR(quality.det_jacobian, 1.0, REFERENCE_TOL);
            EXPECT_NEAR(quality.scaled_jacobian, 1.0, REFERENCE_TOL);
            EXPECT_NEAR(quality.inverse_mean_ratio, 1.0, REFERENCE_TOL);
            EXPECT_NEAR(quality.MIPS, 1.0, REFERENCE_TOL);
        }

        this->save_control_nodes();
        this->save_high_order_mesh_facets();
    }

    TYPED_TEST(SingleQuadControlGridTest, random) {
        this->jiggle_control_node(this->control_grid->facet_nd(FACET, 1, 1), +0.2);
        this->jiggle_control_node(this->control_grid->facet_nd(FACET, 1, 3), -0.2);

        this->expect_mapping_interpolates_control_nodes(FACET);
        this->expect_mapping_moved_at_node(FACET, 1, 1);
        this->expect_quality_invariants_over_grid(FACET, SAMPLE_RESOLUTION);

        this->save_control_nodes();
        this->save_high_order_mesh_facets();
    }

    TYPED_TEST(SingleQuadControlGridTest, quantities) {
        constexpr GEO::index_t QUANTITY_NB = 4;

        this->jiggle_control_node(this->control_grid->facet_nd(FACET, 1, 1), +0.2);
        this->jiggle_control_node(this->control_grid->facet_nd(FACET, 1, 3), -0.2);

        this->control_grid->create_control_node_quantities(QUANTITY_NB);
        EXPECT_EQ(this->control_grid->control_node_quantities_dimension(), QUANTITY_NB);

        auto& quantities = this->control_grid->control_nodes_quantities();
        ASSERT_TRUE(quantities.is_bound());
        for (GEO::index_t v = 0, v_end = this->control_grid->control_nodes_nb(); v < v_end; ++v) {
            for (GEO::index_t d = 0; d < QUANTITY_NB; ++d)
                quantities[QUANTITY_NB*v+d] = static_cast<double>(d+1)*GEO::Numeric::random_float32();
        }

        // At the parametric position of a control node, the interpolation reduces to the nodal value.
        const auto& node_positions = this->control_grid->node_positions_1D();
        for (GEO::index_t i = 0, i_end = this->control_grid->order(); i <= i_end; ++i) {
            for (GEO::index_t j = 0, j_end = this->control_grid->order(); j <= j_end; ++j) {
                const GEO::vec2 uv(node_positions[i], node_positions[j]);
                const auto nd = this->control_grid->facet_nd(FACET, i, j);

                for (GEO::index_t d = 0; d < QUANTITY_NB; ++d) {
                    SCOPED_TRACE(::testing::Message() << "control node (" << i << ", " << j << "), component " << d);
                    EXPECT_NEAR(
                        this->control_grid->compute_facet_uv_quantity(FACET, uv, d),
                        quantities[QUANTITY_NB*nd+d],
                        REFERENCE_TOL
                        );
                }
            }
        }

        // The single-component and all-components evaluation must agree.
        constexpr GEO::index_t CHECK_RESOLUTION = 5;
        for (const auto& uv : grid_unit_samples_2d(CHECK_RESOLUTION)) {
            std::vector<double> q(QUANTITY_NB);
            this->control_grid->compute_facet_uv_quantities(FACET, uv, q.data());

            for (GEO::index_t d = 0; d < QUANTITY_NB; ++d) {
                EXPECT_NEAR(q[d], this->control_grid->compute_facet_uv_quantity(FACET, uv, d), REFERENCE_TOL)
                    << parametric_context("uv", uv) << ", component " << d;
            }
        }

        this->save_control_nodes();
        this->save_high_order_mesh_facets();
    }

    TYPED_TEST(SingleQuadControlGridTest, facet_normal) {
        if constexpr (TypeParam::value != 3) {
            GTEST_SKIP() << "facet normals are only defined for DIM == 3, but DIM == " << TypeParam::value;
        }
        else {
            this->jiggle_control_node(this->control_grid->facet_edge_nd(FACET, 3, 0), +0.2);
            this->jiggle_control_node(this->control_grid->facet_edge_nd(FACET, 1, 2), -0.2);

            constexpr GEO::index_t POINTS_NB = 100;
            constexpr double ARROW_LENGTH = 0.2;

            GEO::Mesh mesh_out(TypeParam::value);
            GEO::index_t new_v = mesh_out.vertices.create_vertices(2*POINTS_NB);
            GEO::index_t new_e = mesh_out.edges.create_edges(POINTS_NB);
            for (GEO::index_t i = 0; i < POINTS_NB; ++i) {
                const GEO::vec2 uv(GEO::Numeric::random_float32(), GEO::Numeric::random_float32());
                const auto position = this->control_grid->compute_facet_uv_position(FACET, uv);
                const auto normal = this->control_grid->compute_facet_uv_normal(FACET, uv);

                // The facet normal is the reversed cross product of the two tangents ...
                const auto derivatives = this->evaluate_derivatives(FACET, uv);
                EXPECT_NEAR(
                    GEO::distance2(normal, -GEO::cross(derivatives.du, derivatives.dv)),
                    0.0,
                    EXACT_DIST2_TOL
                    );

                // ... and it points against the reference normal built from the corner nodes.
                EXPECT_LT(GEO::dot(GEO::normalize(normal), this->control_grid->compute_facet_reference_normal(FACET)), 0.0)
                    << parametric_context("uv", uv);

                mesh_out.vertices.point(new_v) = position;
                mesh_out.vertices.point(new_v+1) = position + ARROW_LENGTH*normal;
                mesh_out.edges.set_vertex(new_e, 0, new_v);
                mesh_out.edges.set_vertex(new_e, 1, new_v+1);

                new_v += 2;
                ++new_e;
            }

            this->control_grid->append_discretized_high_order_facets(mesh_out, BACKGROUND_RESOLUTION);
            EXPECT_TRUE(mesh_out.save(artifact_path(".geogram")));
        }
    }

    TYPED_TEST(SingleQuadControlGridTest, dudv) {
        this->jiggle_control_node(this->control_grid->facet_edge_nd(FACET, 1, 0), +0.2);
        this->jiggle_control_node(this->control_grid->facet_edge_nd(FACET, 3, 2), -0.2);

        this->expect_derivatives_match_finite_differences(FACET);

        // Dump the two tangent vectors as arrows, over a regular grid of parametric samples.
        constexpr GEO::index_t RESOLUTION = 10;
        constexpr GEO::index_t POINTS_NB = (RESOLUTION+1)*(RESOLUTION+1);
        constexpr double ARROW_LENGTH = 0.05;

        GEO::Mesh mesh_out(TypeParam::value);
        GEO::Attribute<GEO::index_t> mesh_out_e_axis(mesh_out.edges.attributes(), "axis");
        GEO::index_t new_v = mesh_out.vertices.create_vertices(3*POINTS_NB);
        GEO::index_t new_e = mesh_out.edges.create_edges(2*POINTS_NB);
        for (const auto& uv : grid_unit_samples_2d(RESOLUTION)) {
            const auto derivatives = this->evaluate_derivatives(FACET, uv);
            const auto position = this->control_grid->compute_facet_uv_position(FACET, uv);

            mesh_out.vertices.point<TypeParam::value>(new_v) = position;
            mesh_out.vertices.point<TypeParam::value>(new_v+1) = position + ARROW_LENGTH*derivatives.du;
            mesh_out.vertices.point<TypeParam::value>(new_v+2) = position + ARROW_LENGTH*derivatives.dv;
            mesh_out.edges.set_vertex(new_e, 0, new_v);
            mesh_out.edges.set_vertex(new_e, 1, new_v+1);
            mesh_out.edges.set_vertex(new_e+1, 0, new_v);
            mesh_out.edges.set_vertex(new_e+1, 1, new_v+2);
            mesh_out_e_axis[new_e] = 0;
            mesh_out_e_axis[new_e+1] = 1;

            new_v += 3;
            new_e += 2;
        }

        this->control_grid->append_discretized_high_order_facets(mesh_out, BACKGROUND_RESOLUTION);
        EXPECT_TRUE(mesh_out.save(artifact_path(".geogram")));
    }

    TYPED_TEST(SingleQuadControlGridTest, measure_not_inverse) {
        constexpr GEO::index_t DIM = TypeParam::value;

        // Ridge over the interior control-node lines. Whatever the amplitude, such a profile keeps
        // the orientation of the mapping, hence detJ stays strictly positive (it is exactly 1 for
        // this construction), i.e. the facet is not inverted.
        this->offset_line(1, bulge_offset<DIM>(0.5));
        this->offset_line(2, bulge_offset<DIM>(1.0));
        this->offset_line(3, bulge_offset<DIM>(0.5));

        this->expect_mapping_interpolates_control_nodes(FACET);

        for (const auto& uv : grid_unit_samples_2d(SAMPLE_RESOLUTION)) {
            const auto quality = this->evaluate_quality(FACET, uv);
            expect_quality_invariants(quality, parametric_context("uv", uv));
            EXPECT_GT(quality.det_jacobian, 0.0) << parametric_context("uv", uv);
        }

        this->save_control_nodes();
        this->save_high_order_mesh_facets();
    }

    TYPED_TEST(SingleQuadControlGridTest, measure_inverse) {
        constexpr GEO::index_t DIM = TypeParam::value;

        // Push the u = 0.25 and u = 0.75 lines apart and bulge the facet: the control net folds
        // over itself, so the mapping is inverted on part of the facet.
        this->offset_line(1, axis_offset<DIM>(0, -1.0) + bulge_offset<DIM>(0.5));
        this->offset_line(2, bulge_offset<DIM>(1.0));
        this->offset_line(3, axis_offset<DIM>(0, +1.0) + bulge_offset<DIM>(0.5));

        this->expect_mapping_interpolates_control_nodes(FACET);

        double min_det_jacobian = std::numeric_limits<double>::max();
        double max_det_jacobian = -std::numeric_limits<double>::max();
        for (const auto& uv : grid_unit_samples_2d(SAMPLE_RESOLUTION)) {
            const auto quality = this->evaluate_quality(FACET, uv);
            expect_quality_invariants(quality, parametric_context("uv", uv));

            min_det_jacobian = std::min(min_det_jacobian, quality.det_jacobian);
            max_det_jacobian = std::max(max_det_jacobian, quality.det_jacobian);
        }

        // The fold is local: the facet holds both an inverted and a valid region.
        EXPECT_LT(min_det_jacobian, 0.0);
        EXPECT_GT(max_det_jacobian, 0.0);

        this->save_control_nodes();
        this->save_high_order_mesh_facets();
    }

    TYPED_TEST(SingleQuadControlGridTest, measure_degenerate) {
        // Collapse two control nodes of the same line onto each other.
        this->control_grid->control_node(this->control_grid->facet_nd(FACET, 2, 2)) =
            this->control_grid->control_node(this->control_grid->facet_nd(FACET, 2, 1));

        this->expect_mapping_interpolates_control_nodes(FACET);

        double min_abs_det_jacobian = std::numeric_limits<double>::max();
        double min_det_jacobian = std::numeric_limits<double>::max();
        double max_det_jacobian = -std::numeric_limits<double>::max();
        for (const auto& uv : grid_unit_samples_2d(2*SAMPLE_RESOLUTION)) {
            const auto quality = this->evaluate_quality(FACET, uv);
            expect_quality_invariants(quality, parametric_context("uv", uv));

            min_abs_det_jacobian = std::min(min_abs_det_jacobian, std::abs(quality.det_jacobian));
            min_det_jacobian = std::min(min_det_jacobian, quality.det_jacobian);
            max_det_jacobian = std::max(max_det_jacobian, quality.det_jacobian);
        }

        // The Jacobian is continuous and changes sign across the facet, so it vanishes somewhere
        // in between: the mapping of the collapsed control net is degenerate.
        EXPECT_LT(min_det_jacobian, 0.0);
        EXPECT_GT(max_det_jacobian, 0.0);

        // It also gets far below the nominal value of the reference element (|detJ| ~ 7e-4
        // around (u, v) = (0.425, 0.375), which the sampling grid must be fine enough to see).
        EXPECT_LT(min_abs_det_jacobian, DEGENERATE_DET_JACOBIAN_TOL);

        this->save_control_nodes();
        this->save_high_order_mesh_facets();
    }

    /**
     * @brief Fixture of the tests running on the mesh made of two unit quads sharing an edge.
     * @tparam DimType Wrapper carrying the physical dimension, as in DimTypes.
     */
    template <typename DimType>
    class TwoQuadControlGridTest : public QuadControlGridTest<DimType::value> {
    protected:
        static constexpr GEO::index_t DIM = DimType::value;
        static constexpr GEO::index_t ORDER = 5;

        void SetUp() override {
            this->create_vertices(TWO_QUAD_CORNERS);
            this->mesh.facets.create_quad(0, 1, 2, 3);
            this->mesh.facets.create_quad(5, 2, 1, 4);
            this->mesh.facets.connect();

            this->control_grid = std::make_unique<QuadControlGrid<DIM>>(this->mesh, ORDER);
        }
    };

    TYPED_TEST_SUITE(TwoQuadControlGridTest, DimTypes);

    TYPED_TEST(TwoQuadControlGridTest, regular) {
        this->expect_mapping_interpolates_control_nodes(0);
        this->expect_mapping_interpolates_control_nodes(1);
        this->expect_identity_mapping(0);
        this->expect_quality_invariants_over_grid(0, SAMPLE_RESOLUTION);
        this->expect_quality_invariants_over_grid(1, SAMPLE_RESOLUTION);

        // The facets share their common edge, hence they must also share its control nodes.
        EXPECT_EQ(this->control_grid->facet_vertex_nd(0, 1), this->control_grid->facet_vertex_nd(1, 2));
        EXPECT_EQ(this->control_grid->facet_vertex_nd(0, 2), this->control_grid->facet_vertex_nd(1, 1));
        for (GEO::index_t k = 0, order = this->control_grid->order(); k <= order; ++k) {
            SCOPED_TRACE(::testing::Message() << "edge node " << k);
            EXPECT_EQ(
                this->control_grid->facet_edge_nd(0, 1, k),
                this->control_grid->facet_edge_nd(1, 1, order - k) // shared edge, opposite orientations
                );
        }

        this->save_control_nodes();
        this->save_high_order_mesh_facets();
    }

    TYPED_TEST(TwoQuadControlGridTest, random) {
        this->jiggle_control_node(this->control_grid->facet_edge_nd(0, 1, 3), +0.2);
        this->jiggle_control_node(this->control_grid->facet_nd(0, 2, 2), -0.2);
        this->jiggle_control_node(this->control_grid->facet_nd(1, 1, 4), +0.2);

        this->expect_mapping_interpolates_control_nodes(0);
        this->expect_mapping_interpolates_control_nodes(1);
        this->expect_quality_invariants_over_grid(0, SAMPLE_RESOLUTION);
        this->expect_quality_invariants_over_grid(1, SAMPLE_RESOLUTION);

        this->save_control_nodes();
        this->save_high_order_mesh_facets();
    }
}
