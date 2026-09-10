//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/4.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <geolio/high_order/hex_control_grid.h>
#include <gtest/gtest.h>
#include "../utils.h"

namespace geolio::test
{
    constexpr GEO::index_t TEST_POINTS_NB = 10000;

    class ProjUVWTest : public ::testing::TestWithParam<GEO::index_t> {
    public:
        void SetUp() override {
            M.vertices.create_vertices(8);
            M.vertices.point(0) = GEO::vec3(0,0,0);
            M.vertices.point(1) = GEO::vec3(1,0,0);
            M.vertices.point(2) = GEO::vec3(0,1,0);
            M.vertices.point(3) = GEO::vec3(1,1,0);
            M.vertices.point(4) = GEO::vec3(0,0,1);
            M.vertices.point(5) = GEO::vec3(1,0,1);
            M.vertices.point(6) = GEO::vec3(0,1,1);
            M.vertices.point(7) = GEO::vec3(1,1,1);
            M.cells.create_hex(0,1,2,3,4,5,6,7);

            uvws.reserve(TEST_POINTS_NB);
            for (GEO::index_t i = 0; i < TEST_POINTS_NB; i++)
                uvws.emplace_back(GEO::Numeric::random_float32(),
                                  GEO::Numeric::random_float32(),
                                  GEO::Numeric::random_float32());
        }

        GEO::Mesh M;
        const GEO::index_t c = 0;
        std::vector<GEO::vec3> uvws;
    };

    /* == hex lv =================================================================================================== */

    class ProjUVWHexVertexTest : public ProjUVWTest {};

    TEST_P(ProjUVWHexVertexTest, project_hex_lv_to_uvw) {
        const auto lv = GetParam();

        ASSERT_LT(lv, 8);
        const auto& p = M.vertices.point(M.cells.vertex(c, lv));
        const auto& uvw = project_hex_lv_to_uvw(lv);
        EXPECT_NEAR(GEO::distance2(p, uvw), 0, 1e-20);
    }

    INSTANTIATE_TEST_SUITE_P(ProjUVWTest, ProjUVWHexVertexTest, ::testing::Values(0, 1, 2, 3, 4, 5, 6, 7));

    /* == hex le =================================================================================================== */

    class ProjUVWHexEdgeTest : public ProjUVWTest {
    public:
        void SetUp() override {
            ProjUVWTest::SetUp();

            ts.reserve(TEST_POINTS_NB);
            for (GEO::index_t i = 0; i < TEST_POINTS_NB; i++)
                ts.push_back(GEO::Numeric::random_float32());
        }

        std::vector<double> ts;
    };

    TEST_P(ProjUVWHexEdgeTest, project_uvw_to_hex_le_t) {
        const auto le = GetParam();

        ASSERT_LT(le, 12);
        for (const auto& uvw : uvws) {
            const auto t = project_uvw_to_hex_le_t(uvw, le);
            EXPECT_GE(t, 0);
            EXPECT_LE(t, 1);

            const auto& ep0 = M.vertices.point(M.cells.edge_vertex(c, le, 0));
            const auto& ep1 = M.vertices.point(M.cells.edge_vertex(c, le, 1));
            const auto p = (1-t)*ep0 + t*ep1;

            EXPECT_NEAR(GEO::dot(ep1-ep0, uvw-p), 0, 1e-30); // orthogonal
        }
    }

    TEST_P(ProjUVWHexEdgeTest, project_hex_le_t_to_uvw) {
        const auto le = GetParam();

        ASSERT_LT(le, 12);
        for (const auto& t : ts) {
            const auto p = (1-t) * M.vertices.point(M.cells.edge_vertex(c, le, 0)) +
                               t * M.vertices.point(M.cells.edge_vertex(c, le, 1));

            const auto uvw = project_hex_le_t_to_uvw(t, le);
            EXPECT_GE(uvw.x, 0);    EXPECT_LE(uvw.x, 1);
            EXPECT_GE(uvw.y, 0);    EXPECT_LE(uvw.y, 1);
            EXPECT_GE(uvw.z, 0);    EXPECT_LE(uvw.z, 1);
            EXPECT_NEAR(GEO::distance2(p, uvw), 0, 1e-20);
        }
    }

    INSTANTIATE_TEST_SUITE_P(ProjUVWTest, ProjUVWHexEdgeTest, ::testing::Values(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11));

    /* == hex lf =================================================================================================== */

    class ProjUVWHexFacetTest : public ProjUVWTest {
    public:
        void SetUp() override {
            ProjUVWTest::SetUp();

            uvs.reserve(TEST_POINTS_NB);
            for (GEO::index_t i = 0; i < TEST_POINTS_NB; i++)
                uvs.emplace_back(GEO::Numeric::random_float32(),
                                  GEO::Numeric::random_float32());
        }

        std::vector<GEO::vec2> uvs;
    };

    TEST_P(ProjUVWHexFacetTest, project_uvw_to_hex_lf_uv) {
        const auto lf = GetParam();

        ASSERT_LT(lf, 6);
        for (const auto& uvw : uvws) {
            const auto uv = project_uvw_to_hex_lf_uv(uvw, lf);
            EXPECT_GE(uv.x, 0);
            EXPECT_GE(uv.y, 0);
            EXPECT_LE(uv.x, 1);
            EXPECT_LE(uv.y, 1);

            const auto& fp0 = M.vertices.point(M.cells.facet_vertex(c, lf, 0));
            const auto& fp1 = M.vertices.point(M.cells.facet_vertex(c, lf, 1));
            const auto& fp2 = M.vertices.point(M.cells.facet_vertex(c, lf, 2));
            const auto& fp3 = M.vertices.point(M.cells.facet_vertex(c, lf, 3));
            const auto p = (1-uv.x)*(1-uv.y)*fp0 + uv.x*(1-uv.y)*fp1 + (1-uv.x)*uv.y*fp3 + uv.x*uv.y*fp2;

            EXPECT_NEAR(GEO::dot(fp1-fp0, uvw-p), 0, 1e-30); // orthogonal
            EXPECT_NEAR(GEO::dot(fp2-fp0, uvw-p), 0, 1e-30); // orthogonal
            EXPECT_NEAR(GEO::dot(fp3-fp0, uvw-p), 0, 1e-30); // orthogonal
        }
    }

    TEST_P(ProjUVWHexFacetTest, project_hex_lf_uv_to_uvw) {
        const auto lf = GetParam();

        ASSERT_LT(lf, 6);
        for (const auto& uv : uvs) {
            const auto& fp0 = M.vertices.point(M.cells.facet_vertex(c, lf, 0));
            const auto& fp1 = M.vertices.point(M.cells.facet_vertex(c, lf, 1));
            const auto& fp2 = M.vertices.point(M.cells.facet_vertex(c, lf, 2));
            const auto& fp3 = M.vertices.point(M.cells.facet_vertex(c, lf, 3));
            const auto p = (1-uv.x)*(1-uv.y)*fp0 + uv.x*(1-uv.y)*fp1 + (1-uv.x)*uv.y*fp3 + uv.x*uv.y*fp2;

            const auto uvw = project_hex_lf_uv_to_uvw(uv, lf);
            EXPECT_GE(uvw.x, 0);    EXPECT_LE(uvw.x, 1);
            EXPECT_GE(uvw.y, 0);    EXPECT_LE(uvw.y, 1);
            EXPECT_GE(uvw.z, 0);    EXPECT_LE(uvw.z, 1);
            EXPECT_NEAR(GEO::distance2(p, uvw), 0, 1e-20);
        }
    }

    INSTANTIATE_TEST_SUITE_P(ProjUVWTest, ProjUVWHexFacetTest, ::testing::Values(0, 1, 2, 3, 4, 5));
}

namespace geolio::test
{
    class HexControlGridTest : public ::testing::Test {
    public:
        void save_control_nodes(const std::string& filepath) const {
            ASSERT_FALSE(control_grid == nullptr);

            GEO::Mesh mesh_out;
            GEO::Attribute<GEO::index_t> mesh_out_v_idx(mesh_out.vertices.attributes(), "idx");

            mesh_out.vertices.create_vertices(control_grid->control_nodes_nb());
            for (const auto& v : mesh_out.vertices) {
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

            EXPECT_TRUE(mesh_out.save(filepath));
        }

        void save_high_order_mesh_border(const std::string& filepath) const {
            ASSERT_FALSE(control_grid == nullptr);

            GEO::Mesh mesh_out;
            GEO::Attribute<GEO::index_t> mesh_out_v_cell(mesh_out.vertices.attributes(), "cell");
            GEO::Attribute<GEO::vec3> mesh_out_v_uvw(mesh_out.vertices.attributes(), "uvw");
            GEO::Attribute<GEO::index_t> mesh_out_f_cell(mesh_out.facets.attributes(), "cell");

            control_grid->append_discretized_high_order_cells_border(
                mesh_out,
                20,
                &mesh_out_v_cell,
                &mesh_out_v_uvw,
                &mesh_out_f_cell);

            eval_vertices_quality(mesh_out, mesh_out_v_cell, mesh_out_v_uvw);

            if (const auto& v_quantities = control_grid->control_nodes_quantities();
                v_quantities.is_bound()
                ) {
                const auto dim = v_quantities.dimension();
                GEO::Attribute<double> mesh_out_v_quantities;
                mesh_out_v_quantities.create_vector_attribute(mesh_out.vertices.attributes(), "quantities", dim);
                for (const auto& v : mesh_out.vertices) {
                    control_grid->compute_cell_uvw_quantities(
                        mesh_out_v_cell[v],
                        mesh_out_v_uvw[v],
                        &mesh_out_v_quantities[dim*v]);
                }
            }

            EXPECT_TRUE(mesh_out.save(filepath));
        }

        void save_high_order_mesh_cells(const std::string& filepath) const {
            ASSERT_FALSE(control_grid == nullptr);

            GEO::Mesh mesh_out;
            GEO::Attribute<GEO::index_t> mesh_out_v_cell(mesh_out.vertices.attributes(), "cell");
            GEO::Attribute<GEO::vec3> mesh_out_v_uvw(mesh_out.vertices.attributes(), "uvw");
            GEO::Attribute<GEO::index_t> mesh_out_c_cell(mesh_out.cells.attributes(), "cell");

            control_grid->append_discretized_high_order_cells(
                mesh_out,
                20,
                &mesh_out_v_cell,
                &mesh_out_v_uvw,
                &mesh_out_c_cell);

            eval_vertices_quality(mesh_out, mesh_out_v_cell, mesh_out_v_uvw);

            if (const auto& v_quantities = control_grid->control_nodes_quantities();
                v_quantities.is_bound()
                ) {
                const auto dim = v_quantities.dimension();
                GEO::Attribute<double> mesh_out_v_quantities;
                mesh_out_v_quantities.create_vector_attribute(mesh_out.vertices.attributes(), "quantities", dim);
                for (const auto& v : mesh_out.vertices) {
                    control_grid->compute_cell_uvw_quantities(
                        mesh_out_v_cell[v],
                        mesh_out_v_uvw[v],
                        &mesh_out_v_quantities[dim*v]);
                }
            }

            EXPECT_TRUE(mesh_out.save(filepath));
        }

    protected:
        void eval_vertices_quality(
            const GEO::Mesh& mesh_out,
            const GEO::Attribute<GEO::index_t>& mesh_out_v_cell,
            const GEO::Attribute<GEO::vec3>& mesh_out_v_uvw
            ) const {
            GEO::Attribute<double> mesh_out_v_det_jacobian(mesh_out.vertices.attributes(), "det_jacobian");
            GEO::Attribute<double> mesh_out_v_scaled_jacobian(mesh_out.vertices.attributes(), "scaled_jacobian");
            GEO::Attribute<double> mesh_out_v_inverse_mean_ratio(mesh_out.vertices.attributes(), "inverse_mean_ratio");
            GEO::Attribute<double> mesh_out_v_MIPS(mesh_out.vertices.attributes(), "MIPS");
            for (const auto& v : mesh_out.vertices) {
                const auto& c = mesh_out_v_cell[v];
                const auto& uvw = mesh_out_v_uvw[v];
                mesh_out_v_det_jacobian[v] = control_grid->compute_cell_uvw_measure(
                    c, uvw, HexControlGrid::MeasureType::DET_JACOBIAN);
                mesh_out_v_scaled_jacobian[v] = control_grid->compute_cell_uvw_measure(
                    c, uvw, HexControlGrid::MeasureType::SCALED_JACOBIAN);
                mesh_out_v_inverse_mean_ratio[v] = control_grid->compute_cell_uvw_measure(
                    c, uvw, HexControlGrid::MeasureType::INVERSE_MEAN_RATIO);
                mesh_out_v_MIPS[v] = control_grid->compute_cell_uvw_measure(
                    c, uvw, HexControlGrid::MeasureType::MIPS);
            }
        }

        GEO::Mesh mesh;
        std::unique_ptr<HexControlGrid> control_grid;
    };

    class SingleHexControlGridTest : public HexControlGridTest {
    protected:
        void SetUp() override {
            mesh.vertices.create_vertices(8);
            mesh.vertices.point(0) = GEO::vec3(0, 0, 0);
            mesh.vertices.point(1) = GEO::vec3(1, 0, 0);
            mesh.vertices.point(2) = GEO::vec3(0, 1, 0);
            mesh.vertices.point(3) = GEO::vec3(1, 1, 0);
            mesh.vertices.point(4) = GEO::vec3(0, 0, 1);
            mesh.vertices.point(5) = GEO::vec3(1, 0, 1);
            mesh.vertices.point(6) = GEO::vec3(0, 1, 1);
            mesh.vertices.point(7) = GEO::vec3(1, 1, 1);
            mesh.cells.create_hex(0, 1, 2, 3, 4, 5, 6, 7);

            constexpr GEO::index_t order = 5;
            control_grid = std::make_unique<HexControlGrid>(mesh, order);
        }
    };

    TEST_F(SingleHexControlGridTest, regular) {
        save_control_nodes(get_current_test_name()+"_nodes.geogram");
        save_high_order_mesh_border(get_current_test_name()+"_border.geogram");
        save_high_order_mesh_cells(get_current_test_name()+"_cells.geogram");
    }

    TEST_F(SingleHexControlGridTest, random) {
        control_grid->control_node(control_grid->cell_edge_nd(0, 1, 2)) += 0.1 *
            GEO::vec3(GEO::Numeric::random_float32(), GEO::Numeric::random_float32(), GEO::Numeric::random_float32());
        control_grid->control_node(control_grid->cell_facet_nd(0, 2, 2, 3)) += 0.1 *
            GEO::vec3(GEO::Numeric::random_float32(), GEO::Numeric::random_float32(), GEO::Numeric::random_float32());
        control_grid->control_node(control_grid->cell_nd(0, 1, 2, 3)) += 0.1 *
            GEO::vec3(GEO::Numeric::random_float32(), GEO::Numeric::random_float32(), GEO::Numeric::random_float32());
        save_control_nodes(get_current_test_name()+"_nodes.geogram");
        save_high_order_mesh_border(get_current_test_name()+"_border.geogram");
        save_high_order_mesh_cells(get_current_test_name()+"_cells.geogram");
    }

    TEST_F(SingleHexControlGridTest, quantities) {
        control_grid->control_node(control_grid->cell_edge_nd(0, 1, 2)) += 0.1 *
            GEO::vec3(GEO::Numeric::random_float32(), GEO::Numeric::random_float32(), GEO::Numeric::random_float32());
        control_grid->control_node(control_grid->cell_facet_nd(0, 2, 2, 3)) += 0.1 *
            GEO::vec3(GEO::Numeric::random_float32(), GEO::Numeric::random_float32(), GEO::Numeric::random_float32());
        control_grid->control_node(control_grid->cell_nd(0, 1, 2, 3)) += 0.1 *
            GEO::vec3(GEO::Numeric::random_float32(), GEO::Numeric::random_float32(), GEO::Numeric::random_float32());

        constexpr GEO::index_t dim = 6;
        this->control_grid->create_control_node_quantities(dim);
        auto& v_quantities = this->control_grid->control_nodes_quantities();
        ASSERT_TRUE(v_quantities.is_bound());
        for (GEO::index_t v = 0, v_end = this->control_grid->control_nodes_nb(); v < v_end; ++v) {
            for (GEO::index_t d = 0; d < dim; ++d)
                v_quantities[dim*v+d] = (d+1)*GEO::Numeric::random_float32();
        }

        save_control_nodes(get_current_test_name()+"_nodes.geogram");
        save_high_order_mesh_border(get_current_test_name()+"_border.geogram");
        save_high_order_mesh_cells(get_current_test_name()+"_cells.geogram");
    }

    TEST_F(SingleHexControlGridTest, facet_normal) {
        constexpr GEO::index_t c = 0;
        constexpr GEO::index_t lf = 1;

        control_grid->control_node(control_grid->cell_facet_nd(c, lf, 2, 3)) += 0.1 *
            GEO::vec3(GEO::Numeric::random_float32(), GEO::Numeric::random_float32(), GEO::Numeric::random_float32());
        control_grid->control_node(control_grid->cell_facet_nd(c, lf, 3, 1)) += 0.1 *
            GEO::vec3(GEO::Numeric::random_float32(), GEO::Numeric::random_float32(), GEO::Numeric::random_float32());

        constexpr GEO::index_t points_nb = 100;
        GEO::Mesh mesh_out;
        GEO::index_t new_v = mesh_out.vertices.create_vertices(2*points_nb);
        GEO::index_t new_e = mesh_out.edges.create_edges(points_nb);
        for (GEO::index_t i = 0; i < points_nb; ++i) {
            const GEO::vec2 uv(GEO::Numeric::random_float32(), GEO::Numeric::random_float32());
            const auto position = control_grid->compute_cell_uvw_position(c, project_hex_lf_uv_to_uvw(uv, lf));
            const auto normal = control_grid->compute_cell_facet_uv_normal(c, lf, uv);

            mesh_out.vertices.point(new_v) = position;
            mesh_out.vertices.point(new_v+1) = position + 0.2*normal;
            mesh_out.edges.set_vertex(new_e, 0, new_v);
            mesh_out.edges.set_vertex(new_e, 1, new_v+1);
            new_v += 2;
            ++new_e;
        }

        control_grid->append_discretized_high_order_cells_border(mesh_out, 30);
        mesh_out.save(get_current_test_name()+".geogram");
    }

    TEST_F(SingleHexControlGridTest, dudvdw) {
        constexpr GEO::index_t c = 0;
        control_grid->control_node(control_grid->cell_nd(c, 1, 2, 3)) += 0.1 *
            GEO::vec3(GEO::Numeric::random_float32(), GEO::Numeric::random_float32(), GEO::Numeric::random_float32());
        control_grid->control_node(control_grid->cell_nd(c, 3, 0, 2)) += 0.1 *
            GEO::vec3(GEO::Numeric::random_float32(), GEO::Numeric::random_float32(), GEO::Numeric::random_float32());

        constexpr GEO::index_t resolution = 10;
        GEO::Mesh mesh_out;
        GEO::Attribute<GEO::index_t> mesh_out_e_axis(mesh_out.edges.attributes(), "axis");
        GEO::index_t new_v = mesh_out.vertices.create_vertices(4*std::pow(resolution+1, 3));
        GEO::index_t new_e = mesh_out.edges.create_edges(3*std::pow(resolution+1, 3));
        for (GEO::index_t i = 0; i < resolution+1; ++i) {
            const double ri = static_cast<double>(i)/resolution;
            for (GEO::index_t j = 0; j < resolution+1; ++j) {
                const double rj = static_cast<double>(j)/resolution;
                for (GEO::index_t k = 0; k < resolution+1; ++k) {
                    const double rk = static_cast<double>(k)/resolution;

                    const GEO::vec3 uvw(ri, rj, rk);
                    GEO::vec3 du, dv, dw;
                    std::vector<double> Bu, Bv, Bw, dBu, dBv, dBw;
                    control_grid->compute_cell_uvw_dudvdw(c, uvw, du, dv, dw, Bu, Bv, Bw, dBu, dBv, dBw);

                    constexpr double length = 0.05;
                    mesh_out.vertices.point(new_v) = uvw;
                    mesh_out.vertices.point(new_v+1) = uvw + length*du;
                    mesh_out.vertices.point(new_v+2) = uvw + length*dv;
                    mesh_out.vertices.point(new_v+3) = uvw + length*dw;
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
            }
        }

        control_grid->append_discretized_high_order_cells(mesh_out, 30);
        mesh_out.save(get_current_test_name()+".geogram");
    }

    class TwoHexControlGridTest : public HexControlGridTest {
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

            constexpr GEO::index_t order = 6;
            control_grid = std::make_unique<HexControlGrid>(mesh, order);
        }
    };

    TEST_F(TwoHexControlGridTest, regular) {
        save_control_nodes(get_current_test_name()+"_nodes.geogram");
        save_high_order_mesh_border(get_current_test_name()+"_border.geogram");
        save_high_order_mesh_cells(get_current_test_name()+"_cells.geogram");
    }

    TEST_F(TwoHexControlGridTest, random) {
        control_grid->control_node(control_grid->cell_edge_nd(0, 2, 3)) += 0.1 *
            GEO::vec3(GEO::Numeric::random_float32(), GEO::Numeric::random_float32(), GEO::Numeric::random_float32());
        control_grid->control_node(control_grid->cell_facet_nd(1, 0, 2, 4)) += 0.1 *
            GEO::vec3(GEO::Numeric::random_float32(), GEO::Numeric::random_float32(), GEO::Numeric::random_float32());
        control_grid->control_node(control_grid->cell_nd(1, 2, 4, 1)) += 0.1 *
            GEO::vec3(GEO::Numeric::random_float32(), GEO::Numeric::random_float32(), GEO::Numeric::random_float32());
        save_control_nodes(get_current_test_name()+"_nodes.geogram");
        save_high_order_mesh_border(get_current_test_name()+"_border.geogram");
        save_high_order_mesh_cells(get_current_test_name()+"_cells.geogram");
    }
}
