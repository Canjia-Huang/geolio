//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/10.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <gtest/gtest.h>
#include <geolio/io/high_order_hex_mesh_io.h>
#include "../utils.h"

namespace geolio::test
{
    class HighOrderHexMeshIO : public ::testing::Test {
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

    class SingleHexCHighOrderHexMeshIO : public HighOrderHexMeshIO {
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

    TEST_F(SingleHexCHighOrderHexMeshIO, version_2_2) {
        control_grid->control_node(control_grid->cell_edge_nd(0, 1, 2)) += 0.1 *
            GEO::vec3(GEO::Numeric::random_float32(), GEO::Numeric::random_float32(), GEO::Numeric::random_float32());
        control_grid->control_node(control_grid->cell_facet_nd(0, 2, 2, 3)) += 0.1 *
            GEO::vec3(GEO::Numeric::random_float32(), GEO::Numeric::random_float32(), GEO::Numeric::random_float32());
        control_grid->control_node(control_grid->cell_nd(0, 1, 2, 3)) += 0.1 *
            GEO::vec3(GEO::Numeric::random_float32(), GEO::Numeric::random_float32(), GEO::Numeric::random_float32());
        EXPECT_TRUE(high_order_hex_mesh_save(*control_grid, get_current_test_name()+".msh", "2.2"));
    }

    TEST_F(SingleHexCHighOrderHexMeshIO, version_4_1) {
        control_grid->control_node(control_grid->cell_edge_nd(0, 1, 2)) += 0.1 *
            GEO::vec3(GEO::Numeric::random_float32(), GEO::Numeric::random_float32(), GEO::Numeric::random_float32());
        control_grid->control_node(control_grid->cell_facet_nd(0, 2, 2, 3)) += 0.1 *
            GEO::vec3(GEO::Numeric::random_float32(), GEO::Numeric::random_float32(), GEO::Numeric::random_float32());
        control_grid->control_node(control_grid->cell_nd(0, 1, 2, 3)) += 0.1 *
            GEO::vec3(GEO::Numeric::random_float32(), GEO::Numeric::random_float32(), GEO::Numeric::random_float32());
        EXPECT_TRUE(high_order_hex_mesh_save(*control_grid, get_current_test_name()+".msh", "4.1"));
    }

    class TwoHexCHighOrderHexMeshIO : public HighOrderHexMeshIO {
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

            constexpr GEO::index_t order = 6;
            control_grid = std::make_unique<HexControlGrid>(mesh, order);
        }
    };

    TEST_F(TwoHexCHighOrderHexMeshIO, version_2_2) {
        control_grid->control_node(control_grid->cell_edge_nd(0, 2, 3)) += 0.1 *
            GEO::vec3(GEO::Numeric::random_float32(), GEO::Numeric::random_float32(), GEO::Numeric::random_float32());
        control_grid->control_node(control_grid->cell_facet_nd(1, 0, 2, 4)) += 0.1 *
            GEO::vec3(GEO::Numeric::random_float32(), GEO::Numeric::random_float32(), GEO::Numeric::random_float32());
        control_grid->control_node(control_grid->cell_nd(1, 2, 4, 1)) += 0.1 *
            GEO::vec3(GEO::Numeric::random_float32(), GEO::Numeric::random_float32(), GEO::Numeric::random_float32());
        EXPECT_TRUE(high_order_hex_mesh_save(*control_grid, get_current_test_name()+".msh", "2.2"));
    }

    TEST_F(TwoHexCHighOrderHexMeshIO, version_4_1) {
        control_grid->control_node(control_grid->cell_edge_nd(0, 2, 3)) += 0.1 *
            GEO::vec3(GEO::Numeric::random_float32(), GEO::Numeric::random_float32(), GEO::Numeric::random_float32());
        control_grid->control_node(control_grid->cell_facet_nd(1, 0, 2, 4)) += 0.1 *
            GEO::vec3(GEO::Numeric::random_float32(), GEO::Numeric::random_float32(), GEO::Numeric::random_float32());
        control_grid->control_node(control_grid->cell_nd(1, 2, 4, 1)) += 0.1 *
            GEO::vec3(GEO::Numeric::random_float32(), GEO::Numeric::random_float32(), GEO::Numeric::random_float32());
        EXPECT_TRUE(high_order_hex_mesh_save(*control_grid, get_current_test_name()+".msh", "4.1"));
    }
}