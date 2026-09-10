//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/10.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <gtest/gtest.h>
#include <geolio/io/high_order_hex_mesh_io.h>
#include "../utils.h"
#include <geogram/points/kd_tree.h>
#include <filesystem>

namespace geolio::test
{
    class HighOrderHexMeshIO : public ::testing::Test {
    protected:
        void same_as(const GEO::Mesh& other_mesh, const std::unique_ptr<HexControlGrid>& other_control_grid) {
            ASSERT_FALSE(control_grid == nullptr);
            ASSERT_FALSE(other_control_grid == nullptr);

            EXPECT_EQ(other_mesh.vertices.nb(), mesh.vertices.nb());
            EXPECT_EQ(other_mesh.facets.nb(), mesh.facets.nb());
            EXPECT_EQ(other_control_grid->control_nodes_nb(), control_grid->control_nodes_nb());

            /* Match nodes */
            std::vector<double> control_node_points;
            control_node_points.reserve(3*control_grid->control_nodes_nb());
            for (GEO::index_t nd = 0, nd_end = control_grid->control_nodes_nb(); nd < nd_end; ++nd) {
                const auto& p = control_grid->control_node(nd);
                for (GEO::index_t d = 0; d < 3; ++d)
                    control_node_points.push_back(p[d]);
            }

            GEO::SmartPointer<GEO::BalancedKdTree> kd_tree = new GEO::BalancedKdTree(3);
            kd_tree->set_points(control_grid->control_nodes_nb(), control_node_points.data());

            std::vector<GEO::index_t> found_control_nodes(control_grid->control_nodes_nb(), 0);
            for (GEO::index_t nd = 0, nd_end = other_control_grid->control_nodes_nb(); nd < nd_end; ++nd) {
                const auto& p = other_control_grid->control_node(nd);
                const auto nearest_nd = kd_tree->get_nearest_neighbor(p.data());
                found_control_nodes[nearest_nd] = 1;
                EXPECT_NEAR(GEO::distance2(control_grid->control_node(nearest_nd), p), 0, 1e-10);
            }
            EXPECT_TRUE(std::ranges::all_of(found_control_nodes, [](const auto b){ return b; }));
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

        /* Save */
        const std::filesystem::path filepath = get_current_test_name()+".msh";
        EXPECT_TRUE(high_order_hex_mesh_save(*control_grid, filepath, "2.2"));

        /* Load */
        GEO::Mesh loaded_mesh;
        std::unique_ptr<HexControlGrid> loaded_control_grid_ptr;
        ASSERT_TRUE(high_order_hex_mesh_load(filepath, loaded_mesh, loaded_control_grid_ptr));
        this->same_as(loaded_mesh, loaded_control_grid_ptr);
    }

    TEST_F(SingleHexCHighOrderHexMeshIO, version_4_1) {
        control_grid->control_node(control_grid->cell_edge_nd(0, 1, 2)) += 0.1 *
            GEO::vec3(GEO::Numeric::random_float32(), GEO::Numeric::random_float32(), GEO::Numeric::random_float32());
        control_grid->control_node(control_grid->cell_facet_nd(0, 2, 2, 3)) += 0.1 *
            GEO::vec3(GEO::Numeric::random_float32(), GEO::Numeric::random_float32(), GEO::Numeric::random_float32());
        control_grid->control_node(control_grid->cell_nd(0, 1, 2, 3)) += 0.1 *
            GEO::vec3(GEO::Numeric::random_float32(), GEO::Numeric::random_float32(), GEO::Numeric::random_float32());

        /* Save */
        const std::filesystem::path filepath = get_current_test_name()+".msh";
        EXPECT_TRUE(high_order_hex_mesh_save(*control_grid, filepath, "4.1"));

        /* Load */
        GEO::Mesh loaded_mesh;
        std::unique_ptr<HexControlGrid> loaded_control_grid_ptr;
        ASSERT_TRUE(high_order_hex_mesh_load(filepath, loaded_mesh, loaded_control_grid_ptr));
        this->same_as(loaded_mesh, loaded_control_grid_ptr);
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
            mesh.cells.connect();

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

        /* Save */
        const std::filesystem::path filepath = get_current_test_name()+".msh";
        EXPECT_TRUE(high_order_hex_mesh_save(*control_grid, filepath, "2.2"));

        /* Load */
        GEO::Mesh loaded_mesh;
        std::unique_ptr<HexControlGrid> loaded_control_grid_ptr;
        ASSERT_TRUE(high_order_hex_mesh_load(filepath, loaded_mesh, loaded_control_grid_ptr));
        this->same_as(loaded_mesh, loaded_control_grid_ptr);
    }

    TEST_F(TwoHexCHighOrderHexMeshIO, version_4_1) {
        control_grid->control_node(control_grid->cell_edge_nd(0, 2, 3)) += 0.1 *
            GEO::vec3(GEO::Numeric::random_float32(), GEO::Numeric::random_float32(), GEO::Numeric::random_float32());
        control_grid->control_node(control_grid->cell_facet_nd(1, 0, 2, 4)) += 0.1 *
            GEO::vec3(GEO::Numeric::random_float32(), GEO::Numeric::random_float32(), GEO::Numeric::random_float32());
        control_grid->control_node(control_grid->cell_nd(1, 2, 4, 1)) += 0.1 *
            GEO::vec3(GEO::Numeric::random_float32(), GEO::Numeric::random_float32(), GEO::Numeric::random_float32());

        /* Save */
        const std::filesystem::path filepath = get_current_test_name()+".msh";
        EXPECT_TRUE(high_order_hex_mesh_save(*control_grid, filepath, "4.1"));

        /* Load */
        GEO::Mesh loaded_mesh;
        std::unique_ptr<HexControlGrid> loaded_control_grid_ptr;
        ASSERT_TRUE(high_order_hex_mesh_load(filepath, loaded_mesh, loaded_control_grid_ptr));
        this->same_as(loaded_mesh, loaded_control_grid_ptr);
    }
}