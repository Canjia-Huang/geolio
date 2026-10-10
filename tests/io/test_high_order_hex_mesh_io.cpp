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
        static constexpr GEO::index_t MESH_DIM = 3;
        static constexpr GEO::index_t QUANTITIES_DIM = 2;
        static constexpr GEO::index_t DIM = MESH_DIM + QUANTITIES_DIM;

        void add_node_random(const GEO::index_t nd) {
            auto& p = control_grid->control_node(nd);
            for (GEO::index_t d = 0; d < DIM; ++d)
                p[d] += 0.1*GEO::Numeric::random_float32();
        }

        GEO::Mesh mesh;
        std::unique_ptr<HexControlGrid<MESH_DIM, QUANTITIES_DIM>> control_grid;
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
            control_grid = std::make_unique<HexControlGrid<MESH_DIM, QUANTITIES_DIM>>(mesh, order);
        }
    };

    TEST_F(SingleHexCHighOrderHexMeshIO, version_2_2) {
        add_node_random(control_grid->cell_edge_nd(0, 1, 2));
        add_node_random(control_grid->cell_facet_nd(0, 2, 2, 3));
        add_node_random(control_grid->cell_nd(0, 1, 2, 3));

        /* Save */
        const std::string filepath = get_current_test_name()+".msh";
        EXPECT_TRUE(high_order_hex_mesh_save(*control_grid, filepath, "2.2"));

        /* Load */
        std::unique_ptr<HexControlGrid<MESH_DIM, QUANTITIES_DIM>> loaded_control_grid_ptr;
        ASSERT_TRUE(high_order_hex_mesh_load(filepath, loaded_control_grid_ptr));
        EXPECT_TRUE(loaded_control_grid_ptr->mesh().save(get_current_test_name()+".geogram"));
    }

    TEST_F(SingleHexCHighOrderHexMeshIO, version_4_1) {
        add_node_random(control_grid->cell_edge_nd(0, 1, 2));
        add_node_random(control_grid->cell_facet_nd(0, 2, 2, 3));
        add_node_random(control_grid->cell_nd(0, 1, 2, 3));

        /* Save */
        const std::string filepath = get_current_test_name()+".msh";
        EXPECT_TRUE(high_order_hex_mesh_save(*control_grid, filepath, "4.1"));

        /* Load */
        GEO::Mesh loaded_mesh;
        std::unique_ptr<HexControlGrid<MESH_DIM, QUANTITIES_DIM>> loaded_control_grid_ptr;
        ASSERT_TRUE(high_order_hex_mesh_load(filepath, loaded_control_grid_ptr));
        EXPECT_TRUE(loaded_control_grid_ptr->mesh().save(get_current_test_name()+".geogram"));
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
            control_grid = std::make_unique<HexControlGrid<MESH_DIM, QUANTITIES_DIM>>(mesh, order);
        }
    };

    TEST_F(TwoHexCHighOrderHexMeshIO, version_2_2) {
        add_node_random(control_grid->cell_edge_nd(0, 2, 3));
        add_node_random(control_grid->cell_facet_nd(1, 0, 2, 4));
        add_node_random(control_grid->cell_nd(1, 2, 4, 1));

        /* Save */
        const std::string filepath = get_current_test_name()+".msh";
        EXPECT_TRUE(high_order_hex_mesh_save(*control_grid, filepath, "2.2"));

        /* Load */
        GEO::Mesh loaded_mesh;
        std::unique_ptr<HexControlGrid<MESH_DIM, QUANTITIES_DIM>> loaded_control_grid_ptr;
        ASSERT_TRUE(high_order_hex_mesh_load(filepath, loaded_control_grid_ptr));
        EXPECT_TRUE(loaded_control_grid_ptr->mesh().save(get_current_test_name()+".geogram"));
    }

    TEST_F(TwoHexCHighOrderHexMeshIO, version_4_1) {
        add_node_random(control_grid->cell_edge_nd(0, 2, 3));
        add_node_random(control_grid->cell_facet_nd(1, 0, 2, 4));
        add_node_random(control_grid->cell_nd(1, 2, 4, 1));

        /* Save */
        const std::string filepath = get_current_test_name()+".msh";
        EXPECT_TRUE(high_order_hex_mesh_save(*control_grid, filepath, "4.1"));

        /* Load */
        GEO::Mesh loaded_mesh;
        std::unique_ptr<HexControlGrid<MESH_DIM, QUANTITIES_DIM>> loaded_control_grid_ptr;
        ASSERT_TRUE(high_order_hex_mesh_load(filepath, loaded_control_grid_ptr));
        EXPECT_TRUE(loaded_control_grid_ptr->mesh().save(get_current_test_name()+".geogram"));
    }
}