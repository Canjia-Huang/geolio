//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/3/12.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//

#include <filesystem>
#include <geogram/mesh/mesh_io.h>
#include <gtest/gtest.h>
#include "utils.h"

namespace GEO::MeshUtils::Test
{
    TEST(DepsGeogramTest, tet_io) {
        GEO::Mesh M;
        M.vertices.create_vertices(4);
        M.vertices.point(0) = GEO::vec3(0,0,0);
        M.vertices.point(1) = GEO::vec3(1,0,0);
        M.vertices.point(2) = GEO::vec3(0,1,0);
        M.vertices.point(3) = GEO::vec3(0,0,1);
        M.cells.create_tet(0,1,2,3);

        for (GEO::index_t lv = 0; lv < 4; ++lv)
            std::cout << "tet lv " << lv << " - v " << M.cells.vertex(0, lv) << std::endl;
        for (GEO::index_t le = 0; le < 6; ++le)
            std::cout << "tet le " << le << " - v " << M.cells.edge_vertex(0, le, 0) << "," << M.cells.edge_vertex(0, le, 1);
        for (GEO::index_t le = 0; le < 6; ++le)
            std::cout << "tet le " << le << " - adj_lf " << M.cells.edge_adjacent_facet(0, le, 0) << "," << M.cells.edge_adjacent_facet(0, le, 1);
        for (GEO::index_t lf = 0; lf < 4; ++lf)
            std::cout << "tet lf " << lf << " - v " << M.cells.facet_vertex(0, lf, 0) << "," << M.cells.facet_vertex(0, lf, 1) << "," << M.cells.facet_vertex(0, lf, 2);

        const std::string write_file_path = get_current_test_name() + ".mesh";
        EXPECT_TRUE(GEO::mesh_save(M, write_file_path));
        EXPECT_TRUE(std::filesystem::exists(write_file_path));

        GEO::Mesh M_in;
        EXPECT_TRUE(GEO::mesh_load(write_file_path, M_in));
        EXPECT_EQ(M_in.vertices.nb(), 4);
        EXPECT_EQ(M_in.cells.nb(), 1);
    }

    TEST(DepsGeogramTest, hex_io) {
        GEO::Mesh M;
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

        for (GEO::index_t lv = 0; lv < 8; ++lv)
            std::cout << "hex lv " << lv << " - v " << M.cells.vertex(0, lv) << std::endl;
        for (GEO::index_t le = 0; le < 12; ++le)
            std::cout << "hex le " << le << " - v " << M.cells.edge_vertex(0, le, 0) << "," << M.cells.edge_vertex(0, le, 1) << std::endl;
        for (GEO::index_t le = 0; le < 12; ++le)
            std::cout << "hex le " << le << " - adj_lf " << M.cells.edge_adjacent_facet(0, le, 0) << "," << M.cells.edge_adjacent_facet(0, le, 1) << std::endl;
        for (GEO::index_t lf = 0; lf < 6; ++lf)
            std::cout << "hex lf " << lf << " - v " << M.cells.facet_vertex(0, lf, 0) << "," << M.cells.facet_vertex(0, lf, 1) << "," << M.cells.facet_vertex(0, lf, 2) << "," << M.cells.facet_vertex(0, lf, 3) << std::endl;

        const std::string write_file_path = get_current_test_name() + ".mesh";
        EXPECT_TRUE(GEO::mesh_save(M, write_file_path));
        EXPECT_TRUE(std::filesystem::exists(write_file_path));

        GEO::Mesh M_in;
        EXPECT_TRUE(GEO::mesh_load(write_file_path, M_in));
        EXPECT_EQ(M_in.vertices.nb(), 8);
        EXPECT_EQ(M_in.cells.nb(), 1);
    }
}