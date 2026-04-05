//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/3/12.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//

#include <filesystem>
#include <geogram/mesh/mesh_io.h>
#include <gtest/gtest.h>
#include "utils.h"
#include "common/log.h"

namespace GEO::MeshUtils::Test
{
    TEST(DepsSpdlogTest, log) {
        LOG::TRACE("Hello World!");
        LOG::DEBUG("Hello World!");
        LOG::INFO("Hello World!");
        LOG::WARN("Hello World!");
        LOG::ERROR("Hello World!");
        LOG::CRITICAL("Hello World!");

        SUCCEED();
    }

    TEST(DepsGeogramTest, io) {
        GEO::Mesh M;
        M.vertices.create_vertices(4);
        M.vertices.point(0) = GEO::vec3(0,0,0);
        M.vertices.point(1) = GEO::vec3(1,0,0);
        M.vertices.point(2) = GEO::vec3(0,1,0);
        M.vertices.point(3) = GEO::vec3(0,0,1);
        M.cells.create_tet(0,1,2,3);

        for (GEO::index_t lv = 0; lv < 4; ++lv)
            LOG::INFO("tet lv {} - v {}", lv, M.cells.vertex(0, lv));
        for (GEO::index_t le = 0; le < 6; ++le)
            LOG::INFO("tet le {} - v {}", le, M.cells.edge_vertex(0, le, 0), M.cells.edge_vertex(0, le, 1));
        for (GEO::index_t lf = 0; lf < 4; ++lf)
            LOG::INFO("tet lf {} - v {},{},{}", lf, M.cells.facet_vertex(0, lf, 0), M.cells.facet_vertex(0, lf, 1), M.cells.facet_vertex(0, lf, 2));

        const std::string write_file_path = get_current_test_name() + ".mesh";
        EXPECT_TRUE(GEO::mesh_save(M, write_file_path));
        EXPECT_TRUE(std::filesystem::exists(write_file_path));

        GEO::Mesh M_in;
        EXPECT_TRUE(GEO::mesh_load(write_file_path, M_in));
        EXPECT_EQ(M_in.vertices.nb(), 4);
        EXPECT_EQ(M_in.cells.nb(), 1);
    }
}