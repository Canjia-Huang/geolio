//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/3/12.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOGRAM_MESH_UTILS_ENVIRONMENT_GEOGRAM_H
#define GEOGRAM_MESH_UTILS_ENVIRONMENT_GEOGRAM_H

#include <geogram/basic/command_line_args.h>
#include <geogram/basic/common.h>
#include <geogram/basic/attributes.h>
#include <gtest/gtest.h>

class GeogramTestEnvironment final : public testing::Environment {
public:
    void SetUp() override {
        GEO::initialize(GEO::GEOGRAM_INSTALL_ALL);
        GEO::CmdLine::import_arg_group("standard");
        GEO::CmdLine::import_arg_group("algo");
    }
};

#endif //GEOGRAM_MESH_UTILS_ENVIRONMENT_GEOGRAM_H