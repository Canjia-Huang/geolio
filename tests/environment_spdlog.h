//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/3/12.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOGRAM_MESH_UTILS_ENVIRONMENT_SPDLOG_H
#define GEOGRAM_MESH_UTILS_ENVIRONMENT_SPDLOG_H

#include <gtest/gtest.h>
#include "common/log.h"

class SpdlogTestEnvironment final : public testing::Environment {
public:
    void SetUp() override {
        spdlog::set_level(spdlog::level::trace);
    }
};

#endif //GEOGRAM_MESH_UTILS_ENVIRONMENT_SPDLOG_H