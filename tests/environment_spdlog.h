//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/3/12.
// Copyright (c) 2026 Graphics@XMU. All rights reserved.
//
#ifndef MESHOPT_ENVIRONMENT_SPDLOG_H
#define MESHOPT_ENVIRONMENT_SPDLOG_H

#include <gtest/gtest.h>
#include "common/log.h"

class SpdlogTestEnvironment final : public testing::Environment {
public:
    void SetUp() override {
        spdlog::set_level(spdlog::level::trace);
    }
};

#endif //MESHOPT_ENVIRONMENT_SPDLOG_H