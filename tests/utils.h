//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/3/12.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef ROBUSTLOCALREMESHING_UTILS_H
#define ROBUSTLOCALREMESHING_UTILS_H

#include <gtest/gtest.h>
#include <string>

namespace geolio::test
{
    inline std::string get_current_test_name(){
        const testing::TestInfo* const current_test_info = testing::UnitTest::GetInstance()->current_test_info();
        return std::string("test")
                + "_"
                + std::string(current_test_info->test_case_name())
                + "_"
                + std::string(current_test_info->name());
    }
}


#endif //ROBUSTLOCALREMESHING_UTILS_H