//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/13.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "test_log_class_example.h"

namespace geolio::test
{
    LOG::Category &LoggingExample::Logger() {
        static LOG::Category category{"test.logging_example"};
        return category;
    }

    void LoggingExample::Run() {
        LOG::TRACE(Logger(), "trace message");
        LOG::DEBUG(Logger(), "debug message {}", 2);
        LOG::INFO(Logger(), "info message {}", 3);
        LOG::WARN(Logger(), "warn message");
        LOG::ERROR(Logger(), "error message");
        LOG::CRITICAL(Logger(), "critical message");

        // No category: this one belongs to the application as a whole.
        LOG::INFO("global message of the same class");
    }
} // namespace geolio::test
