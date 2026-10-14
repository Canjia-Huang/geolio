//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/13.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_TEST_LOG_CLASS_EXAMPLE_H
#define GEOLIO_TEST_LOG_CLASS_EXAMPLE_H

#include <geolio/common/log.h>

namespace geolio::test
{
    /**
     * @brief The "one category per class" example of <geolio/common/log.h>.
     * @details The class owns a named category and names it in every log
     *          statement, so that everything it logs can be silenced, retuned or
     *          redirected on its own, from any other class or application:
     * @code
     *   LOG::Disable(LoggingExample::Logger().Name());
     *   LOG::SetLevel(LoggingExample::Logger().Name(), spdlog::level::debug);
     * @endcode
     *          Its implementation lives in test_log_class_example.cpp, a separate
     *          translation unit from the tests: a category is process-wide, so
     *          disabling it in the test must also silence the messages that are
     *          emitted from here.
     */
    class LoggingExample {
    public:
        /**
         * @brief Returns the category of this class.
         * @details A function-local static, as recommended by log.h: the logger
         *          behind the category is created on first use, hence it inherits
         *          the sinks, the pattern and the level the application has at that
         *          moment instead of the ones of an earlier static initialization.
         * @return The category of this class.
         */
        static LOG::Category &Logger();

        /**
         * @brief Emits one message per level on the category of this class.
         * @details Ends with a global LOG::INFO, i.e. one that names no category:
         *          it goes to the default logger, and silencing this class must
         *          leave it alone.
         */
        static void Run() ;
    };
} // namespace geolio::test

#endif //GEOLIO_TEST_LOG_CLASS_EXAMPLE_H
