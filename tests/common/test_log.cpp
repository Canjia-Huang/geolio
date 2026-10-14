//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/13.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
// Tests of the "one category per class" logging of <geolio/common/log.h>, in the
// order the feature is meant to be used:
//
//   1. a class names its category in its log statements, and those messages end
//      up on that category and nowhere else;
//   2. the category is then silenced, retuned or listed by name, from any other
//      class or translation unit, while the global statements of the same class
//      keep going to the default logger;
//   3. a category is a plain spdlog logger that inherits the configuration of the
//      application, so the two ways of logging stay consistent;
//   4. its name is printed like any other logger name, through `%n`, which the
//      default pattern of spdlog already contains.
//
#include <gtest/gtest.h>
#include <spdlog/sinks/ostream_sink.h>
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <geolio/common/log.h>
#include "test_log_class_example.h"

namespace geolio::test
{
    namespace
    {
        /**
         * @brief The stream that every sink installed by these tests writes into.
         * @details It is a function-local static on purpose: it must outlive every
         *          logger and sink that a test installs, because a category keeps
         *          the sinks it is given for the rest of the process, and a sink
         *          outliving the stream it writes into would dangle as soon as any
         *          later test logs again.
         * @return The capture stream.
         */
        std::ostringstream &CaptureStream() {
            static std::ostringstream stream;
            return stream;
        }

        /** @brief Empties the capture stream. */
        void ClearCapture() {
            CaptureStream().str({});
            CaptureStream().clear();
        }

        /**
         * @brief Returns everything the capture stream has received so far.
         * @return The captured text.
         */
        std::string CapturedText() {
            return CaptureStream().str();
        }

        /**
         * @brief Tells whether the capture stream contains the given text.
         * @param[in] text The text to look for.
         * @return True when the text was captured.
         */
        bool Contains(const std::string &text) {
            return CapturedText().find(text) != std::string::npos;
        }

        /**
         * @brief Opens the category @p name, writing to the capture stream.
         * @details The sinks of the category are replaced, which detaches it from
         *          the console and file sinks of the test environment: a test must
         *          observe exactly the messages it emits. The pattern keeps the
         *          logger name, the level and the message, and drops the timestamp
         *          so that the expectations are readable; an empty pattern keeps the
         *          default one, i.e. the one a category has when the application
         *          sets no pattern at all.
         * @param[in] name The category name, used as the spdlog logger name.
         * @param[in] pattern The pattern of the category, empty for the default one.
         * @return A handle on that category.
         */
        LOG::Category CapturedCategory(const std::string &name, const std::string &pattern = "%n|%l|%v") {
            LOG::Category category{name};
            category.Logger()->sinks() = {std::make_shared<spdlog::sinks::ostream_sink_mt>(CaptureStream())};
            if (!pattern.empty()) {
                category.Logger()->set_pattern(pattern);
            }
            return category;
        }

        /**
         * @brief Prepares the category of the example class for one test.
         * @details Redirects it to the capture stream and puts it back into a known
         *          state at the level given, since a category is process-wide and
         *          the tests that ran before may have silenced or retuned it.
         * @param[in] level The level to leave the category with.
         * @return A handle on the category of the example class.
         */
        LOG::Category CaptureExample(const spdlog::level::level_enum level = spdlog::level::trace) {
            ClearCapture();
            const LOG::Category category = CapturedCategory(LoggingExample::Logger().Name());
            category.SetEnabled(true);
            category.SetLevel(level);
            return category;
        }

        /**
         * @brief Installs a default logger that writes to the capture stream.
         * @details The global tags, i.e. the LOG::TRACE/DEBUG/... statements that
         *          name no category, go to the default logger; pointing that logger
         *          at the capture stream is how these tests observe them. The
         *          previous default logger, hence the console and file sinks of the
         *          test environment, is put back when the guard goes out of scope.
         *          An empty pattern keeps the default one, and the name defaults to
         *          a non-empty one because the default logger of spdlog itself is
         *          unnamed.
         */
        class ScopedDefaultLogger {
        public:
            explicit ScopedDefaultLogger(const std::string &pattern = "%n|%l|%v",
                                         const std::string &name = "test.default")
                : _previous(spdlog::default_logger()) {
                const auto sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(CaptureStream());
                const auto logger = std::make_shared<spdlog::logger>(name, sink);
                if (!pattern.empty()) {
                    logger->set_pattern(pattern);
                }
                logger->set_level(spdlog::level::trace);
                spdlog::set_default_logger(logger);
            }

            ~ScopedDefaultLogger() {
                spdlog::set_default_logger(_previous);
            }

            ScopedDefaultLogger(const ScopedDefaultLogger &) = delete;
            ScopedDefaultLogger &operator=(const ScopedDefaultLogger &) = delete;

        private:
            std::shared_ptr<spdlog::logger> _previous;
        };

        /**
         * @brief Sets the level of every logger, and restores it when done.
         * @details spdlog::set_level() is global, so a test that calls it must put
         *          the previous level back for the tests that follow.
         */
        class ScopedGlobalLevel {
        public:
            explicit ScopedGlobalLevel(const spdlog::level::level_enum level)
                : _previous(spdlog::get_level()) {
                spdlog::set_level(level);
            }

            ~ScopedGlobalLevel() {
                spdlog::set_level(_previous);
            }

            ScopedGlobalLevel(const ScopedGlobalLevel &) = delete;
            ScopedGlobalLevel &operator=(const ScopedGlobalLevel &) = delete;

        private:
            spdlog::level::level_enum _previous;
        };

        /**
         * @brief Gives one logger the given level, and restores its own when done.
         */
        class ScopedLoggerLevel {
        public:
            ScopedLoggerLevel(const std::shared_ptr<spdlog::logger> &logger, const spdlog::level::level_enum level)
                : _logger(logger), _previous(logger->level()) {
                _logger->set_level(level);
            }

            ~ScopedLoggerLevel() {
                _logger->set_level(_previous);
            }

            ScopedLoggerLevel(const ScopedLoggerLevel &) = delete;
            ScopedLoggerLevel &operator=(const ScopedLoggerLevel &) = delete;

        private:
            std::shared_ptr<spdlog::logger> _logger;
            spdlog::level::level_enum _previous;
        };

        /** @brief Sets an environment variable, on every supported platform. */
        void SetEnvironmentVariable(const char *name, const char *value) {
#ifdef _WIN32
            _putenv_s(name, value);
#else
            setenv(name, value, 1);
#endif
        }

        /** @brief Unsets an environment variable, on every supported platform. */
        void UnsetEnvironmentVariable(const char *name) {
#ifdef _WIN32
            _putenv_s(name, "");
#else
            unsetenv(name);
#endif
        }

        /**
         * @brief Sets an environment variable, and puts its previous value back
         *        when the guard goes out of scope.
         */
        class ScopedEnvironmentVariable {
        public:
            ScopedEnvironmentVariable(const char *name, const char *value)
                : _name(name), _was_set(std::getenv(name) != nullptr) {
                if (_was_set) {
                    _previous = std::getenv(name);
                }
                SetEnvironmentVariable(name, value);
            }

            ~ScopedEnvironmentVariable() {
                if (_was_set) {
                    SetEnvironmentVariable(_name.c_str(), _previous.c_str());
                } else {
                    UnsetEnvironmentVariable(_name.c_str());
                }
            }

            ScopedEnvironmentVariable(const ScopedEnvironmentVariable &) = delete;
            ScopedEnvironmentVariable &operator=(const ScopedEnvironmentVariable &) = delete;

        private:
            std::string _name;
            std::string _previous;
            bool _was_set;
        };
    } // namespace

    /**
     * Test 1: the example usage itself. Every log statement of the class carries
     * its category, so every message ends up on that category, with its own name,
     * and the global statement of the same class does not.
     */
    TEST(LogCategoryTest, ExampleClassLogsOnItsOwnCategory) {
        const LOG::Category category = CaptureExample();
        const std::string name = category.Name();

        LoggingExample example;
        geolio::test::LoggingExample::Run();

        EXPECT_TRUE(Contains(name + "|trace|trace message"));
        EXPECT_TRUE(Contains(name + "|debug|debug message 2"));
        EXPECT_TRUE(Contains(name + "|info|info message 3"));
        EXPECT_TRUE(Contains(name + "|warning|warn message"));
        EXPECT_TRUE(Contains(name + "|error|error message"));
        EXPECT_TRUE(Contains(name + "|critical|critical message"));

        // The global statement goes to the default logger, not to the category.
        EXPECT_FALSE(Contains(name + "|info|global message of the same class"));
    }

    /**
     * Test 2: silencing one class by name, from another translation unit. The
     * messages the class emits are dropped, and the global statements of that very
     * class are left alone; enabling it back restores its output.
     */
    TEST(LogCategoryTest, DisableOneClassByName) {
        const LOG::Category category = CaptureExample();
        const ScopedDefaultLogger default_logger;
        const std::string name = category.Name();

        LOG::Disable(name);
        EXPECT_FALSE(LOG::IsEnabled(name));
        EXPECT_FALSE(category.Enabled());

        LoggingExample example;
        geolio::test::LoggingExample::Run();
        EXPECT_FALSE(Contains(name + "|"));
        EXPECT_TRUE(Contains("test.default|info|global message of the same class"));

        LOG::Enable(name);
        EXPECT_TRUE(LOG::IsEnabled(name));

        ClearCapture();
        geolio::test::LoggingExample::Run();
        EXPECT_TRUE(Contains(name + "|info|info message 3"));
    }

    /**
     * Test 3: the category switch is independent of the spdlog level. Raising the
     * global level back to trace reopens the categories that were muted by their
     * level, but it must not reopen one that was switched off.
     */
    TEST(LogCategoryTest, DisableSurvivesAGlobalLevelChange) {
        const LOG::Category category = CaptureExample();
        const std::string name = category.Name();

        LOG::Disable(name);
        {
            const ScopedGlobalLevel level{spdlog::level::trace};
            EXPECT_EQ(spdlog::get_level(), spdlog::level::trace);

            LoggingExample example;
            geolio::test::LoggingExample::Run();
            EXPECT_FALSE(Contains(name + "|"));
        }

        LOG::Enable(name);
        ClearCapture();
        LoggingExample example;
        geolio::test::LoggingExample::Run();
        EXPECT_TRUE(Contains(name + "|trace|trace message"));
    }

    /**
     * Test 4: a category has its own level threshold, which filters the messages
     * of that class only and leaves the default logger at its own level.
     */
    TEST(LogCategoryTest, CategoryLevelFiltersItsOwnMessagesOnly) {
        const LOG::Category category = CaptureExample(spdlog::level::warn);
        const ScopedDefaultLogger default_logger;
        const std::string name = category.Name();

        LoggingExample example;
        geolio::test::LoggingExample::Run();

        EXPECT_EQ(category.Level(), spdlog::level::warn);
        EXPECT_FALSE(Contains(name + "|trace|"));
        EXPECT_FALSE(Contains(name + "|debug|"));
        EXPECT_FALSE(Contains(name + "|info|info message"));
        EXPECT_TRUE(Contains(name + "|warning|warn message"));
        EXPECT_TRUE(Contains(name + "|error|error message"));
        EXPECT_TRUE(Contains(name + "|critical|critical message"));

        EXPECT_TRUE(Contains("test.default|info|global message of the same class"));
    }

    /**
     * Test 5: a new category starts from the configuration of the application.
     * The level is taken from the default logger itself, not from the global level
     * of spdlog, and the sinks, the flush policy and the registry are shared with
     * it, so the two ways of logging cannot drift apart.
     */
    TEST(LogCategoryTest, NewCategoryInheritsTheDefaultLogger) {
        const auto default_logger = spdlog::default_logger();
        const ScopedLoggerLevel level{default_logger, spdlog::level::warn};

        const LOG::Category category{"test.log.inherited"};
        EXPECT_EQ(category.Level(), spdlog::level::warn);
        EXPECT_EQ(category.Logger()->sinks(), default_logger->sinks());
        EXPECT_EQ(category.Logger()->flush_level(), default_logger->flush_level());
        EXPECT_EQ(spdlog::get(category.Name()), category.Logger());
    }

    /**
     * Test 6: per-category levels can come from the environment, for the
     * categories that exist already and for the ones that are created later, and
     * a spec without a bare value leaves the global level of the application
     * alone.
     */
    TEST(LogCategoryTest, EnvironmentLevelsArePerCategory) {
        LOG::Category existing{"test.log.env_existing"};
        existing.SetLevel(spdlog::level::trace);

        {
            // No bare value in the spec: only the named loggers are touched.
            const ScopedEnvironmentVariable variable{"SPDLOG_LEVEL",
                                                     "test.log.env_existing=off,test.log.env_future=warn"};
            LOG::LoadEnvironmentLevels();

            // Applied to the category that already exists...
            EXPECT_EQ(existing.Level(), spdlog::level::off);

            // ... and remembered for the one created afterwards.
            const LOG::Category future{"test.log.env_future"};
            EXPECT_EQ(future.Level(), spdlog::level::warn);

            // A category that the spec does not name keeps the level of the
            // application.
            const LOG::Category untouched{"test.log.env_untouched"};
            EXPECT_EQ(untouched.Level(), spdlog::default_logger()->level());
        }

        // The variable holds its previous value again: reading it once more forgets
        // the levels of the spec above, so a category created from now on inherits
        // the level of the default logger again. A category that already exists
        // keeps the level it was given, exactly like a logger whose level was set
        // directly.
        LOG::LoadEnvironmentLevels();
        const LOG::Category afterwards{"test.log.env_afterwards"};
        EXPECT_EQ(afterwards.Level(), spdlog::default_logger()->level());
    }

    /**
     * Test 7: the source location of a log statement is still captured, on the
     * category form and on the global one, and it is the one of the caller rather
     * than the one inside log.h.
     */
    TEST(LogCategoryTest, CallSiteIsCaptured) {
        ClearCapture();
        const LOG::Category category = CapturedCategory("test.log.source");
        category.SetEnabled(true);
        category.SetLevel(spdlog::level::trace);
        category.Logger()->set_pattern("%s:%# %v");

        const int category_line = __LINE__ + 1;
        LOG::INFO(category, "category message");
        EXPECT_TRUE(Contains("test_log.cpp:" + std::to_string(category_line) + " category message"));

        {
            const ScopedDefaultLogger default_logger{"%s:%# %v"};
            const int global_line = __LINE__ + 1;
            LOG::INFO("global message");
            EXPECT_TRUE(Contains("test_log.cpp:" + std::to_string(global_line) + " global message"));
        }

        EXPECT_FALSE(Contains("log.h:"));
    }

    /**
     * Test 8: a category is identified by its name, so every handle of that name
     * -- in any class, object or translation unit -- drives the same logger, the
     * same switch and the same level, and it is a plain spdlog logger as well.
     */
    TEST(LogCategoryTest, HandlesOfTheSameCategoryShareTheirState) {
        const LOG::Category first{"test.log.shared"};
        const LOG::Category second{"test.log.shared"};

        EXPECT_EQ(first.Name(), second.Name());
        EXPECT_EQ(first.Logger(), second.Logger());
        EXPECT_EQ(spdlog::get(first.Name()), first.Logger());

        first.SetEnabled(false);
        EXPECT_FALSE(first.Enabled());
        EXPECT_FALSE(second.Enabled());
        EXPECT_FALSE(LOG::IsEnabled(first.Name()));

        second.SetLevel(spdlog::level::debug);
        EXPECT_EQ(first.Level(), spdlog::level::debug);

        second.SetEnabled(true);
        EXPECT_TRUE(first.Enabled());
    }

    /**
     * Test 9: the names of the categories can be listed, e.g. to build a
     * command-line flag, and a category that nobody mentioned yet counts as
     * enabled and is created as soon as it is mentioned.
     */
    TEST(LogCategoryTest, CategoriesAreListed) {
        const LOG::Category category{"test.log.listed"};
        const std::vector<std::string> names = LOG::Names();
        EXPECT_NE(std::find(names.begin(), names.end(), category.Name()), names.end());

        EXPECT_TRUE(LOG::IsEnabled("test.log.never_mentioned"));
        LOG::Disable("test.log.never_mentioned");

        const std::vector<std::string> names_after = LOG::Names();
        EXPECT_NE(std::find(names_after.begin(), names_after.end(), "test.log.never_mentioned"), names_after.end());
        EXPECT_FALSE(LOG::IsEnabled("test.log.never_mentioned"));
    }

    /**
     * Test 10: the name of a category is the name of its spdlog logger, so `%n`
     * prints it and the default pattern of spdlog already contains `%n`. A
     * category therefore shows up in the output of an application that sets no
     * pattern at all, and a statement that names no category -- the unnamed
     * default logger -- prints no name.
     */
    TEST(LogCategoryTest, CategoryNameIsPrintedLikeALoggerName) {
        ClearCapture();
        // An empty pattern keeps the default one: this is what a category looks
        // like in an application that never calls spdlog::set_pattern().
        const LOG::Category category = CapturedCategory("test.log.name", "");
        category.SetEnabled(true);
        category.SetLevel(spdlog::level::trace);

        const ScopedDefaultLogger default_logger{"", ""};

        LOG::INFO(category, "categorized message");
        LOG::INFO("uncategorized message");

        const std::string captured = CapturedText();
        std::cout << captured;

        EXPECT_TRUE(Contains("[" + category.Name() + "] [info] [test_log.cpp:"));
        EXPECT_TRUE(Contains("] categorized message"));
        EXPECT_TRUE(Contains("[info] [test_log.cpp:"));
        EXPECT_TRUE(Contains("] uncategorized message"));

        // The default logger has an empty name, and spdlog prints no bracket for
        // it rather than an empty one.
        EXPECT_FALSE(Contains("[]"));
    }

    /**
     * Test 11: the logger of a category is built from what its opener hands over --
     * the logger of that name that the opening module already has, or a clone of the
     * default logger of that module -- and never from a default logger that the
     * registry looks up on its own. That is what keeps the category of an
     * application configured like the application when geolio is a shared library
     * whose spdlog registry is not the one of the application, as on Windows.
     *
     * White box on purpose: through the public API the prototype is always the
     * default logger of the caller, and a single process cannot tell it apart from
     * the one the library would have looked up on its own.
     */
    TEST(LogCategoryTest, CategoryLoggerIsBuiltFromWhatTheCallerHandsOver) {
        ClearCapture();
        const auto sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(CaptureStream());

        // A category that does not exist yet is a clone of the prototype, with its
        // sinks, its flush policy and its level.
        const auto prototype = std::make_shared<spdlog::logger>("prototype", sink);
        prototype->set_level(spdlog::level::critical);
        prototype->flush_on(spdlog::level::debug);
        prototype->set_pattern("%n|%l|%v");

        const auto state = LOG::detail::GetOrCreateCategory("test.log.prototype", nullptr, prototype);
        EXPECT_EQ(state->logger->name(), "test.log.prototype");
        EXPECT_EQ(state->logger->level(), spdlog::level::critical);
        EXPECT_EQ(state->logger->flush_level(), spdlog::level::debug);
        EXPECT_EQ(state->logger->sinks(), prototype->sinks());

        // A logger of that name that the module already has is adopted as is.
        const auto own = std::make_shared<spdlog::logger>("test.log.own", sink);
        const auto own_state = LOG::detail::GetOrCreateCategory("test.log.own", own, prototype);
        EXPECT_EQ(own_state->logger, own);

        // And a category that already exists keeps the logger it was created with,
        // whatever another module hands over later on.
        const auto again = LOG::detail::GetOrCreateCategory("test.log.prototype", own, own);
        EXPECT_EQ(again->logger, state->logger);
        EXPECT_NE(again->logger, own);
    }
} // namespace geolio::test
