//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/26.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_LOG_H
#define GEOLIO_LOG_H

#include <spdlog/spdlog.h>
#include <spdlog/sinks/base_sink.h>
#include <spdlog/sinks/sink.h>

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#ifdef ERROR
#undef ERROR
#endif

namespace geolio::LOG {
    namespace detail {
        /**
         * @brief Captures the file name, function name and line number of a
         *        log call site, captured at compile time via compiler builtins.
         */
        class SourceLocation {
        public:
            /**
             * @brief Constructs a SourceLocation from the given call-site info.
             * @details Defaults to the compiler's builtin FILE/FUNCTION/LINE
             *          macros, so a log statement can be located automatically.
             * @param[in] fileName The source file name.
             * @param[in] funcName The enclosing function name.
             * @param[in] lineNum The line number in the source file.
             */
            constexpr SourceLocation(const char *fileName = __builtin_FILE(),
                                     const char *funcName = __builtin_FUNCTION(),
                                     std::uint32_t lineNum = __builtin_LINE()) noexcept
                    : _fileName(fileName), _funcName(funcName), _lineNum(lineNum) {
            }

            /**
             * @brief Returns the captured source file name.
             * @return The source file name.
             */
            [[nodiscard]] constexpr const char *FileName() const noexcept {
                return _fileName;
            }

            /**
             * @brief Returns the captured enclosing function name.
             * @return The function name.
             */
            [[nodiscard]] constexpr const char *FuncName() const noexcept {
                return _funcName;
            }

            /**
             * @brief Returns the captured source line number.
             * @return The source line number.
             */
            [[nodiscard]] constexpr std::uint32_t LineNum() const noexcept {
                return _lineNum;
            }

        private:
            const char *_fileName;
            const char *_funcName;
            const std::uint32_t _lineNum;
        };

        /**
         * @brief Converts a SourceLocation into an spdlog source_loc.
         * @details Extracts the file name and line number from the location and
         *          passes them together with the function name to the spdlog
         *          source_loc constructor.
         * @param[in] location The compile-time captured call-site location.
         * @return The corresponding spdlog::source_loc.
         */
        constexpr auto GetLogSourceLocation(const SourceLocation &location) {
            return spdlog::source_loc{location.FileName(), static_cast<int>(location.LineNum()),
                                      location.FuncName()};
        }

        /**
         * @brief The process-wide state of one logging category.
         * @details The name, the underlying spdlog logger and the category's own
         *          on/off switch live here instead of inside Category, so every
         *          handle of the same category -- one per class, per object or per
         *          translation unit -- observes and controls the very same state.
         */
        struct CategoryState {
            /** The category name, which is also the underlying spdlog logger name. */
            std::string name;

            /** The underlying spdlog logger, derived from the default logger. */
            std::shared_ptr<spdlog::logger> logger;

            /** The category-level switch; independent of the spdlog level. */
            std::atomic<bool> enabled{true};
        };

        /**
         * @brief Returns the state of the given category, creating it on first use.
         * @details Defined in log.cpp on purpose: a single registry then serves the
         *          geolio library, the applications and the tests, whereas a
         *          header-only registry would be duplicated in every shared object
         *          on platforms without symbol interposition, so that a category
         *          disabled from one module would keep logging in another one.
         * @param[in] name The category name.
         * @return The shared state of that category; never null.
         */
        std::shared_ptr<CategoryState> GetOrCreateCategory(const std::string &name);

        /**
         * @brief Returns the state of the given category without creating it.
         * @param[in] name The category name.
         * @return The shared state, or nullptr when that category does not exist yet.
         */
        std::shared_ptr<CategoryState> FindCategory(const std::string &name);

        /**
         * @brief Returns the names of every category created so far.
         * @return The category names, in unspecified order.
         */
        std::vector<std::string> CategoryNames();

        /**
         * @brief Applies the level specification found in an environment variable.
         * @details Also defined in log.cpp so that the parsed levels are stored in
         *          the same spdlog registry that creates the category loggers.
         * @param[in] variable The environment variable to read.
         */
        void LoadEnvironmentLevels(const char *variable);
    } // namespace detail

    /**
     * @brief A named logger giving one class its own logging configuration.
     * @details A Category is a handle on the process-wide state of a category (see
     *          detail::CategoryState). It wraps a named spdlog logger that shares
     *          the sinks, the formatter and the flush policy of the default logger,
     *          plus an independent on/off switch. Everything logged through the
     *          same category name -- from any class, object or translation unit --
     *          is therefore configured as a whole:
     *
     *            - Category::SetEnabled(false), or LOG::Disable(name), silences
     *              exactly that category, whatever the global level becomes;
     *            - Category::SetLevel() gives the category its own threshold; a
     *              new category otherwise starts at the level of the default
     *              logger, unless its SPDLOG_LEVEL entry says otherwise, and it
     *              follows the later spdlog::set_level() calls;
     *            - the category *is* a plain spdlog logger named after the category,
     *              so spdlog::get(name), spdlog::set_level(), spdlog::set_pattern(),
     *              spdlog::apply_all() and `SPDLOG_LEVEL=info,name=off` all act on it;
     *            - that name is what `%n` prints, and the default pattern of spdlog
     *              contains `%n`, so a category already shows up in the output
     *              without any setup, where the unnamed default logger shows no
     *              name at all:
     * @code
     *   [2026-10-13 12:00:00.000] [info] [frame_field.cpp:42] a global message
     *   [2026-10-13 12:00:00.001] [frame_field] [info] [frame_field.cpp:43] a message of that class
     * @endcode
     *              An application that sets a pattern of its own must keep `%n`
     *              in it, e.g. spdlog::set_pattern("[%n] [%l] %v"), otherwise the
     *              name of the category is dropped along with the other fields
     *              that the pattern does not mention.
     *
     * @code
     *   class FrameField {
     *   public:
     *       static LOG::Category &Logger() {
     *           static LOG::Category category{"frame_field"};
     *           return category;
     *       }
     *
     *       void compute() {
     *           LOG::DEBUG(Logger(), "found {} facets", nb);   // this class only
     *           LOG::DEBUG("global message");                  // unchanged, global
     *       }
     *   };
     *
     *   LOG::Disable("frame_field");   // from anywhere: silence that class only
     *   LOG::SetLevel("frame_field", spdlog::level::debug);   // or retune it
     * @endcode
     *
     * @note The handle is cheap to copy, and a copy refers to the same category:
     *       the switch and the level are shared, not per handle.
     * @note Prefer the function-local static accessor shown above over an inline
     *       static member: the underlying logger is then created on first use, so
     *       it inherits a default logger (sinks, pattern) installed at startup.
     */
    class Category {
    public:
        /**
         * @brief Opens the category with the given name, creating it on first use.
         * @details The underlying spdlog logger is created once per process, as an
         *          spdlog logger of that name cloned from the default logger, and is
         *          registered so that spdlog::get(name) finds it afterwards. It
         *          therefore starts with the sinks, the pattern, the level and the
         *          flush policy the application has at that moment, and it follows
         *          every later spdlog::set_level(), spdlog::set_pattern() and
         *          spdlog::apply_all() call. A logger of that name that already
         *          exists is reused as is.
         * @param[in] name The category name, used as the spdlog logger name.
         */
        explicit Category(const std::string_view name)
                : _state(detail::GetOrCreateCategory(std::string{name})) {
        }

        /**
         * @brief Returns the category name.
         * @return The name, which is also the underlying spdlog logger name.
         */
        [[nodiscard]] const std::string &Name() const noexcept {
            return _state->name;
        }

        /**
         * @brief Returns the underlying spdlog logger.
         * @details Escape hatch for what the category API does not cover, e.g.
         *          replacing its sinks with a dedicated file sink or setting a
         *          category-specific pattern.
         * @return The underlying spdlog logger; never null.
         */
        [[nodiscard]] const std::shared_ptr<spdlog::logger> &Logger() const noexcept {
            return _state->logger;
        }

        /**
         * @brief Tells whether this category currently logs.
         * @return True when the category is switched on; the spdlog level still
         *         filters the messages that are actually emitted.
         */
        [[nodiscard]] bool Enabled() const noexcept {
            return _state->enabled.load(std::memory_order_relaxed);
        }

        /**
         * @brief Switches this category on or off.
         * @details This is a hard, category-scoped mute: it is not undone by a later
         *          spdlog::set_level(), unlike setting the level to off, since the
         *          global level of spdlog is applied to every registered logger.
         * @param[in] enabled False to silence the category, true to allow it again.
         */
        void SetEnabled(const bool enabled) const noexcept {
            _state->enabled.store(enabled, std::memory_order_relaxed);
        }

        /** @brief Switches this category on. */
        void Enable() const noexcept {
            SetEnabled(true);
        }

        /** @brief Switches this category off, so that it logs nothing. */
        void Disable() const noexcept {
            SetEnabled(false);
        }

        /**
         * @brief Returns the log level threshold of this category.
         * @return The lowest level this category currently emits.
         */
        [[nodiscard]] spdlog::level::level_enum Level() const {
            return _state->logger->level();
        }

        /**
         * @brief Sets the log level threshold of this category only.
         * @param[in] level The lowest level this category should emit.
         */
        void SetLevel(const spdlog::level::level_enum level) const {
            _state->logger->set_level(level);
        }

        /** @brief Flushes the pending messages of this category. */
        void Flush() const {
            _state->logger->flush();
        }

        /**
         * @brief Emits one message of this category, with the given call site.
         * @details This is the back end of the LOG::TRACE, LOG::DEBUG, ... tags when
         *          they are handed a category: it checks the category switch first,
         *          then the level of the logger, so that a silenced category costs
         *          nothing but two atomic loads. It is public because those tags are
         *          free templates, and it can be called directly when the message of
         *          a log statement is built elsewhere.
         * @param[in] level The level of the message.
         * @param[in] location The call-site location of the log statement.
         * @param[in] fmt The format string describing the message.
         * @param[in] args The arguments to be formatted into the message.
         */
        template<typename... Args>
        void Emit(const spdlog::level::level_enum level, const detail::SourceLocation &location,
                  fmt::format_string<Args...> fmt, Args &&...args) const {
            if (!Enabled() || !_state->logger->should_log(level)) {
                return;
            }
            _state->logger->log(detail::GetLogSourceLocation(location), level, fmt, std::forward<Args>(args)...);
        }

    private:
        /** The shared state of this category. */
        std::shared_ptr<detail::CategoryState> _state;
    };

    /**
     * @brief Switches one category on or off, creating it on first use.
     * @details Convenience for configurations where no Category handle is at hand,
     *          e.g. a command-line flag of an application.
     * @param[in] category The category name.
     * @param[in] enabled False to silence that category, true to allow it again.
     */
    inline void SetEnabled(const std::string_view category, const bool enabled) {
        Category{category}.SetEnabled(enabled);
    }

    /**
     * @brief Switches one category on, creating it on first use.
     * @param[in] category The category name.
     */
    inline void Enable(const std::string_view category) {
        SetEnabled(category, true);
    }

    /**
     * @brief Silences one category, creating it on first use.
     * @param[in] category The category name.
     */
    inline void Disable(const std::string_view category) {
        SetEnabled(category, false);
    }

    /**
     * @brief Sets the level threshold of one category, creating it on first use.
     * @param[in] category The category name.
     * @param[in] level The lowest level that category should emit.
     */
    inline void SetLevel(const std::string_view category, const spdlog::level::level_enum level) {
        Category{category}.SetLevel(level);
    }

    /**
     * @brief Tells whether one category currently logs.
     * @details A category that was never mentioned is enabled by default.
     * @param[in] category The category name.
     * @return True when that category is switched on.
     */
    [[nodiscard]] inline bool IsEnabled(const std::string_view category) {
        const auto state = detail::FindCategory(std::string{category});
        return state == nullptr || state->enabled.load(std::memory_order_relaxed);
    }

    /**
     * @brief Returns the names of every category created so far.
     * @return The category names, in unspecified order.
     */
    [[nodiscard]] inline std::vector<std::string> Names() {
        return detail::CategoryNames();
    }

    /**
     * @brief Applies per-category levels read from an environment variable.
     * @details Lets a user retune the logging of a class without recompiling, e.g.
     *          @code
     *            SPDLOG_LEVEL="info,frame_field=off,mesh_object=debug" ./my_app
     *          @endcode
     *          where the bare `info` is the global level and every `name=level`
     *          entry is a category. Call it once, at startup, after the level of
     *          the application has been set, since the last writer wins.
     * @param[in] variable The environment variable to read, SPDLOG_LEVEL by default.
     */
    inline void LoadEnvironmentLevels(const char *variable = "SPDLOG_LEVEL") {
        detail::LoadEnvironmentLevels(variable);
    }

    /**
     * @brief Logs a trace-level message, including the call-site location.
     * @details Wraps spdlog's trace logging so that each log statement
     *          automatically carries its compile-time source location. Given a
     *          Category as its first argument, the message goes to that category
     *          instead of the default logger, and obeys the switch and the level
     *          of that category:
     * @code
     *   LOG::TRACE("message of the whole application");
     *   LOG::TRACE(Logger(), "message of this class only");
     * @endcode
     */
    template<typename... Args>
    struct TRACE {
        /**
         * @brief Emits a trace-level log message on the default logger.
         * @details Forwards the formatted message and arguments to spdlog with
         *          the given (or default-captured) source location.
         * @param[in] fmt The format string describing the message.
         * @param[in] args The arguments to be formatted into the message.
         * @param[in] location The call-site location, auto-captured if omitted.
         */
        explicit constexpr TRACE(fmt::format_string<Args...> fmt, Args &&...args, const detail::SourceLocation &location = {}) {
            spdlog::log(GetLogSourceLocation(location), spdlog::level::trace, fmt, std::forward<Args>(args)...);
        }

        /**
         * @brief Emits a trace-level log message on the given category.
         * @details Goes through Category::Emit(), so the message is dropped when
         *          that category is switched off or its level filters it out.
         * @param[in] category The category the message belongs to.
         * @param[in] fmt The format string describing the message.
         * @param[in] args The arguments to be formatted into the message.
         * @param[in] location The call-site location, auto-captured if omitted.
         */
        explicit constexpr TRACE(const Category &category, fmt::format_string<Args...> fmt, Args &&...args, const detail::SourceLocation &location = {}) {
            category.Emit(spdlog::level::trace, location, fmt, std::forward<Args>(args)...);
        }
    };

    /**
     * @brief CTAD deduction guide for TRACE.
     * @details Deduces Args from the format string and the forwarded arguments.
     */
    template<typename... Args>
    TRACE(fmt::format_string<Args...> fmt, Args &&...args) -> TRACE<Args...>;

    /**
     * @brief CTAD deduction guide for the category form of TRACE.
     * @details Deduces Args from the format string and the forwarded arguments,
     *          the leading category being already a complete type.
     */
    template<typename... Args>
    TRACE(const Category &category, fmt::format_string<Args...> fmt, Args &&...args) -> TRACE<Args...>;

    /**
     * @brief Logs a debug-level message, including the call-site location.
     * @details Wraps spdlog's debug logging so that each log statement
     *          automatically carries its compile-time source location. Given a
     *          Category as its first argument, the message goes to that category
     *          instead of the default logger, and obeys the switch and the level
     *          of that category.
     */
    template<typename... Args>
    struct DEBUG {
        /**
         * @brief Emits a debug-level log message on the default logger.
         * @details Forwards the formatted message and arguments to spdlog with
         *          the given (or default-captured) source location.
         * @param[in] fmt The format string describing the message.
         * @param[in] args The arguments to be formatted into the message.
         * @param[in] location The call-site location, auto-captured if omitted.
         */
        explicit constexpr DEBUG(fmt::format_string<Args...> fmt, Args &&...args, const detail::SourceLocation &location = {}) {
            spdlog::log(GetLogSourceLocation(location), spdlog::level::debug, fmt, std::forward<Args>(args)...);
        }

        /**
         * @brief Emits a debug-level log message on the given category.
         * @details Goes through Category::Emit(), so the message is dropped when
         *          that category is switched off or its level filters it out.
         * @param[in] category The category the message belongs to.
         * @param[in] fmt The format string describing the message.
         * @param[in] args The arguments to be formatted into the message.
         * @param[in] location The call-site location, auto-captured if omitted.
         */
        explicit constexpr DEBUG(const Category &category, fmt::format_string<Args...> fmt, Args &&...args, const detail::SourceLocation &location = {}) {
            category.Emit(spdlog::level::debug, location, fmt, std::forward<Args>(args)...);
        }
    };

    /**
     * @brief CTAD deduction guide for DEBUG.
     * @details Deduces Args from the format string and the forwarded arguments.
     */
    template<typename... Args>
    DEBUG(fmt::format_string<Args...> fmt, Args &&...args) -> DEBUG<Args...>;

    /**
     * @brief CTAD deduction guide for the category form of DEBUG.
     * @details Deduces Args from the format string and the forwarded arguments,
     *          the leading category being already a complete type.
     */
    template<typename... Args>
    DEBUG(const Category &category, fmt::format_string<Args...> fmt, Args &&...args) -> DEBUG<Args...>;

    /**
     * @brief Logs an info-level message, including the call-site location.
     * @details Wraps spdlog's info logging so that each log statement
     *          automatically carries its compile-time source location. Given a
     *          Category as its first argument, the message goes to that category
     *          instead of the default logger, and obeys the switch and the level
     *          of that category.
     */
    template<typename... Args>
    struct INFO {
        /**
         * @brief Emits an info-level log message on the default logger.
         * @details Forwards the formatted message and arguments to spdlog with
         *          the given (or default-captured) source location.
         * @param[in] fmt The format string describing the message.
         * @param[in] args The arguments to be formatted into the message.
         * @param[in] location The call-site location, auto-captured if omitted.
         */
        explicit constexpr INFO(fmt::format_string<Args...> fmt, Args &&...args, const detail::SourceLocation &location = {}) {
            spdlog::log(GetLogSourceLocation(location), spdlog::level::info, fmt, std::forward<Args>(args)...);
        }

        /**
         * @brief Emits an info-level log message on the given category.
         * @details Goes through Category::Emit(), so the message is dropped when
         *          that category is switched off or its level filters it out.
         * @param[in] category The category the message belongs to.
         * @param[in] fmt The format string describing the message.
         * @param[in] args The arguments to be formatted into the message.
         * @param[in] location The call-site location, auto-captured if omitted.
         */
        explicit constexpr INFO(const Category &category, fmt::format_string<Args...> fmt, Args &&...args, const detail::SourceLocation &location = {}) {
            category.Emit(spdlog::level::info, location, fmt, std::forward<Args>(args)...);
        }
    };

    /**
     * @brief CTAD deduction guide for INFO.
     * @details Deduces Args from the format string and the forwarded arguments.
     */
    template<typename... Args>
    INFO(fmt::format_string<Args...> fmt, Args &&...args) -> INFO<Args...>;

    /**
     * @brief CTAD deduction guide for the category form of INFO.
     * @details Deduces Args from the format string and the forwarded arguments,
     *          the leading category being already a complete type.
     */
    template<typename... Args>
    INFO(const Category &category, fmt::format_string<Args...> fmt, Args &&...args) -> INFO<Args...>;

    /**
     * @brief Logs a warning-level message, including the call-site location.
     * @details Wraps spdlog's warning logging so that each log statement
     *          automatically carries its compile-time source location. Given a
     *          Category as its first argument, the message goes to that category
     *          instead of the default logger, and obeys the switch and the level
     *          of that category.
     */
    template<typename... Args>
    struct WARN {
        /**
         * @brief Emits a warning-level log message on the default logger.
         * @details Forwards the formatted message and arguments to spdlog with
         *          the given (or default-captured) source location.
         * @param[in] fmt The format string describing the message.
         * @param[in] args The arguments to be formatted into the message.
         * @param[in] location The call-site location, auto-captured if omitted.
         */
        explicit constexpr WARN(fmt::format_string<Args...> fmt, Args &&...args, const detail::SourceLocation &location = {}) {
            spdlog::log(GetLogSourceLocation(location), spdlog::level::warn, fmt, std::forward<Args>(args)...);
        }

        /**
         * @brief Emits a warning-level log message on the given category.
         * @details Goes through Category::Emit(), so the message is dropped when
         *          that category is switched off or its level filters it out.
         * @param[in] category The category the message belongs to.
         * @param[in] fmt The format string describing the message.
         * @param[in] args The arguments to be formatted into the message.
         * @param[in] location The call-site location, auto-captured if omitted.
         */
        explicit constexpr WARN(const Category &category, fmt::format_string<Args...> fmt, Args &&...args, const detail::SourceLocation &location = {}) {
            category.Emit(spdlog::level::warn, location, fmt, std::forward<Args>(args)...);
        }
    };

    /**
     * @brief CTAD deduction guide for WARN.
     * @details Deduces Args from the format string and the forwarded arguments.
     */
    template<typename... Args>
    WARN(fmt::format_string<Args...> fmt, Args &&...args) -> WARN<Args...>;

    /**
     * @brief CTAD deduction guide for the category form of WARN.
     * @details Deduces Args from the format string and the forwarded arguments,
     *          the leading category being already a complete type.
     */
    template<typename... Args>
    WARN(const Category &category, fmt::format_string<Args...> fmt, Args &&...args) -> WARN<Args...>;

    /**
     * @brief Logs an error-level message, including the call-site location.
     * @details Wraps spdlog's error logging so that each log statement
     *          automatically carries its compile-time source location. Given a
     *          Category as its first argument, the message goes to that category
     *          instead of the default logger, and obeys the switch and the level
     *          of that category.
     */
    template<typename... Args>
    struct ERROR {
        /**
         * @brief Emits an error-level log message on the default logger.
         * @details Forwards the formatted message and arguments to spdlog with
         *          the given (or default-captured) source location.
         * @param[in] fmt The format string describing the message.
         * @param[in] args The arguments to be formatted into the message.
         * @param[in] location The call-site location, auto-captured if omitted.
         */
        explicit constexpr ERROR(fmt::format_string<Args...> fmt, Args &&...args, const detail::SourceLocation &location = {}) {
            spdlog::log(GetLogSourceLocation(location), spdlog::level::err, fmt, std::forward<Args>(args)...);
        }

        /**
         * @brief Emits an error-level log message on the given category.
         * @details Goes through Category::Emit(), so the message is dropped when
         *          that category is switched off or its level filters it out.
         * @param[in] category The category the message belongs to.
         * @param[in] fmt The format string describing the message.
         * @param[in] args The arguments to be formatted into the message.
         * @param[in] location The call-site location, auto-captured if omitted.
         */
        explicit constexpr ERROR(const Category &category, fmt::format_string<Args...> fmt, Args &&...args, const detail::SourceLocation &location = {}) {
            category.Emit(spdlog::level::err, location, fmt, std::forward<Args>(args)...);
        }
    };

    /**
     * @brief CTAD deduction guide for ERROR.
     * @details Deduces Args from the format string and the forwarded arguments.
     */
    template<typename... Args>
    ERROR(fmt::format_string<Args...> fmt, Args &&...args) -> ERROR<Args...>;

    /**
     * @brief CTAD deduction guide for the category form of ERROR.
     * @details Deduces Args from the format string and the forwarded arguments,
     *          the leading category being already a complete type.
     */
    template<typename... Args>
    ERROR(const Category &category, fmt::format_string<Args...> fmt, Args &&...args) -> ERROR<Args...>;

    /**
     * @brief Logs a critical-level message, including the call-site location.
     * @details Wraps spdlog's critical logging so that each log statement
     *          automatically carries its compile-time source location. Given a
     *          Category as its first argument, the message goes to that category
     *          instead of the default logger, and obeys the switch and the level
     *          of that category.
     */
    template<typename... Args>
    struct CRITICAL {
        /**
         * @brief Emits a critical-level log message on the default logger.
         * @details Forwards the formatted message and arguments to spdlog with
         *          the given (or default-captured) source location.
         * @param[in] fmt The format string describing the message.
         * @param[in] args The arguments to be formatted into the message.
         * @param[in] location The call-site location, auto-captured if omitted.
         */
        explicit constexpr CRITICAL(fmt::format_string<Args...> fmt, Args &&...args, const detail::SourceLocation &location = {}) {
            spdlog::log(GetLogSourceLocation(location), spdlog::level::critical, fmt, std::forward<Args>(args)...);
        }

        /**
         * @brief Emits a critical-level log message on the given category.
         * @details Goes through Category::Emit(), so the message is dropped when
         *          that category is switched off or its level filters it out.
         * @param[in] category The category the message belongs to.
         * @param[in] fmt The format string describing the message.
         * @param[in] args The arguments to be formatted into the message.
         * @param[in] location The call-site location, auto-captured if omitted.
         */
        explicit constexpr CRITICAL(const Category &category, fmt::format_string<Args...> fmt, Args &&...args, const detail::SourceLocation &location = {}) {
            category.Emit(spdlog::level::critical, location, fmt, std::forward<Args>(args)...);
        }
    };

    /**
     * @brief CTAD deduction guide for CRITICAL.
     * @details Deduces Args from the format string and the forwarded arguments.
     */
    template<typename... Args>
    CRITICAL(fmt::format_string<Args...> fmt, Args &&...args) -> CRITICAL<Args...>;

    /**
     * @brief CTAD deduction guide for the category form of CRITICAL.
     * @details Deduces Args from the format string and the forwarded arguments,
     *          the leading category being already a complete type.
     */
    template<typename... Args>
    CRITICAL(const Category &category, fmt::format_string<Args...> fmt, Args &&...args) -> CRITICAL<Args...>;
} // namespace LOG

#endif //GEOLIO_LOG_H
