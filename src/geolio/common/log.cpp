//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/13.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <geolio/common/log.h>

#include <spdlog/cfg/env.h>
#include <spdlog/details/os.h>

#include <mutex>
#include <unordered_map>

namespace geolio::LOG {
    namespace detail {
        namespace {
            /**
             * @brief Lowercases a level name, as spdlog does before parsing it.
             * @param[in,out] str The string to lowercase in place.
             */
            void ToLower(std::string &str) {
                for (auto &ch: str) {
                    if (ch >= 'A' && ch <= 'Z') {
                        ch = static_cast<char>(ch + ('a' - 'A'));
                    }
                }
            }

            /**
             * @brief Removes the leading and trailing blanks of a spec entry.
             * @param[in] str The spec entry to trim.
             * @return The trimmed entry, empty when it is only blanks.
             */
            std::string Trim(const std::string &str) {
                const char *blanks = " \n\r\t";
                const auto first = str.find_first_not_of(blanks);
                if (first == std::string::npos) {
                    return {};
                }
                return str.substr(first, str.find_last_not_of(blanks) - first + 1);
            }

            /**
             * @brief Extracts the `name = level` entries of a SPDLOG_LEVEL spec.
             * @details The spec has the same shape as the one spdlog reads, e.g.
             *          `"off,demo.worker=debug"`: a bare value is the global level,
             *          and every `name = level` entry is the level of one logger.
             *          The global part is left to spdlog itself; only the per-name
             *          entries are returned here, because spdlog applies those to
             *          the loggers that already exist while this registry has to
             *          keep them for the categories that are created later on.
             * @param[in] spec The value of the environment variable.
             * @return The level of each named logger; unrecognized level names are
             *         ignored, exactly as spdlog does.
             */
            std::unordered_map<std::string, spdlog::level::level_enum> ParseCategoryLevels(const std::string &spec) {
                std::unordered_map<std::string, spdlog::level::level_enum> levels;
                std::size_t begin = 0;
                while (begin <= spec.size()) {
                    const auto comma = spec.find(',', begin);
                    const auto entry = Trim(spec.substr(begin, comma - begin));
                    begin = comma == std::string::npos ? spec.size() + 1 : comma + 1;

                    const auto equal = entry.find('=');
                    if (equal == std::string::npos) {
                        continue;
                    }

                    const auto name = Trim(entry.substr(0, equal));
                    auto level_name = Trim(entry.substr(equal + 1));
                    ToLower(level_name);
                    const auto level = spdlog::level::from_str(level_name);
                    // An unknown level name is reported as `off` by spdlog, and so
                    // is the literal "off": only the latter is a level to keep.
                    if (name.empty() || (level == spdlog::level::off && level_name != "off")) {
                        continue;
                    }
                    levels[name] = level;
                }
                return levels;
            }

            /**
             * @brief The categories of the running process, by name.
             * @details Deliberately defined in this translation unit rather than in
             *          the header: the registry is then shared by the geolio library,
             *          the applications and the tests, so a category switched off or
             *          retuned from one of them is observed by all of them. A
             *          header-only registry would instead be duplicated in every
             *          shared object on platforms without symbol interposition.
             */
            class CategoryRegistry {
            public:
                /**
                 * @brief Returns the single registry of the process.
                 * @return The registry instance.
                 */
                static CategoryRegistry &Instance() {
                    static CategoryRegistry registry;
                    return registry;
                }

                /**
                 * @brief Returns the state of one category, creating it on first use.
                 * @param[in] name The category name.
                 * @return The shared state of that category; never null.
                 */
                std::shared_ptr<CategoryState> GetOrCreate(const std::string &name) {
                    std::lock_guard<std::mutex> lock(_mutex);
                    if (const auto it = _states.find(name); it != _states.end()) {
                        return it->second;
                    }

                    auto state = std::make_shared<CategoryState>();
                    state->name = name;
                    state->logger = MakeLogger(name);
                    _states.emplace(name, state);
                    return state;
                }

                /**
                 * @brief Returns the state of one category without creating it.
                 * @param[in] name The category name.
                 * @return The shared state, or nullptr when that category is unknown.
                 */
                std::shared_ptr<CategoryState> Find(const std::string &name) const {
                    std::lock_guard<std::mutex> lock(_mutex);
                    const auto it = _states.find(name);
                    return it == _states.end() ? nullptr : it->second;
                }

                /**
                 * @brief Returns the names of every category created so far.
                 * @return The category names, in unspecified order.
                 */
                std::vector<std::string> Names() const {
                    std::lock_guard<std::mutex> lock(_mutex);
                    std::vector<std::string> names;
                    names.reserve(_states.size());
                    for (const auto &[name, state]: _states) {
                        names.push_back(name);
                    }
                    return names;
                }

                /**
                 * @brief Replaces the level of every named logger of the spec.
                 * @param[in] levels The level of each named logger of the spec.
                 */
                void SetLevels(std::unordered_map<std::string, spdlog::level::level_enum> levels) {
                    std::lock_guard<std::mutex> lock(_mutex);
                    _levels = std::move(levels);
                }

            private:
                /**
                 * @brief Creates the spdlog logger backing a new category.
                 * @details A logger that already carries that name is reused as is,
                 *          so a category is exactly an spdlog logger: spdlog::get()
                 *          finds it, and a level given to it before the first log
                 *          call is respected. A new logger is a clone of the default
                 *          logger, hence it starts with the sinks, the formatter, the
                 *          level and the flush policy of the application, and it is
                 *          registered so that spdlog::set_level(), spdlog::set_pattern()
                 *          and spdlog::apply_all() keep acting on it. The clone is
                 *          registered directly, without spdlog::initialize_logger():
                 *          that one would reset the shared sinks to the pattern of
                 *          the registry, silently reformatting the output of the
                 *          default logger, and would drop a level that was set on the
                 *          default logger itself instead of globally.
                 * @param[in] name The category name.
                 * @return The logger of that category; never null.
                 */
                std::shared_ptr<spdlog::logger> MakeLogger(const std::string &name) {
                    if (auto logger = spdlog::get(name)) {
                        return logger;
                    }

                    auto logger = spdlog::default_logger()->clone(name);
                    spdlog::register_logger(logger);

                    // A named entry of SPDLOG_LEVEL outranks the inherited level,
                    // including for a category that is only created later on.
                    if (const auto it = _levels.find(name); it != _levels.end()) {
                        logger->set_level(it->second);
                    }
                    return logger;
                }

                /** Guards _states and _levels, which the log calls read. */
                mutable std::mutex _mutex;

                /** The state of every category mentioned so far, by name. */
                std::unordered_map<std::string, std::shared_ptr<CategoryState>> _states;

                /** The level of each named logger of the last SPDLOG_LEVEL spec. */
                std::unordered_map<std::string, spdlog::level::level_enum> _levels;
            };
        } // namespace

        std::shared_ptr<CategoryState> GetOrCreateCategory(const std::string &name) {
            return CategoryRegistry::Instance().GetOrCreate(name);
        }

        std::shared_ptr<CategoryState> FindCategory(const std::string &name) {
            return CategoryRegistry::Instance().Find(name);
        }

        std::vector<std::string> CategoryNames() {
            return CategoryRegistry::Instance().Names();
        }

        void LoadEnvironmentLevels(const char *variable) {
            // spdlog applies the global level of the spec, and the per-name levels
            // of the loggers that already exist; the per-name levels are kept here
            // as well, for the categories created after this call. An unset or
            // empty variable forgets the levels that were remembered from an
            // earlier spec, so that reading the environment twice is idempotent.
            const auto spec = spdlog::details::os::getenv(variable);
            if (!spec.empty()) {
                spdlog::cfg::load_env_levels(variable);
            }
            CategoryRegistry::Instance().SetLevels(ParseCategoryLevels(spec));
        }
    } // namespace detail
} // namespace LOG
