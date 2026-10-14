//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/13.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <geolio/common/log.h>

#include <spdlog/cfg/env.h>
#include <spdlog/details/os.h>

#include <mutex>
#include <optional>
#include <unordered_map>

namespace geolio::LOG {
    namespace detail {
        namespace {
            /**
             * @brief The levels that a SPDLOG_LEVEL specification asks for.
             */
            struct LevelSpec {
                /** The level of each named logger; `off` silences that logger. */
                std::unordered_map<std::string, spdlog::level::level_enum> named;

                /** The level of the loggers that no entry names, when the spec has one. */
                std::optional<spdlog::level::level_enum> global;
            };

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
             * @brief Parses a SPDLOG_LEVEL specification.
             * @details spdlog reads the same specification, but only applies it to
             *          the loggers of its own registry; the categories of a module
             *          whose registry is a different one -- which is the case of a
             *          shared geolio library whose spdlog is header-only -- keep the
             *          result here instead. The shape is the one spdlog documents: a
             *          comma separated list whose bare value is the global level and
             *          whose `name = level` entries are the level of one logger, e.g.
             *          `"off,frame_field=debug"`. An unrecognized level name is
             *          ignored, exactly as spdlog does.
             * @param[in] spec The value of the environment variable.
             * @return The levels that the specification asks for.
             */
            LevelSpec ParseLevelSpec(const std::string &spec) {
                LevelSpec levels;
                std::size_t begin = 0;
                while (begin <= spec.size()) {
                    const auto comma = spec.find(',', begin);
                    const auto entry = Trim(spec.substr(begin, comma - begin));
                    begin = comma == std::string::npos ? spec.size() + 1 : comma + 1;
                    if (entry.empty()) {
                        continue;
                    }

                    const auto equal = entry.find('=');
                    const auto name = equal == std::string::npos ? std::string{} : Trim(entry.substr(0, equal));
                    auto level_name = Trim(entry.substr(equal == std::string::npos ? 0 : equal + 1));
                    ToLower(level_name);
                    // An unknown level name is reported as `off` by spdlog, and so is
                    // the literal "off": only the latter is a level to keep.
                    const auto level = spdlog::level::from_str(level_name);
                    if (level == spdlog::level::off && level_name != "off") {
                        continue;
                    }

                    if (name.empty()) {
                        levels.global = level;
                    } else {
                        levels.named[name] = level;
                    }
                }
                return levels;
            }

            /**
             * @brief The categories of the running process, by name.
             * @details Deliberately defined in this translation unit rather than in
             *          the header: the state of a category is then shared by the
             *          geolio library, the applications and the tests, so a category
             *          switched off or retuned from one of them is observed by all of
             *          them. A header-only state would instead be duplicated in every
             *          shared object on platforms without symbol interposition.
             *
             *          The spdlog logger of a category comes from the module that
             *          opens it (see AdoptCategoryLogger() and GetOrCreateCategory()
             *          in the header), since the spdlog registries themselves are not
             *          shared on those platforms.
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
                 * @param[in] existing A logger of that name that the calling module
                 *            already has, or nullptr to create one from @p prototype.
                 * @param[in] prototype The default logger of the calling module.
                 * @return The shared state of that category; never null.
                 */
                std::shared_ptr<CategoryState> GetOrCreate(const std::string &name,
                                                          std::shared_ptr<spdlog::logger> existing,
                                                          const std::shared_ptr<spdlog::logger> &prototype) {
                    std::lock_guard<std::mutex> lock(_mutex);
                    if (const auto it = _states.find(name); it != _states.end()) {
                        return it->second;
                    }

                    auto state = std::make_shared<CategoryState>();
                    state->name = name;
                    // A clone of the default logger of the calling module: same sinks,
                    // same formatter, same flush policy, same starting level. Cloning
                    // is what keeps a category of the application configured like the
                    // rest of the application, including where the spdlog registry of
                    // the library is not the one of the application.
                    state->logger = existing != nullptr ? std::move(existing) : prototype->clone(name);
                    ApplyLevel(_levels, *state);
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
                 * @brief Remembers the levels of a spec, and applies them to the
                 *        categories that already exist.
                 * @details spdlog applies the same specification to the loggers of
                 *          its own registry, which does not contain the categories of
                 *          another module when the registries are split, so this
                 *          registry applies them to its categories as well.
                 * @param[in] spec The levels of the specification that was read.
                 */
                void SetLevels(LevelSpec spec) {
                    std::lock_guard<std::mutex> lock(_mutex);
                    for (auto &[name, state]: _states) {
                        ApplyLevel(spec, *state);
                    }
                    _levels = std::move(spec);
                }

            private:
                /**
                 * @brief Gives one category the level that a spec has for it.
                 * @details Mirrors the choice spdlog makes for a new logger: the
                 *          entry that names the category wins, then the global entry,
                 *          and a category that neither names keeps the level it has.
                 * @param[in] spec The specification to read the level from.
                 * @param[in,out] state The state of the category to retune.
                 */
                static void ApplyLevel(const LevelSpec &spec, CategoryState &state) {
                    if (const auto it = spec.named.find(state.name); it != spec.named.end()) {
                        state.logger->set_level(it->second);
                    } else if (spec.global.has_value()) {
                        state.logger->set_level(*spec.global);
                    }
                }

                /** Guards _states and _levels, which the log calls read. */
                mutable std::mutex _mutex;

                /** The state of every category mentioned so far, by name. */
                std::unordered_map<std::string, std::shared_ptr<CategoryState>> _states;

                /** The levels of the last SPDLOG_LEVEL specification that was read. */
                LevelSpec _levels;
            };
        } // namespace

        std::shared_ptr<CategoryState> GetOrCreateCategory(const std::string &name,
                                                           std::shared_ptr<spdlog::logger> existing,
                                                           const std::shared_ptr<spdlog::logger> &prototype) {
            return CategoryRegistry::Instance().GetOrCreate(name, std::move(existing), prototype);
        }

        std::shared_ptr<CategoryState> FindCategory(const std::string &name) {
            return CategoryRegistry::Instance().Find(name);
        }

        std::vector<std::string> CategoryNames() {
            return CategoryRegistry::Instance().Names();
        }

        void LoadEnvironmentLevels(const char *variable) {
            // spdlog applies the global level of the spec, and the per-name levels
            // of the loggers that already exist; the whole spec is kept here as well,
            // both for the categories of another registry and for the ones that are
            // created after this call. An unset or empty variable forgets the levels
            // that were remembered from an earlier spec, so that reading the
            // environment twice is idempotent.
            const auto spec = spdlog::details::os::getenv(variable);
            if (!spec.empty()) {
                spdlog::cfg::load_env_levels(variable);
            }
            CategoryRegistry::Instance().SetLevels(ParseLevelSpec(spec));
        }
    } // namespace detail
} // namespace LOG
