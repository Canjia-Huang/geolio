//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/29.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_APPLICATION_REGISTRY_H
#define GEOLIO_APPLICATION_REGISTRY_H

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "base_application.h"
#include "geolio/geobox/object/base_object.h"

namespace geolio::geobox
{
    /** Objects container shared by all sub-applications. */
    using ApplicationObjects = std::vector<std::shared_ptr<BaseObject>>;

    /**
     * Registry of plugin-style sub-applications (BaseApplication subclasses).
     * Mirrors the MeshIOHandler factory pattern from geolio/io/io.h: concrete
     * applications register a creation function at startup, and
     * GeoBoxApplication instantiates every registered application afterwards.
     * A concrete application decides its own display name (and gets a unique
     * instance id) when the class is defined, so registration only adds the
     * menu placement: where the entry goes under the "Applications" menu.
     */
    class BaseApplicationRegistry {
    public:
        /**
         * @brief Creator function type for a registered sub-application.
         * @details All registered concrete types must provide a constructor
         *          (const ApplicationObjects&), which is the current
         *          convention (see LocalOperationOptimizationApplication).
         *          The menu path is passed in, so the menu placement stays out
         *          of the concrete classes.
         */
        using Creator = std::unique_ptr<BaseApplication>(*)(
            const std::vector<std::string>& menu_path,
            const ApplicationObjects& objects
        );

        /**
         * @brief One registered sub-application: where its entry goes in the
         *        "Applications" menu, and how to create it.
         * @details Display name, instance id and tooltip belong to the
         *          application itself (see BaseApplication), so they are not
         *          duplicated here.
         */
        struct Entry {
            /** Parent sub-menu labels, outermost first; when empty, the
             *  created application defaults to the "ungrouped" sub-menu (see
             *  BaseApplication::set_menu_path()). */
            std::vector<std::string> menu_path;
            /** Creates the application and attaches @c menu_path to it. */
            Creator creator;
        };

        /**
         * @brief Helper class to register a creator at function-local static
         *        initialization, mirroring GEO::Factory::RegisterCreator.
         * @tparam ConcreteType the sub-application class to register.
         */
        template <class ConcreteType>
        struct RegisterCreator {
            /**
             * @brief Registers @p ConcreteType.
             * @param[in] menu_path labels of the parent sub-menus under the
             *            "Applications" menu, outermost first; when empty, the
             *            application lands in the "ungrouped" sub-menu.
             */
            explicit RegisterCreator(std::vector<std::string> menu_path = {}) {
                BaseApplicationRegistry::instance().register_creator(
                    std::move(menu_path),
                    &BaseApplicationRegistry::create<ConcreteType>
                );
            }
        };

        /**
         * @brief Gets the registry unique instance.
         */
        static BaseApplicationRegistry& instance() {
            static BaseApplicationRegistry registry;
            return registry;
        }

        /**
         * @brief Generic creator: instantiates @p ConcreteType with the
         *        objects container and attaches the menu path to the instance.
         * @details Menu placement is registration data, so concrete
         *          applications stay unaware of it and do not need an extra
         *          constructor parameter; the name comes from the class.
         */
        template <class ConcreteType>
        static std::unique_ptr<BaseApplication> create(
            const std::vector<std::string>& menu_path,
            const ApplicationObjects& objects
        ) {
            auto application = std::make_unique<ConcreteType>(objects);
            application->set_menu_path(menu_path);
            return application;
        }

        /**
         * @brief Registers a creator under a menu path.
         * @param[in] menu_path sub-menu chain the application lives in.
         * @param[in] creator the creation function.
         */
        void register_creator(std::vector<std::string> menu_path, Creator creator) {
            entries_.push_back({std::move(menu_path), creator});
        }

        /**
         * @brief Lists all registered entries in registration order (which is
         *        also the order of the "Applications" menu).
         */
        [[nodiscard]] const std::vector<Entry>& entries() const {
            return entries_;
        }

    private:
        std::vector<Entry> entries_;
    };
}

#define GEOLIO_CONCAT_IMPL(a, b) a##b
#define GEOLIO_CONCAT(a, b) GEOLIO_CONCAT_IMPL(a, b)

/**
 * @brief Helper macro to register a BaseApplication subclass.
 * @details Declares a function-local static RegisterCreator object that, on
 *          the first call of the enclosing function, registers how to create
 *          @p type. The display name (and the instance id) are decided by the
 *          class itself, so they are not passed here. Mirrors
 *          geo_register_MeshIOHandler_creator() from geolio/io/io.h.
 *          The optional trailing arguments are the labels of the parent
 *          sub-menus, outermost first: none means the entry is grouped under
 *          "ungrouped", while
 *          geolio_register_BaseApplication_creator(Type, "Mesh", "Remeshing")
 *          puts it in Applications > Mesh > Remeshing.
 * @param[in] type the ConcreteType subclass of BaseApplication.
 * @param[in] ... optional parent sub-menu labels (outermost first).
 */
#define geolio_register_BaseApplication_creator(type, ...) \
    static geolio::geobox::BaseApplicationRegistry::RegisterCreator<type> \
        GEOLIO_CONCAT(geolio_register_application_creator_, __LINE__)( \
            std::vector<std::string>{__VA_ARGS__})

#endif //GEOLIO_APPLICATION_REGISTRY_H
