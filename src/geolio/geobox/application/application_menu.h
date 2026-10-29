//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/29.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_APPLICATION_MENU_H
#define GEOLIO_APPLICATION_MENU_H

#include <string>
#include <utility>
#include <vector>

#include "base_application.h"

namespace geolio::geobox
{
    /**
     * @brief One node of the "Applications" menu tree.
     * @details A node is either a sub-menu (application == nullptr, label is
     *          the sub-menu title) or a leaf (application != nullptr, drawn by
     *          BaseApplication::draw_menu()). The tree only contains the
     *          sub-menus that actually hold an application, so no empty
     *          sub-menu can ever appear in the menu bar.
     */
    struct ApplicationMenuNode {
        /** Sub-menu title; unused (empty) for leaf nodes. */
        std::string label;
        /** Entries of a sub-menu; empty for leaf nodes. */
        std::vector<ApplicationMenuNode> children;
        /** The application of a leaf node; nullptr for sub-menu nodes. */
        BaseApplication* application = nullptr;

        /** @brief Tells whether this node is a sub-menu (not a leaf). */
        [[nodiscard]] bool is_submenu() const {
            return application == nullptr;
        }
    };

    /** @brief Root level of the "Applications" menu (its direct children). */
    using ApplicationMenu = std::vector<ApplicationMenuNode>;

    /**
     * @brief Finds the sub-menu labelled @p label, appending it if missing.
     * @details Sub-menus are matched by their title, so applications sharing a
     *          menu path automatically end up in the same sub-menu, whatever
     *          the registration order is.
     */
    inline ApplicationMenuNode& find_or_add_submenu(
        ApplicationMenu& menu, const std::string& label
    ) {
        for (auto& node : menu) {
            if (node.is_submenu() && node.label == label)
                return node;
        }
        menu.push_back(ApplicationMenuNode{label, {}, nullptr});
        return menu.back();
    }

    /**
     * @brief Inserts @p app in the menu tree, at application.menu_path().
     * @details Empty path components are ignored. BaseApplication already
     *          guarantees a non-empty path (applications that never set one
     *          are grouped under UNGROUPED_APPLICATIONS_MENU_NAME), so every
     *          leaf ends up inside at least one sub-menu. The insertion order
     *          is the order the applications are registered in.
     */
    inline void insert_application_menu_entry(
        ApplicationMenu& menu, BaseApplication& application
    ) {
        ApplicationMenu* level = &menu;
        for (const auto& label : application.menu_path()) {
            if (label.empty())
                continue;
            level = &find_or_add_submenu(*level, label).children;
        }
        level->push_back(ApplicationMenuNode{"", {}, &application});
    }

    /**
     * @brief Draws one level of the "Applications" menu tree.
     * @details Sub-menus are opened with ImGui::BeginMenu() and drawn
     *          recursively; leaves delegate to BaseApplication::draw_menu().
     *          This function knows nothing about concrete applications.
     */
    inline void draw_application_menu(const ApplicationMenu& menu) {
        for (const auto& node : menu) {
            if (node.is_submenu()) {
                if (ImGui::BeginMenu(node.label.c_str())) {
                    draw_application_menu(node.children);
                    ImGui::EndMenu();
                }
            } else if (node.application != nullptr) {
                node.application->draw_menu();
            }
        }
    }
}

#endif //GEOLIO_APPLICATION_MENU_H
