//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/23.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_BASE_APPLICATION_H
#define GEOLIO_BASE_APPLICATION_H
#include <string>
#include <utility>
#include <vector>
#include <geogram_gfx/third_party/imgui/imgui.h>
#include <geolio/common/utils.h>

namespace geolio::geobox
{
    /**
     * @brief Base class for plugin-style sub-applications (e.g. mesh apps)
     *        embedded in the main GeoBox application.
     * @details Each sub-application contributes one menu entry
     *          (draw_menu()) and one floating window (draw_window()), whose
     *          title is application_name_ and whose visibility is shared
     *          between the menu checkbox and the window's close button.
     *          The concrete class decides its own display name (passed to the
     *          BaseApplication constructor) and gets a generated instance id,
     *          so neither has to be supplied at the registration site.
     *          The entry is placed in a (possibly empty) chain of sub-menus
     *          below the "Applications" menu: menu_path() holds the labels of
     *          those parent menus, from the root one to the closest one.
     */
    class BaseApplication {
    public:
        explicit BaseApplication(std::string application_name)
            : id_(generate_random_string(22)),
              application_name_(std::move(application_name))
        {}

        virtual ~BaseApplication() = default;

        /**
         * @brief Unique identifier of this application instance.
         * @details Generated when the instance is constructed, so neither the
         *          concrete class nor the registration site has to provide
         *          one.
         */
        [[nodiscard]] const std::string& id() const {
            return id_;
        }

        /**
         * @brief User-facing name of this application.
         * @details Used as the menu item label and as the window title.
         *          Decided by the concrete class in its constructor.
         */
        [[nodiscard]] const std::string& application_name() const {
            return application_name_;
        }

        /**
         * @brief Labels of the sub-menus this application lives in.
         * @details Empty means the entry is a direct child of the
         *          "Applications" menu. A path like {"Mesh", "Optimization"}
         *          yields Applications > Mesh > Optimization > entry.
         *          Filled in by the application factory from the registration
         *          data (see application_registry.h).
         */
        [[nodiscard]] const std::vector<std::string>& menu_path() const {
            return menu_path_;
        }

        /**
         * @brief Sets the sub-menu chain this application lives in.
         * @details Called once by the registry-based factory right after
         *          construction, so concrete applications do not need to
         *          know about menu placement.
         */
        void set_menu_path(std::vector<std::string> menu_path) {
            menu_path_ = std::move(menu_path);
        }

        /**
         * @brief Draws this application's entries in the main menu bar.
         * @details Template: a checkbox item that toggles the floating
         *          window, then hands off to draw_menu_items() so derived
         *          classes can append their own entries.
         */
        virtual void draw_menu() {
            ImGui::MenuItem(application_name_.c_str(),
                nullptr,
                &window_visible_
            );
            // Show the menu description as a tooltip when the mouse hovers
            // over this menu item.
            if (ImGui::IsItemHovered() && !menu_tooltip_.empty())
                ImGui::SetTooltip("%s", menu_tooltip_.c_str());
        }

        /**
         * @brief Draws the application's floating window.
         * @details The window is titled after application_name_ and is drawn
         *          only while window_visible_ is true. Its close button
         *          clears window_visible_ (which also unchecks the menu
         *          item); the body is provided by draw_window_contents().
         */
        virtual void draw_window() {
            if (!window_visible_)
                return;

            const ImVec2 viewport_size = ImGui::GetMainViewport()->Size;
            ImGui::SetNextWindowSize(
                ImVec2(viewport_size.x * 0.3f, viewport_size.y * 0.5f),
                ImGuiCond_FirstUseEver
            );
            // Center the window on the viewport the first time it is shown;
            // afterwards the user may drag it anywhere.
            ImGui::SetNextWindowPos(
                ImGui::GetMainViewport()->GetCenter(),
                ImGuiCond_FirstUseEver,
                ImVec2(0.5f, 0.5f)
            );
            ImGui::SetNextWindowBgAlpha(0.6f);
            if (ImGui::Begin(application_name_.c_str(), &window_visible_))
                draw_window_contents();

            ImGui::End();
        }

    protected:
        /**
         * @brief Hook: derived classes draw the window body here.
         */
        virtual void draw_window_contents() = 0;

        const std::string id_;
        const std::string application_name_;
        std::string menu_tooltip_;
        /** Parent sub-menu labels, outermost first (see menu_path()). */
        std::vector<std::string> menu_path_;
        bool window_visible_ = false;
    };
}

#endif //GEOLIO_BASE_APPLICATION_H
