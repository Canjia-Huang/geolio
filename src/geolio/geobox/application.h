//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/18.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_APPLICATION_H
#define GEOLIO_APPLICATION_H

#include <geogram_gfx/gui/simple_mesh_application.h>
#include "colormap.h"
#include "geolio/geobox/object/base_object.h"

namespace geolio::geobox
{
    /**
     * @brief GeoBox application entry point.
     */
    class GeoBoxApplication : public GEO::SimpleMeshApplication {
    public:
        /**
         * @brief Constructs the GeoBox application and initializes default state.
         */
        GeoBoxApplication();

        /**
         * @brief Draws the application GUI and all editor panels.
         */
        void draw_gui() override;

    protected:
        /**
         * @brief Populates @c my_colormaps_ after the OpenGL context is ready.
         * @details The base class initializes its own colormap table in a non-virtual method,
         *          so this helper is called explicitly to fill the GeoBox-specific map list.
         */
        void init_colormaps();

        /**
         * @brief Initializes the OpenGL-dependent application state.
         */
        void GL_initialize() override;

        /**
         * @brief Applies the UI style after ImGui has been initialized.
         * @details This mirrors the Polyscope style setup because the base class resets the
         *          GUI theme to light mode during its initialization path.
         */
        void ImGui_initialize() override;

        /**
         * @brief Draws the controller property panel in the main UI.
         */
        void draw_controller_properties_window();

        /**
         * @brief Draws editable controller parameters for the selected object or tool.
         */
        void draw_controller_properties();

        /**
         * @brief Draws the viewer-related property section.
         */
        void draw_viewer_properties() override;

        /**
         * @brief Draws the properties for all objects currently managed by the application.
         */
        void draw_objects_properties();

        /**
         * @brief Draws the object properties window container.
         */
        void draw_object_properties_window() override;

        /**
         * @brief Draws the active object's property widgets.
         */
        void draw_object_properties() override;

        /**
         * @brief Renders the main scene content.
         */
        void draw_scene() override;

        /**
         * @brief Draws the about panel and application information.
         */
        void draw_about() override;

        /**
         * @brief Draws the application window menu entries.
         */
        void draw_windows_menu() override;

        /**
         * @brief Draws the rotation gizmo in the viewport.
         * @details The gizmo is rendered in the lower-left corner and updates the current
         *          object rotation when the user drags it.
         */
        void draw_rotation_gizmo();

        /**
         * @brief Focuses the camera on a given object.
         * @param[in] object_ptr Optional object to center the view on; if null, the current
         *                      selection or default target is used.
         */
        void camera_focus(const std::shared_ptr<BaseObject>& object_ptr = nullptr);

        /**
         * @brief Loads a scene or model from the given file path.
         * @param[in] filepath Path to the file to import.
         * @return true if the file was loaded successfully; false otherwise.
         */
        bool load(const std::string& filepath) override;

        std::vector<geolio::geobox::ColormapInfo> my_colormaps_;

        // Current size of the Object Properties window, tracked so its
        // right edge can be kept flush against the viewport's right edge.
        ImVec2 object_properties_size_{0.0f, 0.0f};

        std::vector<std::shared_ptr<BaseObject>> objects_;
        std::weak_ptr<BaseObject> selected_object_;
    };
}

#endif //GEOLIO_APPLICATION_H
