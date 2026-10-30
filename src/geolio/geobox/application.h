//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/18.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_APPLICATION_H
#define GEOLIO_APPLICATION_H

#include <geogram_gfx/gui/simple_mesh_application.h>
#include "application/base_application.h"
#include "application/application_menu.h"
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

        /**
         * @brief Called on every input event; replaces the base class's
         *        100-frame redraw storm with a small per-event frame budget.
         */
        void update() override;

        /**
         * @brief Decides whether a frame must be drawn.
         * @details Returns true when the scene is dirty, when animation is
         *          running, when ImGui is capturing mouse/keyboard (hover,
         *          clicks, popups, slider drags need frames to work), or when
         *          a few budget frames remain after the latest input event.
         */
        bool needs_to_redraw() const override;

        /**
         * @brief Renders the 3D scene only when it changed.
         * @details The scene is rendered into an offscreen framebuffer, which
         *          is then blitted to the back buffer every frame. When the
         *          scene is unchanged the render pass is skipped and only the
         *          blit runs: it overwrites the whole back buffer (wiping any
         *          previous frame's UI), so ImGui is drawn on a clean scene
         *          image without ghosting.
         */
        void draw_graphics() override;

    protected:
        /**
         * @brief Initializes the OpenGL-dependent application state.
         */
        void GL_initialize() override;

        /**
         * @brief Releases the OpenGL-dependent application state (offscreen
         *        scene framebuffer, colormap table, image texture cache).
         */
        void GL_terminate() override;

        /**
         * @brief Marks the scene dirty on key events (base class shortcuts
         *        may toggle render state).
         */
        void key_callback(int key, int scancode, int action, int mods) override;

        /**
         * @brief Marks the scene dirty while the user drags in the viewport
         *        (rotate / translate / zoom).
         */
        void cursor_pos_callback(double x, double y, int source) override;

        /**
         * @brief Marks the scene dirty on scroll (zoom changes).
         */
        void scroll_callback(double xoffset, double yoffset) override;

        /**
         * @brief Marks the scene dirty after a resize (the retained back
         *        buffer content is invalidated by the framebuffer resize).
         */
        void resize(GEO::index_t w, GEO::index_t h, GEO::index_t fb_w, GEO::index_t fb_h) override;

        // == Offscreen scene framebuffer (redraw-on-demand) ========================
        // The 3D scene is rendered into this framebuffer and blitted to the
        // back buffer every frame, so UI-only frames can skip the scene render
        // while keeping the previous scene image and a clean UI (no ghosting).

        /**
         * @brief (Re)creates the offscreen scene framebuffer.
         * @param[in] w Framebuffer width in pixels.
         * @param[in] h Framebuffer height in pixels.
         */
        void create_scene_framebuffer(GLsizei w, GLsizei h);

        /**
         * @brief Deletes the offscreen scene framebuffer, if any.
         */
        void delete_scene_framebuffer();

        /**
         * @brief Copies the offscreen scene framebuffer into the default
         *        back buffer, overwriting the previous frame's UI.
         */
        void blit_scene_framebuffer() const;

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

        void draw_menu_bar() override;

        /**
         * @brief Draws the about panel and application information.
         */
        void draw_about() override;

        /**
         * @brief Draws the application window menu entries.
         */
        void draw_windows_menu() override;

        void draw_application_menus() override;

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

        /** True when the rendered scene (camera, mesh state, ...) changed and
         *  the 3D view must be re-rendered. Cleared after each full render. */
        bool scene_dirty_ = true;

        /** Whether the redraw-on-demand scheme is active (default on).
         *  When false, the original event-driven scheme is used: every redraw
         *  re-renders the whole scene and every event triggers the base
         *  class's 100-frame redraw storm. Togglable from the Windows menu. */
        bool redraw_on_demand_ = false;

        // Offscreen scene framebuffer (see create_scene_framebuffer()).
        GLuint scene_fbo_ = 0;
        GLuint scene_color_tex_ = 0;
        GLuint scene_depth_rb_ = 0;
        GLsizei scene_fb_w_ = 0;
        GLsizei scene_fb_h_ = 0;

        /** Frames to draw after the most recent input event (replaces the
         *  base class's 100-frame redraw storm). */
        mutable GEO::index_t redraw_budget_ = 0;

        bool controller_properties_visible_ = true;
        bool arc_ball_visible_ = true;

        std::vector<std::shared_ptr<BaseObject>> objects_;
        std::weak_ptr<BaseObject> selected_object_;

        // Current size of the Object Properties window, tracked so its
        // right edge can be kept flush against the viewport's right edge.
        ImVec2 object_properties_size_{0.0f, 0.0f};

        std::vector<std::unique_ptr<BaseApplication>> applications_;

        /** "Applications" menu tree, built once from each application's own
         *  application_name() / menu_path(), i.e. from the registration data
         *  in application/register_applications.h. Keeping it as a tree is
         *  what lets draw_application_menus() stay generic: adding a
         *  sub-application or a sub-menu never touches that function. */
        ApplicationMenu application_menu_;
    };
}

#endif //GEOLIO_APPLICATION_H
