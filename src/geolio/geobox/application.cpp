//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/18.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
// imoguizmo relies on the ImVec2 courtesy math operators, which geogram's
// imgui only provides when this macro is set. It must be defined before
// imgui.h is first included in this translation unit.
#define IMGUI_DEFINE_MATH_OPERATORS
#include "application.h"
#include <algorithm>
#include <cstring>
#include <unordered_map>
#include <vector>
#include <geogram/basic/command_line.h>
#include <geogram/basic/file_system.h>
#include <geogram/mesh/mesh_io.h>
#include <geogram_gfx/GLUP/GLUP.h>
#include <geogram_gfx/third_party/imgui/imgui_internal.h>
#include "imoguizmo.hpp"
#include "geolio/common/log.h"
#include "geolio/common/parse_filepath.h"
#include "geolio/io/fra_io.h"
#include "file_dialog.h"
#include "object/mesh_object.h"
#include "colormap.h"
#include <geolio/geobox/application/register_applications.h>
#include <geolio/geobox/light_shader.h>
#include <geolio/geobox/scene_light.h>
#include <geogram_gfx/full_screen_effects/ambient_occlusion.h>
#include <geogram_gfx/full_screen_effects/unsharp_masking.h>

namespace geolio::geobox
{
    namespace
    {
        /** @brief Whether @p window is one of GeoGram's file dialogs. */
        bool is_file_dialog(const ImGuiWindow* window) {
            if (window == nullptr || window->Name == nullptr)
                return false;
            // The directory and file lists of the dialog are child windows
            // named "<dialog name>/...": they are skipped below.
            return std::strncmp(window->Name, "Load...##", 9) == 0 ||
                   std::strncmp(window->Name, "Save as...##", 12) == 0;
        }

        /**
         * @brief Sizes the file dialogs drawn during the current frame.
         * @details GeoGram asks for a 400x415 window for them, times its layout
         *          scaling (see draw_file_dialog()). They are given a size that
         *          is comfortable for their two file lists, and the same on every
         *          display -- shrunk only when the window of the application
         *          cannot hold it. A dialog is sized during the first frames it
         *          is shown, and not afterwards: GeoGram applies its own size on
         *          the first one, and a size the user chose must be kept.
         */
        void resize_file_dialogs() {
            // Frames a dialog is still being sized for, per dialog.
            static std::unordered_map<ImGuiID, int> frames_left;
            constexpr int FRAMES_TO_SIZE = 2;

            constexpr float DIALOG_WIDTH = 640.0f;
            constexpr float DIALOG_HEIGHT = 600.0f;
            constexpr float DISPLAY_MARGIN = 80.0f;

            ImGuiContext& context = *ImGui::GetCurrentContext();
            const ImVec2 display = context.IO.DisplaySize;
            const ImVec2 wanted(
                std::min(DIALOG_WIDTH, display.x - DISPLAY_MARGIN),
                std::min(DIALOG_HEIGHT, display.y - DISPLAY_MARGIN));

            for (ImGuiWindow* window : context.Windows) {
                if (window == nullptr ||
                    window->LastFrameActive != context.FrameCount ||
                    (window->Flags & ImGuiWindowFlags_ChildWindow) ||
                    !is_file_dialog(window)) {
                    continue;
                }
                auto it = frames_left.find(window->ID);
                if (it == frames_left.end())
                    it = frames_left.emplace(window->ID, FRAMES_TO_SIZE).first;
                if (it->second <= 0)
                    continue;
                --it->second;
                ImGui::SetWindowSize(window, wanted, ImGuiCond_Always);
            }
        }
    }

    GeoBoxApplication::GeoBoxApplication(
        ) : SimpleMeshApplication("Geolio - GeoBox")
    {
        /* Register and instantiate every sub-application: instead of
         * hardcoding each concrete application here, the registry is
         * populated by register_additional_Applications() (see
         * application/register_applications.h, mirroring how additional
         * MeshIO handlers are registered in geolio/io/io.h), and each
         * registered creator is invoked with this application's objects
         * container. Registration data also carries the sub-menu the
         * application belongs to, so the menu tree is built here from the
         * instances themselves: no concrete type is known at this point. */
        register_additional_Applications();
        for (const auto& entry : BaseApplicationRegistry::instance().entries()) {
            auto application = entry.creator(entry.menu_path, objects_);
            if (!application)
                continue;
            insert_application_menu_entry(application_menu_, *application);
            applications_.push_back(std::move(application));
        }
    }

    void GeoBoxApplication::update(
        ) {
        if (!redraw_on_demand_) {
            // Original scheme: restore the base class behavior (a 100-frame
            // redraw storm after every event).
            Application::update();
            return;
        }
        // The base class resets a 100-frame redraw counter on every event
        // (NB_FRAMES_UPDATE_INIT in Application::update()), which re-renders
        // the (possibly huge) scene up to 100 times per event. A couple of
        // frames is enough for ImGui to process the event; everything else is
        // driven by scene_dirty_ / ImGui capture below.
        redraw_budget_ = 2;
    }

    bool GeoBoxApplication::needs_to_redraw(
        ) const {
        if (!redraw_on_demand_)
            return Application::needs_to_redraw();

        if (animate())
            return true;

        if (scene_dirty_)
            return true;

        // ImGui needs continuous frames to process hover, clicks, popups and
        // slider drags, so keep rendering while it captures the mouse/keyboard.
        if (ImGui::GetIO().WantCaptureMouse ||
            ImGui::GetIO().WantCaptureKeyboard)
            return true;

        if (redraw_budget_ > 0) {
            --redraw_budget_;
            return true;
        }

        return false;
    }

    void GeoBoxApplication::create_scene_framebuffer(
        const GLsizei w, const GLsizei h
        ) {
        delete_scene_framebuffer();
        scene_fb_w_ = w;
        scene_fb_h_ = h;

        glGenFramebuffers(1, &scene_fbo_);
        glBindFramebuffer(GL_FRAMEBUFFER, scene_fbo_);

        glGenTextures(1, &scene_color_tex_);
        glBindTexture(GL_TEXTURE_2D, scene_color_tex_);
        glTexImage2D(
            GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0,
            GL_RGBA, GL_UNSIGNED_BYTE, nullptr
        );
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glFramebufferTexture2D(
            GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
            GL_TEXTURE_2D, scene_color_tex_, 0
        );

        glGenRenderbuffers(1, &scene_depth_rb_);
        glBindRenderbuffer(GL_RENDERBUFFER, scene_depth_rb_);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, w, h);
        glFramebufferRenderbuffer(
            GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
            GL_RENDERBUFFER, scene_depth_rb_
        );

        const bool complete =
            (glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE);

        // Restore default bindings.
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glBindRenderbuffer(GL_RENDERBUFFER, 0);
        glBindTexture(GL_TEXTURE_2D, 0);

        if (!complete) {
            // Fall back to direct rendering if offscreen rendering fails.
            delete_scene_framebuffer();
        }
    }

    void GeoBoxApplication::delete_scene_framebuffer(
        ) {
        if (scene_fbo_ != 0)
            glDeleteFramebuffers(1, &scene_fbo_);
        if (scene_color_tex_ != 0)
            glDeleteTextures(1, &scene_color_tex_);
        if (scene_depth_rb_ != 0)
            glDeleteRenderbuffers(1, &scene_depth_rb_);
        scene_fbo_ = 0;
        scene_color_tex_ = 0;
        scene_depth_rb_ = 0;
        scene_fb_w_ = 0;
        scene_fb_h_ = 0;
    }

    void GeoBoxApplication::blit_scene_framebuffer(
        ) const {
        glBindFramebuffer(GL_READ_FRAMEBUFFER, scene_fbo_);
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
        glBlitFramebuffer(
            0, 0, scene_fb_w_, scene_fb_h_,
            0, 0, scene_fb_w_, scene_fb_h_,
            GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT,
            GL_NEAREST
        );
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    void GeoBoxApplication::draw_graphics(
        ) {
        if (!redraw_on_demand_) {
            // Original scheme: always render the full scene directly.
            SimpleApplication::draw_graphics();
            scene_dirty_ = false;
            return;
        }

        const GLsizei fb_w = static_cast<GLsizei>(get_frame_buffer_width());
        const GLsizei fb_h = static_cast<GLsizei>(get_frame_buffer_height());
        if (scene_fbo_ == 0 || scene_fb_w_ != fb_w || scene_fb_h_ != fb_h)
            create_scene_framebuffer(fb_w, fb_h);

        if (scene_fbo_ == 0) {
            // Offscreen rendering unavailable: fall back to rendering the
            // scene directly into the back buffer on every frame.
            SimpleApplication::draw_graphics();
            scene_dirty_ = false;
            return;
        }

        if (scene_dirty_ || animate()) {
            // Render the 3D scene into the offscreen framebuffer.
            glBindFramebuffer(GL_FRAMEBUFFER, scene_fbo_);
            SimpleApplication::draw_graphics();
            glBindFramebuffer(GL_FRAMEBUFFER, 0);
            scene_dirty_ = false;
        }
        // Copy the scene image into the back buffer. The blit overwrites the
        // whole buffer, wiping the previous frame's UI, so ImGui (drawn next
        // by the base one_frame()) starts from a clean scene - no ghosting.
        blit_scene_framebuffer();
    }

    void GeoBoxApplication::GL_terminate(
        ) {
        // Release the colormap textures GeoBox owns. They are the property of
        // the process-wide colormap library rather than of this application,
        // because the objects that draw with them do not know the application
        // (see ColormapLibrary).
        ColormapLibrary::instance().terminate();
        // The images displayed by the TEXTURE painting mode are cached by
        // file name and shared by the objects (see MeshObject::texture()).
        release_texture_cache();
        // The program the surfaces are shaded with belongs to the process (see
        // light_shader()): it is released here, while the context is current.
        light_shader().release();
        delete_scene_framebuffer();
        SimpleMeshApplication::GL_terminate();
    }

    void GeoBoxApplication::cursor_pos_callback(
        const double x, const double y, const int source
        ) {
        SimpleApplication::cursor_pos_callback(x, y, source);
        // Only viewport drags (rotate / translate / zoom) change the scene.
        if (mouse_op_ != MOUSE_NOOP)
            scene_dirty_ = true;
    }

    void GeoBoxApplication::scroll_callback(
        const double xoffset, const double yoffset
        ) {
        SimpleApplication::scroll_callback(xoffset, yoffset);
        scene_dirty_ = true;
    }

    void GeoBoxApplication::key_callback(
        const int key, const int scancode, const int action, const int mods
        ) {
        SimpleApplication::key_callback(key, scancode, action, mods);
        // Base class shortcuts (F-keys, ...) may toggle render state.
        scene_dirty_ = true;
    }

    void GeoBoxApplication::resize(
        const GEO::index_t w, const GEO::index_t h, const GEO::index_t fb_w, const GEO::index_t fb_h
        ) {
        SimpleApplication::resize(w, h, fb_w, fb_h);
        // A resized framebuffer invalidates the retained back buffer content,
        // so the next frame must be a full re-render.
        scene_dirty_ = true;
    }

    void GeoBoxApplication::draw_gui(
        ) {
        // A recreated ImGui context starts on the defaults of the panels: the
        // geometry is given back before they are drawn, the open/closed state of
        // their headers at the end of the frame (see restore_ui_state()).
        if (ui_state_to_restore_)
            restore_ui_state();

        draw_menu_bar();
        // Draw each sub-application's floating window (visibility is toggled
        // from the corresponding menu entry in the menu bar).
        for (const auto& app_ptr : applications_)
            app_ptr->draw_window();

        draw_controller_properties_window();
        // draw_viewer_properties_window();
        draw_object_properties_window();

        // Built-in ImGui demo window (toggled from the Windows menu). It
        // passes its own visibility flag so its close button hides it again.
        if (imgui_demo_visible_)
            ImGui::ShowDemoWindow(&imgui_demo_visible_);
        // draw_console();
        // draw_command_window();
        // draw_command_line_editor();

        if (text_editor_visible_)
            text_editor_.draw();
        if (draw_file_dialog(
                "##load_dlg", filename_, GEO::geo_imgui_string_length))
            load(filename_);

        // The "Save as..." of an object (see draw_objects_properties()): the
        // dialog was opened on the object whose row was right-clicked.
        if (draw_file_dialog(
                "##object_save_dlg", filename_, GEO::geo_imgui_string_length)) {
            if (const auto object = object_to_save_.lock()) {
                if (object->save(filename_))
                    LOG::INFO("Saved `{}` to {}", object->name(), filename_);
                else
                    LOG::ERROR(
                        "Could not save `{}` to {}", object->name(), filename_);
            }
            object_to_save_.reset();
        }

        if (status_bar_->active()) {
            const auto w = static_cast<float>(get_frame_buffer_width());
            const auto h = static_cast<float>(get_frame_buffer_height());
            float STATUS_HEIGHT = status_bar_->get_window_height();
            if(STATUS_HEIGHT == 0.0f)
                STATUS_HEIGHT = static_cast<float>(get_font_size());

            STATUS_HEIGHT *= 1.5f;
            ImGui::SetNextWindowPos(
                ImVec2(0.0f, h-STATUS_HEIGHT),
                ImGuiCond_Always
            );
            ImGui::SetNextWindowSize(
                ImVec2(w,STATUS_HEIGHT-1.0f),
                ImGuiCond_Always
            );
            status_bar_->draw();
        }

        draw_rotation_gizmo();

        // Objects whose data cannot be rendered warn about it here, on top of
        // the rest of the UI (they are modal, so they cannot be skipped).
        for (const auto& object : objects_)
            object->draw_validity_dialog();

        // All the file dialogs of the frame (Load/Save menu entries, texture
        // image pickers, ...) have been drawn at this point: give the ones that
        // just appeared the size the other dialogs have (see
        // resize_file_dialogs()).
        resize_file_dialogs();

        // The panels have been drawn: the windows exist, so the open/closed state
        // of their headers can be given back to them (see restore_ui_state()).
        if (ui_state_to_restore_) {
            for (ImGuiWindow* window : ImGui::GetCurrentContext()->Windows) {
                if (window->Name == nullptr)
                    continue;
                const auto state = window_states_.find(window->Name);
                if (state != window_states_.end())
                    window->StateStorage = state->second;
            }
            window_states_.clear();
            ui_settings_.clear();
            ui_state_to_restore_ = false;
        }

        // Redraw-on-demand: any real widget interaction (click, slider drag,
        // text edit) may have changed the render state, so schedule a full
        // re-render for the next frame. Pure hover / mouse movement over the
        // UI does not dirty the scene and only costs cheap UI-only frames.
        if (ImGui::IsAnyItemActive() ||
            (ImGui::GetIO().WantCaptureMouse &&
             (ImGui::IsMouseClicked(ImGuiMouseButton_Left) ||
              ImGui::IsMouseReleased(ImGuiMouseButton_Left)))) {
            scene_dirty_ = true;
        }
    }

    void GeoBoxApplication::GL_initialize(
        ) {
        SimpleMeshApplication::GL_initialize();

        // Build the colormap table GeoBox offers. It belongs to the
        // process-wide library rather than to the application, so that the
        // objects that draw with it -- which know nothing of the application,
        // and can be created by any sub-application -- can read it too.
        // Uploading the textures needs the GL context, which is why it is done
        // here, right after the base class set the GL state up (and built its
        // own, larger colormap table).
        ColormapLibrary::instance().initialize();
    }

    void GeoBoxApplication::ImGui_terminate(
        ) {
        save_ui_state();
        SimpleMeshApplication::ImGui_terminate();
    }

    void GeoBoxApplication::save_ui_state(
        ) {
        // The geometry of the windows, in the format ImGui saves its settings in
        // (positions, sizes, the collapsed state of whole windows).
        size_t settings_size = 0;
        const char* settings = ImGui::SaveIniSettingsToMemory(&settings_size);
        if (settings != nullptr)
            ui_settings_.assign(settings, settings_size);

        // The open/closed state of the collapsing headers of each window, which
        // the settings do not hold: it is kept in the state storage of the window.
        window_states_.clear();
        for (const ImGuiWindow* window :
             ImGui::GetCurrentContext()->Windows) {
            if (window->Name != nullptr)
                window_states_[window->Name] = window->StateStorage;
        }

        ui_state_to_restore_ = true;
    }

    void GeoBoxApplication::restore_ui_state(
        ) {
        // The windows created this frame read their geometry from the settings,
        // which therefore have to be there before they are drawn.
        if (!ui_settings_.empty()) {
            ImGui::LoadIniSettingsFromMemory(
                ui_settings_.data(), ui_settings_.size());
        }
        ui_settings_.clear();
    }

    void GeoBoxApplication::ImGui_initialize(
        ) {
        SimpleMeshApplication::ImGui_initialize();
        set_style("Light");

        // Axis length relative to the gizmo size; with the projection
        // normalized to m11 = 1 in draw_rotation_gizmo(), the axes extend
        // exactly axisLengthScale * size from the center.
        ImOGuizmo::config.axisLengthScale = 0.4f;

        // set_background_color(GEO::vec4f(0.08f, 0.12f, 0.22f, 1.0f)); // dark blue
    }

    void GeoBoxApplication::draw_controller_properties_window(
        ) {
        if (!controller_properties_visible_)
            return;

        const ImVec2 viewport_size = ImGui::GetMainViewport()->Size;
        ImGui::SetNextWindowPos(
            ImVec2(0.0f, ImGui::GetFrameHeight()), ImGuiCond_FirstUseEver
        );
        ImGui::SetNextWindowSize(
            ImVec2(viewport_size.x * 0.25f, viewport_size.y * 0.5f),
            ImGuiCond_FirstUseEver
        );
        ImGui::SetNextWindowBgAlpha(0.6f);
        if (ImGui::Begin("Controller", nullptr, ImGuiWindowFlags_NoDocking))
            draw_controller_properties();

        ImGui::End();
    }

    void GeoBoxApplication::draw_controller_properties(
        ) {
        if (ImGui::CollapsingHeader("Viewer"))
            draw_viewer_properties();
        if (ImGui::CollapsingHeader("Object", ImGuiTreeNodeFlags_DefaultOpen))
            draw_objects_properties();
    }

    bool GeoBoxApplication::open_group(
        const char* name,
        const bool default_open
        ) {
        // A group is a tree node, not a header: the headers of the panels are
        // framed rows that span the panel (see draw_viewer_properties()), and a
        // group has to be told apart from them -- it is an arrow and a name, and
        // ImGui indents what it holds.
        // The id of the node is kept apart from the ids of the widgets that carry
        // the name of the group -- the switch of the clipping carries it (see
        // draw_viewer_properties()) -- with a part that is not drawn, so that
        // clicking one does not act on the other.
        const std::string label = std::string(name) + "##group";
        ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_SpanAvailWidth;
        if (default_open)
            flags |= ImGuiTreeNodeFlags_DefaultOpen;
        return ImGui::TreeNodeEx(label.c_str(), flags);
    }

    void GeoBoxApplication::draw_viewer_properties(
        ) {
        // The camera button brings the scene back in view. It stays at the top of
        // the panel, out of the groups.
        if (ImGui::Button((GEO::icon_UTF8("home")).c_str(), ImVec2(-1.0, 0.0)))
            home();
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Frame the scene again.");

        // What is on: whether the scene is lit, whether it is clipped, and the
        // color it is drawn on, on one line and out of the groups -- the groups of
        // the light and of the clipping are only there when they are on.
        if (three_D_) {
            ImGui::Checkbox("Lighting", &lighting_);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Shade the scene with a light.");
            ImGui::SameLine();
            ImGui::Checkbox("Clipping", &clip_plane_.active);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(
                    "Cut the scene with a plane: the group of its settings is "
                    "below.");
            ImGui::SameLine();
        }
        ImGui::ColorEdit3WithPalette("Backgnd", background_color_.data());
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Color the scene is drawn on.");

        // The groups of settings, closed: the panel then shows what can be set
        // without taking the room of the settings themselves.
        if (three_D_ && lighting_ && open_group("Light")) {
            draw_light_properties();
            ImGui::TreePop();
        }
        if (three_D_ && clip_plane_.active && open_group("Clipping")) {
            draw_clipping_group();
            ImGui::TreePop();
        }
        if (open_group("Shading")) {
            draw_shading_group();
            ImGui::TreePop();
        }
    }


    void GeoBoxApplication::draw_clipping_group(
        ) {
        // The clipping plane of the viewer, which is on: whether it follows the
        // camera or stays fixed to the scene (see ClipAnchor), the direction it
        // faces and where it stands along it, the side of it that is kept, and the
        // GLUP mode it discards the elements with (see ClipPlane). Its switch is
        // with the ones of the panel, above (see draw_viewer_properties()).
        const auto combo_width = [](const char* label) {
            return -(ImGui::CalcTextSize(label).x +
                     ImGui::GetStyle().ItemInnerSpacing.x);
        };
        // The widgets of the group that carry a value and the name of it after them
        // -- the shift and the mode -- share this width, the one the longest name
        // leaves room for.
        const float value_width = combo_width("shift");

        // Whether the plane follows the camera, i.e. whether the scene turns under
        // it (as with GeoGram's clipping and Graphite's camera clipping), or stays
        // fixed to the scene and cuts the same place whatever the view is.
        bool follows_camera = (clip_plane_.anchor == ClipAnchor::VIEW);
        if (ImGui::Checkbox("follow cam", &follows_camera)) {
            clip_plane_.anchor = follows_camera ?
                ClipAnchor::VIEW : ClipAnchor::SCENE;
            if (follows_camera) {
                // The plane starts facing the camera, through the middle of the
                // scene, and is kept there while it follows it (see apply_clip()).
                clip_plane_.normal = camera_direction();
                clip_plane_.shift = 0.0f;
            }
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(
                "The plane follows the camera, facing it: the scene turns under "
                "it, as it does with the clipping of GeoGram and that of "
                "Graphite's camera. Left off, the plane keeps its direction in the "
                "axes of the scene and cuts the same place whatever the view is.");

        ImGui::SameLine();

        ImGui::Checkbox("invert", &clip_plane_.invert);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Keep the other side of the plane.");

        // The direction the plane faces, as the three components of a vector: any
        // direction, not just the axes of the scene. While the plane follows the
        // camera its direction is the one of the camera, which the panel shows but
        // does not let one set.
        float normal[3] = {
            float(clip_plane_.normal[0]),
            float(clip_plane_.normal[1]),
            float(clip_plane_.normal[2])
        };
        // The three components of the normal, each followed by the button that
        // points the plane along its axis of the scene: they share the width of the
        // panel, the buttons taken out of it.
        static const char* AXIS_BUTTON_NAMES[] = {"x", "y", "z"};
        static const char* AXIS_FIELD_NAMES[] = {"##x", "##y", "##z"};
        const ImGuiStyle& style = ImGui::GetStyle();
        const float button_width =
            ImGui::CalcTextSize("x").x + 2.0f * style.FramePadding.x;
        const float field_width = (
            ImGui::GetContentRegionAvail().x -
            3.0f * button_width -
            5.0f * style.ItemSpacing.x
        ) / 3.0f;

        ImGui::BeginDisabled(follows_camera);
        bool normal_changed = false;
        ImGui::PushItemWidth(field_width);
        for (int i = 0; i < IM_ARRAYSIZE(AXIS_FIELD_NAMES); ++i) {
            if (i != 0)
                ImGui::SameLine();
            normal_changed = ImGui::DragFloat(
                AXIS_FIELD_NAMES[i], &normal[i], 0.01f) || normal_changed;
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(
                    "Direction the plane faces, in the axes of the scene: the "
                    "plane is perpendicular to it, and only the direction counts, "
                    "not the length. A plane that follows the camera keeps the "
                    "direction of the camera: untick it to set another one.");
            ImGui::SameLine();
            if (ImGui::Button(AXIS_BUTTON_NAMES[i])) {
                normal[0] = (i == 0) ? 1.0f : 0.0f;
                normal[1] = (i == 1) ? 1.0f : 0.0f;
                normal[2] = (i == 2) ? 1.0f : 0.0f;
                normal_changed = true;
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(
                    "Face the plane along the %s axis of the scene.",
                    AXIS_BUTTON_NAMES[i]);
        }
        ImGui::PopItemWidth();
        ImGui::EndDisabled();

        if (normal_changed)
            clip_plane_.normal = GEO::vec3(normal[0], normal[1], normal[2]);

        // Where the plane stands along its normal, in the units of the scene and
        // without a bound: the panel cannot guess the extent of the scene, and the
        // value can be typed (ctrl-click) when dragging it is too slow.
        ImGui::PushItemWidth(value_width);
        ImGui::DragFloat("shift", &clip_plane_.shift, 0.01f, 0.0f, 0.0f, "%.3f");
        ImGui::PopItemWidth();
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(
                "Where the plane stands along its normal, in the units of the "
                "scene: the side the normal points to is kept.");

        // The GLUP mode the elements are discarded with.
        static const char* CLIP_MODE_NAMES[] = {
            "standard", "whole_cells", "straddling_cells", "slice_cells"
        };
        int mode = int(clip_plane_.mode) - int(GLUP_CLIP_STANDARD);
        ImGui::PushItemWidth(value_width);
        if (ImGui::Combo(
                "mode", &mode, CLIP_MODE_NAMES, IM_ARRAYSIZE(CLIP_MODE_NAMES))) {
            clip_plane_.mode = GLUPclipMode(int(GLUP_CLIP_STANDARD) + mode);
        }
        ImGui::PopItemWidth();
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(
                "How the clipping discards the elements: STANDARD cuts them, "
                "WHOLE_CELLS and STRADDLING_CELLS keep or drop whole cells, "
                "SLICE_CELLS cuts them open.");
    }





    void GeoBoxApplication::draw_shading_group(
        ) {
        // How the scene is shaded: the full screen effect, which shades the whole
        // image, then the shading controls the selected object adds there (see
        // BaseObject::draw_scene_properties()).

        // The width of the combos of the group, which the object draws its own with
        // (see shading_combo_width()).
        ImGui::PushItemWidth(shading_combo_width());
        if (ImGui::Combo(
                "sfx", reinterpret_cast<int*>(&effect_),
                "none\0SSAO\0cartoon\0\0")) {
            switch(effect_) {
                case 0: full_screen_effect_.reset(); break;
                case 1: full_screen_effect_ = new GEO::AmbientOcclusionImpl(); break;
                case 2: full_screen_effect_ = new GEO::UnsharpMaskingImpl(); break;
                default: assert(0);
            }
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(
                "Full screen effect applied to the rendered scene: SSAO shades "
                "the creases, cartoon flattens the colors and outlines them.");
        ImGui::PopItemWidth();

        if (const auto object = selected_object_.lock())
            object->draw_scene_properties();

    }

    GEO::vec3 GeoBoxApplication::camera_direction(
        ) const {
        // The rotation of the scene maps the scene to the view, whose camera looks
        // along -Z: the direction it looks along, in the axes of the scene, is the
        // third row of that rotation, negated (the matrix is stored column major).
        GLdouble m[16];
        glupPushMatrix();
        glupLoadIdentity();
        glupMultMatrix(object_rotation_.get_value());
        glupGetMatrixdv(GLUP_MODELVIEW_MATRIX, m);
        glupPopMatrix();

        GEO::vec3 direction(-m[2], -m[6], -m[10]);
        const double length = std::sqrt(
            direction[0] * direction[0] +
            direction[1] * direction[1] +
            direction[2] * direction[2]
        );
        if (length > 0.0)
            direction = (1.0 / length) * direction;
        return direction;
    }

    void GeoBoxApplication::apply_clip(
        ) {
        // A plane that follows the camera keeps its normal on the direction the
        // camera looks along: it is what makes the scene turn under it, and it is
        // what the panel shows of the plane while the camera turns.
        if (clip_plane_.anchor == ClipAnchor::VIEW)
            clip_plane_.normal = camera_direction();

        if (!clip_plane_.active) {
            // A zeroed plane rather than only the toggle turned off: some shaders
            // test the plane without testing the toggle, as Graphite notes.
            const double equation[4] = {0.0, 0.0, 0.0, 0.0};
            glupClipPlane(equation);
            glupDisable(GLUP_CLIPPING);
            return;
        }

        // The plane is offset by a small part of the scene along its normal, so
        // that the vertices that lie on it do not fall on one side or the other
        // depending on the rounding of the frame: GLUP classifies the vertices of
        // the cells on the CPU, in single precision, against the plane of the
        // frame (see clip_plane_equation()), and a cell of a regular grid would
        // blink as soon as one of its vertices sits on the plane.
        const double bias = 1e-4 * roi_radius_;

        const GEO::vec3 center(
            0.5 * (roi_.xyz_min[0] + roi_.xyz_max[0]),
            0.5 * (roi_.xyz_min[1] + roi_.xyz_max[1]),
            0.5 * (roi_.xyz_min[2] + roi_.xyz_max[2])
        );
        const GEO::vec4 equation =
            clip_plane_equation(clip_plane_, center, bias);

        double coefficients[4] = {
            equation[0], equation[1], equation[2], equation[3]
        };
        glupEnable(GLUP_CLIPPING);
        glupClipPlane(coefficients);
        glupClipMode(clip_plane_.mode);
    }




    void GeoBoxApplication::draw_light_properties(
        ) {
        // The light of the scene: the direction it comes from (which is the one
        // GLUP is given for the frame), the parameters GeoBox shades with (see
        // LightShader), and the specular factor, which the objects give to the
        // shading (see BaseObject::draw_lighting_properties()).
        ImGui::Checkbox("Edit light", &edit_light_);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(
                "Rotate the light by dragging in the viewport (instead of "
                "rotating the scene).");

        if (edit_light_) {
            // While the light is aimed with the mouse, the angles below are just
            // what the drag moves: they are left out.
            ImGui::TextDisabled("drag in the viewport");
        }
        else {
            // Both angles on one line: they are short drags, and the panel is
            // narrow. The tooltips give the full names and the unit, the values
            // are read back from the rotation of the light on every frame --
            // which stays what the light is computed from, so that dragging the
            // light in the viewport and editing them agree.
            const GEO::vec2 angles = scene_light_angles(light_direction());
            float azimuth = static_cast<float>(angles.x);
            float elevation = static_cast<float>(angles.y);

            const ImGuiStyle& style = ImGui::GetStyle();
            const float label_width = std::max(
                ImGui::CalcTextSize("az").x,
                ImGui::CalcTextSize("el").x);
            ImGui::PushItemWidth(
                0.5f * (ImGui::GetContentRegionAvail().x - style.ItemSpacing.x)
                - label_width - style.ItemInnerSpacing.x);

            bool moved = ImGui::DragFloat(
                "az", &azimuth, 0.5f, -180.0f, 180.0f, "%.1f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(
                    "Azimuth of the light, in degrees: the angle its direction "
                    "makes with the X axis of the view.");
            ImGui::SameLine();
            moved = ImGui::DragFloat(
                "el", &elevation, 0.5f, -90.0f, 90.0f, "%.1f") || moved;
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(
                    "Elevation of the light, in degrees: the angle its direction "
                    "makes with the horizontal plane of the view.");
            ImGui::PopItemWidth();

            if (moved) {
                set_light_direction(scene_light_direction_from_angles(
                    {azimuth, elevation}));
            }
        }

        // The parameters of the light itself, other than its direction (see
        // SceneLight): GeoBox shades the surfaces with its own program to make
        // them settings (see LightShader), and they are the light of the whole
        // scene, as the light of GLUP is.
        ImGui::ColorEdit3WithPalette("Color", scene_light().color.data());
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Color of the light: it tints what it reaches.");

        const ImGuiStyle& style = ImGui::GetStyle();
        const float light_label_width =
            ImGui::CalcTextSize("intensity").x + style.ItemInnerSpacing.x;
        ImGui::PushItemWidth(-light_label_width);
        ImGui::SliderFloat(
            "intensity", &scene_light().intensity, 0.0f, 4.0f, "%.2f");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(
                "Intensity of the light: 1 is the light GeoGram shades with.");
        ImGui::SliderFloat(
            "ambient", &scene_light().ambient, 0.0f, 1.0f, "%.2f");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(
                "Ambient level, added to every lit fragment: GeoGram's is "
                "0.2.");
        ImGui::DragFloat(
            "shininess", &scene_light().specular_exponent,
            0.5f, 1.0f, 256.0f, "%.0f");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(
                "Exponent of the specular highlight: the higher, the smaller "
                "and sharper the highlight. GeoGram's is 30.");
        ImGui::PopItemWidth();

        if (!LightShader::supported()) {
            ImGui::TextDisabled(
                "Light parameters need the GLUPES2 profile.");
        }

        // What the selected object brings to the light, i.e. its specular
        // factor: it belongs with the controls of the light rather than with the
        // geometry of the object.
        if (const auto object = selected_object_.lock())
            object->draw_lighting_properties();

        // The whole light back on its defaults: the direction, the color, the
        // intensity, the ambient level, the exponent of the highlight, and the
        // specular factor of the object. It closes the group, as it acts on all of
        // its settings.
        if (ImGui::Button("reset", ImVec2(-1.0, 0.0))) {
            light_rotation_.reset();
            scene_light() = SceneLight();
            if (const auto object = selected_object_.lock())
                object->reset_lighting_properties();
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(
                "Put the light back on its defaults: the direction it comes "
                "from, its color, its intensity, its ambient level, the "
                "exponent of its highlight, and the specular factor of the "
                "object.");
    }

    void GeoBoxApplication::draw_objects_properties(
        ) {
        // Geogram's icon font is monospaced (advance = 1.5*font_size), which can
        // exceed the default button height. Size the button to the icon's actual
        // text extent so ImGui's (0.5,0.5) text alignment centers the glyph.
        const float icon_text_width =
            ImGui::CalcTextSize(GEO::icon_UTF8("xmark").c_str()).x;
        const auto icon_button_size = 0.75f * std::max(
            ImGui::GetFrameHeight(),
            icon_text_width + 2.0f * ImGui::GetStyle().FramePadding.x);

        // With the box shrunken to 75%, tighten the buttons' inner padding so
        // the glyph still fits and stays centered inside it. FramePadding.y must
        // stay 0: it becomes the button's text baseline, and a nonzero value
        // makes SameLine() shift the following Selectable down by that amount,
        // so its text/highlight would stick out below the buttons.
        const ImVec2 icon_button_padding(
            std::max(0.0f, (icon_button_size - icon_text_width) * 0.5f),
            0.0f);

        // Master row: actions that apply to all objects.
        // Give the master row a distinct background so it stands out from object rows.
        ImGui::GetWindowDrawList()->AddRectFilled(
            ImGui::GetCursorScreenPos(),
            ImVec2(
                ImGui::GetCursorScreenPos().x + ImGui::GetContentRegionAvail().x,
                ImGui::GetCursorScreenPos().y + icon_button_size),
            ImGui::GetColorU32(ImVec4(0.35f, 0.61f, 0.80f, 0.8f)));
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, icon_button_padding);

        // Show / hide all objects.
        const bool all_visible = std::ranges::all_of(objects_,
                                                     [](const std::shared_ptr<BaseObject>& object) {
                                                         return object->visible();
                                                     });
        if (ImGui::Button(
            GEO::icon_UTF8(all_visible ? "eye" : "eye-slash").c_str(),
            ImVec2(icon_button_size, icon_button_size)
            )) {
            for (const auto& object : objects_)
                object->set_visible(!all_visible);
        }

        ImGui::SameLine();
        // Focus the camera on all objects.
        if (ImGui::Button(
            GEO::icon_UTF8("camera").c_str(),
            ImVec2(icon_button_size, icon_button_size)
            ) && !objects_.empty())
            camera_focus();

        ImGui::SameLine();
        // Delete all objects.
        if (ImGui::Button(
            GEO::icon_UTF8("xmark").c_str(),
            ImVec2(icon_button_size, icon_button_size)
            )) {
            objects_.clear();
            selected_object_.reset();
        }
        ImGui::PopStyleVar();

        ImGui::SameLine();
        // Clicking the rest of the row clears the current object selection.
        ImGui::PushStyleVar(
            ImGuiStyleVar_SelectableTextAlign, ImVec2(0.0f, 0.5f));
        if (ImGui::Selectable(
            "Clear Selection", false, 0, ImVec2(0.0f, icon_button_size)
            ))
            selected_object_.reset();
        ImGui::PopStyleVar();

        ImGui::Separator();

        for (auto it = objects_.begin(); it != objects_.end();) {
            const auto& base_object = *it;

            ImGui::PushID(base_object.get());

            ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, icon_button_padding);
            if (ImGui::Button(
                GEO::icon_UTF8(
                    base_object->visible() ? "eye" : "eye-slash").c_str(),
                ImVec2(icon_button_size, icon_button_size)
                ))
                base_object->set_visible(!base_object->visible());

            ImGui::SameLine();
            // Focus the camera on this object's bounding box.
            if (ImGui::Button(
                GEO::icon_UTF8("camera").c_str(),
                ImVec2(icon_button_size, icon_button_size)
                ))
                camera_focus(base_object);

            ImGui::SameLine();
            if (ImGui::Button(
                GEO::icon_UTF8("xmark").c_str(),
                ImVec2(icon_button_size, icon_button_size)
                )) {
                ImGui::PopStyleVar();
                it = objects_.erase(it);
                ImGui::PopID();

                continue;
            }
            ImGui::PopStyleVar();

            ImGui::SameLine();
            // The rest of the row (name + trailing space) is clickable and
            // selects the object; the text is vertically centered like the buttons.
            ImGui::PushStyleVar(
                ImGuiStyleVar_SelectableTextAlign, ImVec2(0.0f, 0.5f));
            const bool is_selected =
                (selected_object_.lock() == base_object);
            if (ImGui::Selectable(
                base_object->name().c_str(),
                is_selected,
                0,
                ImVec2(0.0f, icon_button_size)
                ))
                selected_object_ = base_object;
            ImGui::PopStyleVar();

            // Right-clicking the row acts on that object: the operations that
            // need to know which object they work on belong here, not in the
            // File menu.
            if (ImGui::BeginPopupContextItem()) {
                if (ImGui::MenuItem(GEO::icon_UTF8("save") + " Save as..."))
                    open_object_save_dialog(base_object);
                ImGui::EndPopup();
            }

            ++it;
            ImGui::PopID();

            ImGui::Separator();
        }
    }

    void GeoBoxApplication::open_object_save_dialog(
        const std::shared_ptr<BaseObject>& object
        ) {
        // The formats the dialog offers: the ones the application knows how to
        // write (the extensions the registered IO handlers answer for) that the
        // object also accepts -- what a kind of object can store is up to the
        // object (see BaseObject::can_write_extension()).
        std::vector<std::string> registered_extensions;
        GEO::String::split_string(
            supported_write_file_extensions(), ';', registered_extensions);

        const auto writable = [&](const std::string& extension) {
            return !extension.empty() &&
                   std::ranges::find(registered_extensions, extension) !=
                       registered_extensions.end() &&
                   object->can_write_extension(extension);
        };

        // The default format is the one of the file the object was read from.
        // An object that was not read from a file -- or that cannot be written
        // in that format -- defaults to the first format it accepts: there is no
        // format every kind of object can be written in (.geogram for a mesh,
        // .fra for a directional field, ...).
        const std::string original_extension =
            GEO::FileSystem::extension(object->filepath());
        std::string default_extension = original_extension;
        if (!writable(default_extension)) {
            const auto first = std::ranges::find_if(
                registered_extensions,
                [&](const std::string& extension) {
                    return writable(extension);
                });
            default_extension =
                first != registered_extensions.end() ? *first : std::string();
        }

        // The format list of the dialog starts on its first entry, so the
        // default format goes first; the other writable formats follow, in the
        // order the application registered them. The dialog takes its first
        // entry as the format it saves in, and the file name below is built with
        // that one -- the two cannot disagree.
        std::vector<std::string> extensions;
        if (writable(default_extension))
            extensions.push_back(default_extension);
        for (const std::string& extension : registered_extensions) {
            if (extension == default_extension || !writable(extension))
                continue;
            extensions.push_back(extension);
        }
        if (extensions.empty()) {
            LOG::WARN(
                "`{}` cannot be saved: no file format is available for it.",
                object->name());
            return;
        }

        // The dialog proposes the name of the file the object was read from --
        // its name only, without the directory: the dialog shows the directory
        // it saves into, and repeating the path in the name box is noise. An
        // object that was not read from a file is proposed its own name. Either
        // way with the format the dialog starts on, i.e. the first one of the
        // list.
        std::string name;
        if (!object->filepath().empty())
            name = GEO::FileSystem::base_name(object->filepath(), true);
        if (name.empty())
            name = object->name();
        std::string suggested = name + "." + extensions.front();
        suggested.resize(
            std::min(
                suggested.size(),
                std::size_t(GEO::geo_imgui_string_length - 1)));
        std::copy(suggested.begin(), suggested.end(), filename_);
        filename_[suggested.size()] = '\0';

        object_to_save_ = object;
        ImGui::OpenFileDialog(
            "##object_save_dlg",
            GEO::String::join_strings(extensions, ';').c_str(),
            filename_,
            ImGuiExtFileDialogFlags_Save);
    }

    void GeoBoxApplication::draw_object_properties_window(
        ) {
        if (!object_properties_visible_)
            return;
        if (selected_object_.expired())
            return;

        constexpr float WINDOWS_WIDTH = 0.2f;

        const ImVec2 viewport_size = ImGui::GetMainViewport()->Size;
        if (object_properties_size_.x <= 0.0f)
            object_properties_size_ = ImVec2(
                viewport_size.x * WINDOWS_WIDTH, viewport_size.y * 0.5f);

        // Anchor the window's right edge to the viewport's right edge, so it
        // stays docked in the top-right corner when the viewport is resized
        // while keeping the tracked (constant) width and height.
        ImGui::SetNextWindowPos(
            ImVec2(
                viewport_size.x - object_properties_size_.x,
                ImGui::GetFrameHeight()),
            ImGuiCond_Always
        );
        ImGui::SetNextWindowSize(object_properties_size_, ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowBgAlpha(0.6f);
        if (ImGui::Begin(
            "Object Properties", nullptr,
            ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoMove)) {
            draw_object_properties();
            object_properties_size_ = ImGui::GetWindowSize();
        }

        ImGui::End();
    }

    void GeoBoxApplication::draw_object_properties(
        ) {
        const auto selected_object = selected_object_.lock();
        if (!selected_object)
            return;
        if (!selected_object->visible())
            return;

        selected_object->draw_object_properties(objects_);
    }

    void GeoBoxApplication::draw_scene(
        ) {
        // GeoGram set its own clipping up before calling this (see
        // SimpleApplication::draw_scene_begin()): the clipping of the viewer
        // replaces it, so that the scene is cut by one plane only.
        apply_clip();

        for (const auto& base_object : objects_) {
            // An object whose data cannot be rendered is left out of the scene:
            // drawing it would read outside of it (see
            // BaseObject::renderable()).
            if (base_object->visible() && base_object->renderable())
                base_object->draw_scene(lighting_);
        }
    }

    void GeoBoxApplication::draw_menu_bar(
        ) {
        if (!menubar_visible_)
            return;

        if (ImGui::BeginMainMenuBar()) {
            if (ImGui::BeginMenu("File")) { // =========================================================================
                if (!supported_read_file_extensions().empty())
                    draw_load_menu();

                if (!current_file_.empty()) {
                    if (ImGui::MenuItem(GEO::icon_UTF8("save") + " Save")) {
                        if (save(current_file_))
                            LOG::INFO("Saved {}", current_file_);
                        else
                            LOG::ERROR("Could not save {}", current_file_);
                    }
                }
                // "Save as..." lives in the context menu of the objects of the
                // scene (see draw_objects_properties()): the File menu cannot
                // know which object to save, since the scene holds several.
                draw_fileops_menu();

                draw_about();

                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("Windows")) {
                draw_windows_menu();
                ImGui::EndMenu();
            }
            draw_application_menus();

            ImGui::EndMainMenuBar();
        }
    }

    void GeoBoxApplication::draw_about(
        ) {
        ImGui::Separator();
        if(ImGui::BeginMenu(GEO::icon_UTF8("info") + " About...")) {
            ImGui::Text("Geolio Visualization Tool");
            ImGui::Text("\n");
            ImGui::Separator();
            ImGui::Text("This is a visualization tool based on the Geolio library (derived from the Geogram library).");
            ImGui::Text("\n");
            ImGui::Text("GEOLIO website: ");
            ImGui::Text("https://github.com/Canjia-Huang/geolio");
            ImGui::Text("\n");
            ImGui::Text("GEOGRAM website (version: %s): ", GEO::Environment::instance()->get_value("version").c_str());
            ImGui::Text("https://github.com/BrunoLevy/geogram");

            ImGui::EndMenu();
        }
    }

    void GeoBoxApplication::draw_windows_menu(
        ) {
        ImGui::MenuItem(
            GEO::icon_UTF8("eye") + " Controller properties",
            0,
            &controller_properties_visible_
        );
        ImGui::MenuItem(
            GEO::icon_UTF8("edit") + " Object properties",
            0,
            &object_properties_visible_
        );
        ImGui::MenuItem(
            GEO::icon_UTF8("group-arrows-rotate") + " Arc ball",
            0,
            &arc_ball_visible_
        );
        ImGui::MenuItem(
            GEO::icon_UTF8("flask") + " ImGui demo",
            0,
            &imgui_demo_visible_
        );
        {
            bool needs_to_close = false;
            needs_to_close = ImGui::BeginMenu(GEO::icon_UTF8("font") + " Font size");
            if (phone_screen_ || needs_to_close) {
                static GEO::index_t font_sizes[] = {10, 12, 14, 16, 18, 22};
                for (unsigned int font_size : font_sizes) {
                    bool selected = (get_font_size() == font_size);
                    if(ImGui::MenuItem(
                            GEO::String::to_string(font_size),
                           nullptr,
                           &selected))
                        set_font_size(font_size);

                }
                if (needs_to_close)
                    ImGui::EndMenu();
            }
        }
        {
            bool needs_to_close = false;
            needs_to_close = ImGui::BeginMenu(GEO::icon_UTF8("cog") + " Style");
            if(phone_screen_ || needs_to_close) {
                std::vector<std::string> styles;
                GEO::String::split_string(get_styles(), ';', styles);
                for (const auto & style : styles) {
                    bool selected = (get_style() == style);
                    if(ImGui::MenuItem(style, nullptr, &selected))
                        set_style(style);
                }
                if(needs_to_close)
                    ImGui::EndMenu();
            }
        }
        {
            ImGui::Separator();
            // Toggle between the redraw-on-demand scheme (default) and the
            // original scheme where every redraw re-renders the whole scene.
            if (ImGui::MenuItem(
                GEO::icon_UTF8("sync-alt") + " On-demand redraw",
                nullptr,
                &redraw_on_demand_
                )) {
                // Force one full render right after switching, so the view
                // does not stay stuck on a stale frame.
                scene_dirty_ = true;
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(
                    "alleviate UI operation stutter during large-scale rendering.");
        }
    }

    void GeoBoxApplication::draw_application_menus(
        ) {
        // Purely generic: the hierarchy and the entries both come from the
        // menu tree, which is built from the registration data (see
        // application/register_applications.h). Adding a sub-application or a
        // new sub-menu level does not require changing this function.
        if (application_menu_.empty())
            return;
        if (ImGui::BeginMenu("Applications")) {
            draw_application_menu(application_menu_);
            ImGui::EndMenu();
        }
    }

    void GeoBoxApplication::draw_rotation_gizmo(
        ) {
        if (!arc_ball_visible_)
            return;

        // There is nothing to rotate if no object is loaded.
        if (objects_.empty())
            return;

        const float scale = ImGui::scaling();
        const float size = 165.0f * scale;
        const float margin = 16.0f * scale;

        const ImVec2 viewport_size = ImGui::GetMainViewport()->Size;

        // Keep the gizmo above the status bar when it is visible.
        float bottom_margin = margin;
        if (status_bar_->active()) {
            float status_height = status_bar_->get_window_height();
            if (status_height == 0.0f)
                status_height = static_cast<float>(get_font_size());
            bottom_margin += status_height * 1.5f;
        }

        ImOGuizmo::SetRect(
            margin, viewport_size.y - size - bottom_margin, size);

        // The current model rotation. GEO::mat4 stores the same row-major
        // layout as GLUP, and imoguizmo expects its matrices transposed
        // (standard column-major); a plain element copy does exactly that
        // transpose, so the gizmo's axes match the model's orientation.
        const double* rotation = object_rotation_.get_value().data();
        float view[16];
        for (GEO::index_t i = 0; i < 16; ++i)
            view[i] = static_cast<float>(rotation[i]);

        // GLUP projection matrix, still current from draw_graphics().
        // imoguizmo sizes the axes directly from the projection's scale, but
        // geogram's camera has a very narrow field of view (9 deg aperture),
        // whose projection matrix has a large vertical scale (m11 ~ 2.5x
        // aspect). Normalize the projection so the axes keep the library's
        // intended on-screen length; the uniform scale only affects the
        // length, not the axis directions.
        double proj_d[16];
        glupGetMatrixdv(GLUP_PROJECTION_MATRIX, proj_d);
        const auto proj_scale = static_cast<float>(1.0 / proj_d[5]);
        float proj[16];
        for (GEO::index_t i = 0; i < 16; ++i)
            proj[i] = static_cast<float>(proj_d[i]) * proj_scale;

        ImOGuizmo::BeginFrame();
        if (ImOGuizmo::DrawGizmo(view, proj, 1.0f)) {
            GEO::mat4 new_rotation;
            for (GEO::index_t i = 0; i < 16; ++i)
                new_rotation.data()[i] = view[i];

            // imoguizmo's lookAt() writes a translation into the last column;
            // object_rotation_ holds a pure rotation, so drop it.
            new_rotation.data()[3]  = 0.0f;
            new_rotation.data()[7]  = 0.0f;
            new_rotation.data()[11] = 0.0f;
            new_rotation.data()[12] = 0.0f;
            new_rotation.data()[13] = 0.0f;
            new_rotation.data()[14] = 0.0f;
            new_rotation.data()[15] = 1.0f;

            object_rotation_.set_value(new_rotation);
            scene_dirty_ = true;
        }
    }

    void GeoBoxApplication::camera_focus(
        const std::shared_ptr<BaseObject>& object_ptr
        ) {
        home();

        if (object_ptr == nullptr) { // focus all
            double xyzmin[3] = {
                std::numeric_limits<double>::max(),
                std::numeric_limits<double>::max(),
                std::numeric_limits<double>::max()
            };
            double xyzmax[3] = {
                -std::numeric_limits<double>::max(),
                -std::numeric_limits<double>::max(),
                -std::numeric_limits<double>::max()
            };
            for (const auto& object : objects_) {
                double bmin[3], bmax[3];
                object->get_bbox(bmin, bmax);
                for (GEO::coord_index_t i = 0; i < 3; ++i) {
                    xyzmin[i] = std::min(xyzmin[i], bmin[i]);
                    xyzmax[i] = std::max(xyzmax[i], bmax[i]);
                }
            }
            set_region_of_interest(
                xyzmin[0], xyzmin[1], xyzmin[2],
                xyzmax[0], xyzmax[1], xyzmax[2]);
            LOG::DEBUG("focus: {}, {}, {}, {}, {}, {}", xyzmin[0], xyzmin[1], xyzmin[2], xyzmax[0], xyzmax[1], xyzmax[2]);
        }
        else {
            double xyzmin[3];
            double xyzmax[3];
            object_ptr->get_bbox(xyzmin, xyzmax);
            set_region_of_interest(
                xyzmin[0], xyzmin[1], xyzmin[2],
                xyzmax[0], xyzmax[1], xyzmax[2]);
            LOG::DEBUG("focus: {}, {}, {}, {}, {}, {}", xyzmin[0], xyzmin[1], xyzmin[2], xyzmax[0], xyzmax[1], xyzmax[2]);
        }

        scene_dirty_ = true;
    }

    namespace
    {
        /**
         * @brief Adds the extensions of the directional fields to a list of
         *        mesh formats.
         * @details .fra is not a mesh format: GeoLio reads and writes the
         *          vectors of a field with its own functions (see
         *          geolio::fra_load()), so GeoGram does not list it, and the
         *          file dialogs would not offer it.
         * @param[in] mesh_extensions Extensions of the mesh formats, separated
         *            by ';' (possibly empty).
         * @return The same list, followed by .fra.
         */
        std::string with_directional_field_extension(
            const std::string& mesh_extensions
            ) {
            std::string extensions = mesh_extensions;
            if (!extensions.empty())
                extensions += ';';
            extensions += "fra";
            return extensions;
        }
    }

    std::string GeoBoxApplication::supported_read_file_extensions(
        ) {
        return with_directional_field_extension(
            SimpleMeshApplication::supported_read_file_extensions());
    }

    std::string GeoBoxApplication::supported_write_file_extensions(
        ) {
        return with_directional_field_extension(
            SimpleMeshApplication::supported_write_file_extensions());
    }

    bool GeoBoxApplication::load(
        const std::string& filepath
        ) {
        home();

        // The .fra files hold the vectors of a directional field, not a mesh:
        // they are read by load_directional_field(), not by the mesh loader.
        if (GEO::FileSystem::extension(filepath) == "fra")
            return load_directional_field(filepath);

        GEO::Mesh mesh;
        if (!mesh.load(filepath)) {
            LOG::ERROR("Cannot load mesh from `{}`!", filepath);
            return false;
        }

        /* Create object */
        const auto object_ptr = std::make_shared<MeshObject>(
            get_filename(filepath),
            filepath,
            mesh);

        objects_.push_back(object_ptr);

        /* Focus */
        // The object checked its own data when it was created (see
        // MeshObject::check_validity()): framing a camera on data that cannot be
        // read would put coordinates that are not finite in the view matrix,
        // which breaks the display of the whole scene.
        if (object_ptr->renderable())
            camera_focus(object_ptr);
        selected_object_ = object_ptr;

        scene_dirty_ = true;

        return true;
    }

    bool GeoBoxApplication::load_directional_field(
        const std::string& filepath
        ) {
        GEO::index_t vectors_nb_per_element = GEO::NO_INDEX;
        std::vector<double> vectors;
        if (!fra_load(filepath, vectors_nb_per_element, vectors)) {
            LOG::ERROR("Cannot load a directional field from `{}`!", filepath);
            return false;
        }

        const auto object = std::make_shared<DirectionalObject>(
            get_filename(filepath),
            filepath,
            vectors_nb_per_element,
            vectors);

        // The field draws vectors on the elements of a mesh: when exactly one
        // mesh object of the scene has as many elements as the field has
        // frames, it is attached to it, so that the file shows up without
        // having to pick anything. When several meshes match -- or one matches
        // in several ways -- the properties of the object offer the choice.
        {
            std::shared_ptr<MeshObject> candidate;
            GEO::MeshElementsFlags candidate_kind = GEO::MESH_NONE;
            bool ambiguous = false;
            for (const auto& other : objects_) {
                const auto mesh_object =
                    std::dynamic_pointer_cast<MeshObject>(other);
                if (mesh_object == nullptr)
                    continue;
                const auto kinds = object->attachable_subelements(*mesh_object);
                if (kinds.empty())
                    continue;
                ambiguous = ambiguous || candidate != nullptr ||
                            kinds.size() != 1;
                candidate = mesh_object;
                candidate_kind = kinds.front();
            }
            if (!ambiguous && candidate != nullptr)
                object->attach_to(candidate, candidate_kind);
        }

        objects_.push_back(object);
        // Without a mesh the object is not drawn, and has no extent to frame
        // the camera on (see DirectionalObject::attached()).
        if (object->attached())
            camera_focus(object);
        selected_object_ = object;

        scene_dirty_ = true;

        LOG::INFO(
            "Loaded directional field `{}`: {} frames of {} vectors.",
            object->name(),
            object->nb_frames(),
            object->vectors_nb_per_element());
        return true;
    }
}
