//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/31.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_FILE_DIALOG_H
#define GEOLIO_FILE_DIALOG_H
#include <cstddef>
#include <geogram_gfx/gui/application.h>
#include <geogram_gfx/imgui_ext/imgui_ext.h>

namespace geolio::geobox
{
    /**
     * @brief Draws a GeoGram file dialog, laid out like GeoGram's own dialogs.
     * @details GeoGram lays its dialogs out with ImGui::scaling(), a ratio of
     *          the font size and the display pixel ratio. With the fonts GeoBox
     *          uses it is about 0.35, while the dialogs are designed for the
     *          1.0 - 2.0 of GeoGram's own applications: at 0.35 the room they
     *          reserve for their footer is smaller than the footer itself, so
     *          the dialog shows a scrollbar, and their two list boxes are a
     *          little too wide for the space left by that scrollbar. The scaling
     *          is raised for the duration of the dialog -- GeoBox's own widgets
     *          that use it (e.g. the size of the rotation gizmo) are drawn
     *          outside of it and keep theirs.
     * @param[in] label Window label of the dialog (see ImGui::FileDialog()).
     * @param[in,out] filename Buffer that receives the selected file name.
     * @param[in] filename_length Size of @p filename.
     * @return true when a file was selected.
     */
    inline bool draw_file_dialog(
        const char* label,
        char* filename,
        const std::size_t filename_length
        ) {
        // The scaling the dialogs are designed for, and the font size to global
        // font scale ratio it corresponds to (see ImGui::scaling()).
        constexpr float DIALOG_SCALING = 1.5f;
        const float font_size = ImGui::GetFontSize();
        const float scaling_per_unit =
            font_size > 40.0f ? font_size / 30.0f : font_size / 20.0f;

        GEO::Application* const application = GEO::Application::instance();
        const float application_scaling = application->get_font_global_scale();
        application->set_font_global_scale(DIALOG_SCALING / scaling_per_unit);
        const bool selected =
            ImGui::FileDialog(label, filename, filename_length);
        application->set_font_global_scale(application_scaling);
        return selected;
    }
}

#endif //GEOLIO_FILE_DIALOG_H
