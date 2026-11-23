//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/31.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_FILE_DIALOG_H
#define GEOLIO_FILE_DIALOG_H
#include <cstddef>
#include <string>
#include <vector>
#include <geogram/basic/file_system.h>
#include <geogram_gfx/gui/application.h>
#include <geogram_gfx/imgui_ext/imgui_ext.h>

namespace geolio::geobox
{
    /**
     * @brief Root node of a GeoGram file dialog, placed in a chosen directory.
     * @details A file dialog lists the directory of the node it is rooted at,
     *          and reads it from that node only when the node changes: opening
     *          the dialog again with the node it already has keeps it where the
     *          user left it (see the FileDialog class of GeoGram's imgui_ext).
     *          This node answers the two questions the dialog asks to find out
     *          where to start -- its working directory, and the directory
     *          GeoGram prefers on Windows and Android -- with the directory it
     *          was given, and hands every other operation to the root of the
     *          real file system, so that a dialog rooted at it works like any
     *          other. Rooting a dialog at a node it has not seen yet therefore
     *          puts the dialog in that directory (see
     *          open_file_dialog_in_directory()).
     */
    class FileDialogRootNode : public GEO::FileSystem::Node
    {
    public:
        /**
         * @brief Constructs a node that puts the dialogs rooted at it in
         *        @p directory.
         * @param[in] directory absolute path of that directory.
         */
        explicit FileDialogRootNode(
            const std::string& directory = std::string()
            )
            : directory_(directory) {
            // The node that does the work: the one GeoGram roots its dialogs
            // at when they are not given a node of their own.
            GEO::FileSystem::get_root(real_root_);
        }

        // A node stands for a file system, it is handed to the dialog rather
        // than stored, so it is neither copied nor moved.
        FileDialogRootNode(const FileDialogRootNode&) = delete;
        FileDialogRootNode& operator=(const FileDialogRootNode&) = delete;

        /**
         * @brief Sets the directory the dialogs rooted at this node start in.
         * @param[in] directory absolute path of that directory.
         */
        void set_directory(const std::string& directory) {
            directory_ = directory;
        }

        /**
         * @brief Sets whether this node is being handed to a dialog.
         * @details GeoGram does not ask the same thing of a node for the two
         *          uses it has of its "documents directory": it is where the
         *          dialogs of Windows and Android start, and where the "home"
         *          button of a dialog goes on every platform. Only the first of
         *          them must be the directory of this node, and only while the
         *          node is being handed to the dialog (see
         *          open_file_dialog_in_directory()), which is what the flag
         *          tells apart.
         * @param[in] seeding true for the duration of that call.
         */
        void set_seeding(const bool seeding) {
            seeding_ = seeding;
        }

        /**
         * @brief Returns the directory the dialogs rooted here start in.
         * @return The directory this node was given.
         */
        std::string get_current_working_directory() override {
            return directory_;
        }

        /**
         * @brief Returns the directory the "home" button of the dialog goes to.
         * @return The directory this node was given while the node is being
         *         handed to a dialog, the documents directory of the real file
         *         system otherwise (see set_seeding()).
         */
        std::string documents_directory() override {
            return seeding_ ? directory_ : real_root_->documents_directory();
        }

        // Everything below is the real file system: a dialog is rooted at this
        // node to be placed in a directory, not to work on another file
        // system. The string and traversal helpers of GEO::FileSystem::Node
        // need no override, they are implemented on top of these.

        bool is_file(const std::string& path) override {
            return real_root_->is_file(path);
        }

        bool is_directory(const std::string& path) override {
            return real_root_->is_directory(path);
        }

        bool create_directory(const std::string& path) override {
            return real_root_->create_directory(path);
        }

        bool delete_directory(const std::string& path) override {
            return real_root_->delete_directory(path);
        }

        bool delete_file(const std::string& path) override {
            return real_root_->delete_file(path);
        }

        bool get_directory_entries(
            const std::string& path, std::vector<std::string>& result
            ) override {
            return real_root_->get_directory_entries(path, result);
        }

        bool set_current_working_directory(const std::string& path) override {
            return real_root_->set_current_working_directory(path);
        }

        bool rename_file(
            const std::string& old_name, const std::string& new_name
            ) override {
            return real_root_->rename_file(old_name, new_name);
        }

        GEO::Numeric::uint64 get_time_stamp(const std::string& path) override {
            return real_root_->get_time_stamp(path);
        }

        bool set_executable_flag(const std::string& filename) override {
            return real_root_->set_executable_flag(filename);
        }

        bool touch(const std::string& filename) override {
            return real_root_->touch(filename);
        }

        std::string normalized_path(const std::string& path) override {
            return real_root_->normalized_path(path);
        }

        std::string home_directory() override {
            return real_root_->home_directory();
        }

        std::string load_file_as_string(const std::string& path) override {
            return real_root_->load_file_as_string(path);
        }

        bool copy_file(
            const std::string& from, const std::string& to
            ) override {
            return real_root_->copy_file(from, to);
        }

    private:
        /** @brief Directory the dialogs rooted at this node start in. */
        std::string directory_;
        /** @brief Node of the real file system, which does all the work. */
        GEO::FileSystem::Node* real_root_ = nullptr;
        /** @brief Whether the node is being handed to a dialog. */
        bool seeding_ = false;
    };

    /**
     * @brief Directory GeoGram's file dialogs start in by default.
     * @details The directory a dialog lists when it is not rooted at a node of
     *          its own: the working directory of the process, or the documents
     *          directory ("My Documents") on the platforms where GeoGram
     *          prefers it.
     * @return The absolute path of that directory.
     */
    inline std::string default_file_dialog_directory() {
#if defined(GEO_OS_WINDOWS) || defined(GEO_OS_ANDROID)
        return GEO::FileSystem::documents_directory();
#else
        return GEO::FileSystem::get_current_working_directory();
#endif
    }

    /**
     * @brief Opens a GeoGram file dialog that starts in @p directory.
     * @details The directory of a dialog is the one of the node it is rooted
     *          at, which GeoGram reads only when the node changes (see
     *          FileDialogRootNode). Two nodes are therefore used in turn: the
     *          node handed to the dialog is never the one it already has, so
     *          the dialog starts in @p directory every time it is opened, and
     *          GeoGram never keeps a pointer to a node that has to be
     *          destroyed -- which it has no way of giving back. Both rest on
     *          GeoGram re-reading the node when it changes: a GeoGram that
     *          stopped doing so would leave the dialogs in the directory the
     *          user left them in, as they are without this helper.
     * @param[in] label Window label of the dialog (see ImGui::FileDialog()).
     * @param[in] extensions ';'-separated list of the extensions it manages.
     * @param[in] filename Name proposed in its file name box.
     * @param[in] flags Whether the dialog loads or saves.
     * @param[in] directory Absolute path of the directory it starts in.
     */
    inline void open_file_dialog_in_directory(
        const char* label,
        const char* extensions,
        const char* filename,
        const ImGuiExtFileDialogFlags flags,
        const std::string& directory
        ) {
        constexpr std::size_t NODES = 2;
        static FileDialogRootNode nodes[NODES];
        static std::size_t next_node = 0;

        FileDialogRootNode& node = nodes[next_node];
        next_node = (next_node + 1) % NODES;
        node.set_directory(directory);
        node.set_seeding(true);
        ImGui::OpenFileDialog(label, extensions, filename, flags, &node);
        node.set_seeding(false);
    }

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
