//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/19.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "mesh_object.h"
#include <map>
#include <geogram/basic/file_system.h>
#include <geogram_gfx/imgui_ext/imgui_ext.h>
#include <geogram_gfx/imgui_ext/icon_font.h>
#include <geogram_gfx/third_party/imgui/imgui.h>
#include <geogram/image/image.h>
#include <geogram/image/image_library.h>
#include <geogram/mesh/mesh_geometry.h>
#include <geogram/mesh/mesh_io.h>
#include <geogram_gfx/GLUP/GLUP.h>
// glupPrivateVertex*()/glupPrivateTexCoord*(): the immediate-mode entry points
// used by MeshGfx::draw_vertex() and MeshGfx::draw_attribute_as_tex_coord(),
// which the cell-facet passes below reproduce (see GeoGram PR #386).
#include <geogram_gfx/GLUP/GLUP_private.h>
#include <geogram_gfx/basic/GL.h>
#include <geolio/geobox/file_dialog.h>

#include "geolio/common/log.h"

namespace geolio::geobox
{
    namespace
    {
        /**
         * @brief GL textures built for the TEXTURE painting mode, indexed by
         *        image file name.
         * @details Shared by all the objects, and released by
         *          release_texture_cache() while the GL context is current.
         */
        std::map<std::string, GLuint> texture_cache;

        /**
         * @brief Builds a GL texture from an image file.
         * @details The image is converted to RGBA8, uploaded and mipmapped.
         *          Images are tiled (GL_REPEAT), as Graphite does for texture
         *          painting.
         * @param[in] filename Path of the image file.
         * @return The GL texture, or 0 if the image could not be loaded or has
         *         an unsupported encoding.
         */
        GLuint create_texture_from_image(const std::string& filename) {
            GEO::Image_var image =
                GEO::ImageLibrary::instance()->load_image(filename);
            if (image.is_null()) {
                LOG::ERROR("Cannot load texture image `{}`!", filename);
                return 0;
            }
            if (image->component_encoding() != GEO::Image::BYTE) {
                LOG::ERROR(
                    "Unsupported component encoding in texture image `{}`!",
                    filename);
                return 0;
            }

            const std::size_t width = image->width();
            const std::size_t height = image->height();
            const std::size_t nb_pixels = width * height;
            const std::size_t bpp = image->bytes_per_pixel();
            const GEO::Memory::byte* source = image->base_mem_byte_ptr();

            // stb decodes PNG/JPEG as RGBA8, so the common case can be
            // uploaded directly instead of allocating a second buffer (an 8K
            // image alone is 256 MiB).
            std::vector<GLubyte> converted;
            const GLubyte* pixels = nullptr;
            if (image->color_encoding() == GEO::Image::RGBA && bpp == 4) {
                pixels = reinterpret_cast<const GLubyte*>(source);
            }
            else {
                converted.resize(4 * nb_pixels);
                for (std::size_t i = 0; i < nb_pixels; ++i) {
                    GLubyte r = 0;
                    GLubyte g = 0;
                    GLubyte b = 0;
                    GLubyte a = 255;
                    switch (image->color_encoding()) {
                    case GEO::Image::GRAY:
                        r = g = b = source[0];
                        break;
                    case GEO::Image::RGB:
                        r = source[0];
                        g = source[1];
                        b = source[2];
                        break;
                    case GEO::Image::BGR:
                        b = source[0];
                        g = source[1];
                        r = source[2];
                        break;
                    case GEO::Image::RGBA:
                        r = source[0];
                        g = source[1];
                        b = source[2];
                        a = source[3];
                        break;
                    default:
                        LOG::ERROR(
                            "Unsupported color encoding in texture image "
                            "`{}`!",
                            filename);
                        return 0;
                    }
                    converted[4 * i + 0] = r;
                    converted[4 * i + 1] = g;
                    converted[4 * i + 2] = b;
                    converted[4 * i + 3] = a;
                    source += bpp;
                }
                pixels = converted.data();
            }

            GLuint texture = 0;
            glGenTextures(1, &texture);
            glBindTexture(GL_TEXTURE_2D, texture);
            glTexImage2D(
                GL_TEXTURE_2D, 0, GL_RGBA, GLsizei(width), GLsizei(height), 0,
                GL_RGBA, GL_UNSIGNED_BYTE, pixels);
            glGenerateMipmap(GL_TEXTURE_2D);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
            glTexParameteri(
                GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                GL_LINEAR_MIPMAP_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glBindTexture(GL_TEXTURE_2D, 0);
            return texture;
        }

        /**
         * @brief Tests whether MeshGfx can sample an attribute on a store as
         *        texture coordinates.
         * @details The volume cell-facet path is handled by GeoBox itself for
         *          scalar attributes only, so vector attributes on that store
         *          are not offered (see cell_facets_attribute_active()).
         */
        bool can_be_textured(const GEO::MeshElementsFlags subelements) {
            switch (subelements) {
            case GEO::MESH_VERTICES:
            case GEO::MESH_EDGES:
            case GEO::MESH_FACETS:
            case GEO::MESH_FACET_CORNERS:
            case GEO::MESH_CELLS:
            case GEO::MESH_CELL_CORNERS:
                return true;
            default:
                return false;
            }
        }

        /**
         * @brief Splits a full attribute name into its store and base name.
         */
        void split_attribute_name(
            const std::string& full_name,
            GEO::MeshElementsFlags& subelements,
            std::string& name
            ) {
            GEO::index_t component = 0;
            subelements = GEO::MESH_NONE;
            name.clear();
            if (!GEO::Mesh::parse_attribute_name(
                    full_name, subelements, name, component)) {
                subelements = GEO::MESH_NONE;
                name.clear();
            }
        }

        /**
         * @brief Directory of the texture images bundled with GeoBox.
         * @details The source directory is recorded at configure time
         *          (GEOLIO_TEXTURES_DIR); the path relative to the working
         *          directory is a fallback for builds that do not define it.
         *          The result is absolute, so that it keeps designating the
         *          same directory after the working directory is restored.
         */
        std::string texture_directory() {
#ifdef GEOLIO_TEXTURES_DIR
            if (GEO::FileSystem::is_directory(GEOLIO_TEXTURES_DIR))
                return GEOLIO_TEXTURES_DIR;
#endif
            return GEO::FileSystem::normalized_path(
                GEO::FileSystem::get_current_working_directory() +
                "/src/geolio/geobox/textures");
        }

        /**
         * @brief Opens the texture image picker in the texture directory.
         * @details GeoGram's FileDialog starts in the working directory of the
         *          process when it is created. The working directory is
         *          therefore changed for the duration of this call (the dialog
         *          is created and lists its first directory there) and
         *          restored right after, so the rest of the application is not
         *          affected. Later openings keep the directory the user chose.
         */
        void open_texture_file_dialog(
            const char* label,
            const char* extensions,
            const char* filename
            ) {
            const std::string previous_directory =
                GEO::FileSystem::get_current_working_directory();
            const bool changed =
                GEO::FileSystem::set_current_working_directory(
                    texture_directory());
            ImGui::OpenFileDialog(
                label, extensions, filename, ImGuiExtFileDialogFlags_Load);
            if (changed) {
                GEO::FileSystem::set_current_working_directory(
                    previous_directory);
            }
        }

        /**
         * @brief Image displayed by the TEXTURE painting mode by default.
         * @details The bundled 8K UV checker.
         */
        std::string default_texture_filename() {
            std::string directory = texture_directory();
            if (!directory.empty() && directory.back() != '/')
                directory += '/';
            return directory + "CustomUVChecker_byValle_8K_checker_Normals.png";
        }
    }

    void release_texture_cache() {
        for (const auto& [name, texture] : texture_cache) {
            if (texture != 0)
                glDeleteTextures(1, &texture);
        }
        texture_cache.clear();
    }

    MeshObject::MeshObject(
        std::string name,
        std::string filepath,
        const GEO::Mesh& mesh
        ) : BaseObject(std::move(name), std::move(filepath))
    {
        mesh_.copy(mesh);
        mesh_gfx_.set_mesh(&mesh_);
        surface_has_borders_ = compute_surface_has_borders();
        element_counts_ = count_elements();

        if (mesh_.edges.nb() == 0 && mesh_.facets.nb() == 0 && mesh_.cells.nb() == 0)
            show_vertices_ = true;
        // Seed the absolute (model-space) marker size from the mesh extent:
        // one hundredth of the bounding-box diagonal roughly reproduces the
        // on-screen size of the former fixed pixel points at the initial
        // framing, whatever the scale of the model.
        if (mesh_.vertices.nb() != 0) {
            if (const float diag = bbox_diagonal();
                diag > 0.0f)
                vertices_size_ = 0.003f * diag;
        }

        if (mesh_.edges.nb() == 0)
            show_edges_ = false;
        if (mesh_.facets.nb() == 0)
            show_surface_mesh_ = false;
        if (mesh_.cells.nb() == 0)
            show_volume_ = false;

        set_attribute(vertices_point_attribute_name_);

        // Default image of the TEXTURE painting mode: the UV checker bundled
        // in the texture directory (see default_texture_filename()).
        texture_filename_ = default_texture_filename();

        // Data that cannot be rendered crashes the drawing passes: the object
        // checks it as soon as it exists, and warns the user if it has to (see
        // draw_validity_dialog()). Done here, in the most derived constructor,
        // because a virtual call in the base constructor would not reach the
        // check of this class.
        MeshObject::check_validity();
    }

    void MeshObject::draw_object_properties(
        ) {
        ImGui::PushID(this);

        BaseObject::draw_object_properties();

        // An operation may have changed the mesh since the last frame (see
        // mesh_changed()): the Info panel and the border controls below would
        // otherwise show what the mesh was before it.
        update_mesh_caches();

        if (!data_valid_) {
            // The mesh was kept as it is (the validity dialog was answered with
            // "skip"), so it is not drawn. Checking again re-opens the dialog,
            // where it can be repaired.
            ImGui::TextDisabled("Not displayed: invalid data.");
            if (ImGui::Button("Check validity", ImVec2(-1, 0)))
                check_validity();
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip(
                    "Run the validity check again, and offer to repair the "
                    "mesh.");
            }
        }

        if (ImGui::CollapsingHeader("Info")) {
            /* == Element count ==================================================================================== */
            // One row per element kind (vertices, edges, facets, cells), with
            // the kinds the mesh does not contain left out, and indented rows
            // breaking a kind down: the dimension of the vertices, the arity
            // of the facets, the type of the cells. A breakdown row whose
            // count is zero is left out too, so the table only ever shows
            // what the mesh actually has.
            const auto format_count = [](const GEO::index_t count) {
                // Thousands separators: a multi-million element mesh stays
                // readable, and the counts remain comparable column-wise.
                const std::string digits =
                    std::to_string(static_cast<unsigned long long>(count));
                std::string formatted;
                formatted.reserve(digits.size() + digits.size() / 3);
                for (std::size_t i = 0; i < digits.size(); ++i) {
                    if (i != 0 && (digits.size() - i) % 3 == 0)
                        formatted.push_back(',');
                    formatted.push_back(digits[i]);
                }
                return formatted;
            };

            if (ImGui::BeginTable(
                "##InfoTable", 2,
                ImGuiTableFlags_BordersOuter | ImGuiTableFlags_BordersInnerV |
                ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)
                ) {
                ImGui::TableSetupColumn(
                    "Element", ImGuiTableColumnFlags_WidthStretch, 0.66f);
                ImGui::TableSetupColumn(
                    "Count", ImGuiTableColumnFlags_WidthStretch, 0.34f);

                // Moves the cursor so that the text drawn next ends flush with
                // the right edge of the cell: the counts are numbers, and they
                // read better lined up on their last digit.
                const auto right_aligned = [](const char* text) {
                    ImGui::SetCursorPosX(
                        ImGui::GetCursorPosX() +
                        std::max(
                            0.0f,
                            ImGui::GetContentRegionAvail().x -
                            ImGui::CalcTextSize(text).x));
                };

                // Header row. Drawn by hand rather than with TableHeadersRow()
                // so that the title of the count column is right-aligned over
                // the values, like they are.
                ImGui::TableNextRow(ImGuiTableRowFlags_Headers);
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted("Element");
                ImGui::TableSetColumnIndex(1);
                right_aligned("Count");
                ImGui::TextUnformatted("Count");

                // The count of the current row, right-aligned in the second
                // column. Clicking it copies the bare value to the clipboard.
                const auto count_cell = [&format_count, &right_aligned](
                    const char* id, const GEO::index_t count) {
                    const std::string text = format_count(count);
                    // The element name disambiguates the ImGui ID in case two
                    // counts happen to be equal.
                    const std::string label = text + "##" + id;
                    const float label_width =
                        ImGui::CalcTextSize(text.c_str()).x;
                    ImGui::TableSetColumnIndex(1);
                    right_aligned(text.c_str());
                    if (ImGui::Selectable(
                        label.c_str(), false, 0, ImVec2(label_width, 0.0f)))
                        ImGui::SetClipboardText(text.c_str());
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("Click to copy");
                };

                // Horizontal offset at which a tree node draws its label: the
                // arrow, plus the padding on both sides of it (see
                // ImGui::TreeNodeBehavior()). The kind rows without a
                // breakdown indent their name by the same amount, so that all
                // the kinds line up whether they have an arrow or not.
                const float label_offset =
                    ImGui::GetFontSize() +
                    2.0f * ImGui::GetStyle().FramePadding.x;

                // A row naming an element kind that has nothing to break down
                // (edges), drawn on the table's header colour so that the
                // kinds stand out from their breakdown.
                const auto element_row = [&count_cell, label_offset](
                    const char* name, const GEO::index_t count) {
                    ImGui::TableNextRow(ImGuiTableRowFlags_Headers);
                    ImGui::TableSetColumnIndex(0);
                    ImGui::SetCursorPosX(
                        ImGui::GetCursorPosX() + label_offset);
                    ImGui::TextUnformatted(name);
                    count_cell(name, count);
                };

                // A row naming an element kind that has a breakdown. The
                // arrow opens and closes the breakdown rows below it; like
                // any tree node it starts collapsed, and remembers its state.
                // @return true when the breakdown is open and must be drawn.
                const auto kind_row = [&count_cell](
                    const char* name, const GEO::index_t count) {
                    ImGui::TableNextRow();
                    // Same background as the plain kind rows, so that all the
                    // kinds read as a group whether they expand or not.
                    ImGui::TableSetBgColor(
                        ImGuiTableBgTarget_RowBg0,
                        ImGui::GetColorU32(ImGuiCol_TableHeaderBg));
                    ImGui::TableSetColumnIndex(0);
                    const bool open = ImGui::TreeNodeEx(
                        name, ImGuiTreeNodeFlags_None, "%s", name);
                    count_cell(name, count);
                    return open;
                };

                // A breakdown row of the element kind above it. Left out when
                // the mesh has none of it. The row inherits the indentation of
                // the tree node its kind row pushed, so no explicit indentation
                // is needed here.
                const auto detail_row = [&count_cell](
                    const char* name, const GEO::index_t count) {
                    if (count == 0)
                        return;
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::TextUnformatted(name);
                    count_cell(name, count);
                };

                if (mesh_.vertices.nb() > 0) {
                    if (kind_row("Vertices", mesh_.vertices.nb())) {
                        detail_row("dimension", mesh_.vertices.dimension());
                        ImGui::TreePop();
                    }
                }
                if (mesh_.edges.nb() > 0)
                    element_row("Edges", mesh_.edges.nb());
                if (mesh_.facets.nb() > 0) {
                    if (kind_row("Facets", mesh_.facets.nb())) {
                        detail_row("tris", element_counts_.triangles);
                        detail_row("quads", element_counts_.quads);
                        detail_row("polygons", element_counts_.polygons);
                        ImGui::TreePop();
                    }
                }
                if (mesh_.cells.nb() > 0) {
                    if (kind_row("Cells", mesh_.cells.nb())) {
                        // GeoGram's cell types, in the enum's order: these are
                        // also the names the enum values are usually called by.
                        static const char* CELL_TYPE_NAMES[] = {
                            "tets", "hexes", "prisms", "pyramids", "connectors"
                        };
                        static_assert(
                            std::size(CELL_TYPE_NAMES) ==
                            GEO::MESH_NB_CELL_TYPES,
                            "one name per GeoGram cell type");
                        for (int type = 0;
                             type < GEO::MESH_NB_CELL_TYPES; ++type)
                            detail_row(
                                CELL_TYPE_NAMES[type],
                                element_counts_.cells[type]);
                        ImGui::TreePop();
                    }
                }

                ImGui::EndTable();
            }
        }

        if (ImGui::CollapsingHeader("Render", ImGuiTreeNodeFlags_DefaultOpen)) {
            const auto s = static_cast<float>(ImGui::scaling());

            /* == Painting mode ==================================================================================== */
            // TEXTURE can only be offered when the mesh has a 2D vector
            // attribute to sample the image with. A mode left over from a
            // reloaded mesh that lost its texture coordinates falls back to
            // SOLID_COLOR.
            const std::string texture_attributes = texture_attribute_names();
            const bool can_texture = !texture_attributes.empty();
            if (painting_mode_ == PaintingMode::TEXTURE && !can_texture)
                painting_mode_ = PaintingMode::SOLID_COLOR;

            // Same layout as Graphite for the properties below: the name on
            // its own line, then the value widget. "Painting" is the title of
            // the block, so it is rendered as separator text in a dark blue
            // accent to stand out from the property labels.
            static const char* PAINTING_MODE_NAMES[] = {
                "SOLID_COLOR", "ATTRIBUTE", "TEXTURE"
            };
            const int nb_painting_modes = can_texture ? 3 : 2;
            int painting_mode = static_cast<int>(painting_mode_);
            ImGui::PushStyleColor(
                ImGuiCol_Text,
                ImVec4(
                    title_color_.x,
                    title_color_.y,
                    title_color_.z,
                    title_color_.w));
            ImGui::SeparatorText("Painting");
            ImGui::PopStyleColor();
            ImGui::PushItemWidth(-10.0f);
            if (ImGui::Combo(
                    "##painting", &painting_mode,
                    PAINTING_MODE_NAMES, nb_painting_modes)) {
                painting_mode_ = static_cast<PaintingMode>(painting_mode);
                // Select the first texture-coordinate attribute the first time
                // the mode is chosen, as Graphite does.
                if (painting_mode_ == PaintingMode::TEXTURE &&
                    tex_coord_full_name_.empty()) {
                    set_tex_coords(texture_attributes.substr(
                        0, texture_attributes.find(';')));
                }
            }
            ImGui::PopItemWidth();
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(
                    "Painting mode: SOLID_COLOR, ATTRIBUTE or TEXTURE "
                    "(same modes as Graphite).");

            if (painting_mode_ == PaintingMode::ATTRIBUTE) {
                if (attribute_min_ == 0.0f && attribute_max_ == 0.0f)
                    autorange();

                if (ImGui::Button(
                    (vertices_point_attribute_name_ + "##Attribute").c_str(),
                    ImVec2(-1, 0)))
                    ImGui::OpenPopup("##Attributes");

                if (ImGui::BeginPopup("##Attributes")) {
                    std::vector<std::string> attributes;
                    GEO::String::split_string(attribute_names(), ';', attributes);

                    for (const auto& attribute : attributes) {
                        if (ImGui::Button(attribute.c_str())) {
                            set_attribute(attribute);
                            ImGui::CloseCurrentPopup();
                        }
                    }
                    ImGui::EndPopup();
                }

                ImGui::InputFloat("min", &attribute_min_);
                ImGui::InputFloat("max", &attribute_max_);
                if (ImGui::Button("autorange", ImVec2(-1, 0)))
                    autorange();

                // The preview shows the colormap as the attribute samples it:
                // its nearest-filtered variant when "smooth" is unchecked, and
                // back to front when "flip" is (the image is not modified, its
                // texture coordinates are reversed). The popup below keeps
                // showing the colormaps in their canonical orientation, so they
                // stay identifiable while selecting one.
                const ImVec2 preview_uv0(
                    flip_colormap_ ? 1.0f : 0.0f, 0.0f);
                const ImVec2 preview_uv1(
                    flip_colormap_ ? 0.0f : 1.0f, 1.0f);
                if (ImGui::ImageButton(
                    "choose_colormap",
                    static_cast<ImTextureID>(colormap_texture()),
                    ImVec2(0.95f * ImGui::GetContentRegionAvail().x, 8.0f*s),
                    preview_uv0, preview_uv1)
                    ) {
                    ImGui::OpenPopup("##Colormap");
                }
                if (ImGui::BeginPopup("##Colormap")) {
                    const std::vector<ColormapInfo>& available_colormaps =
                        colormaps();
                    for (GEO::index_t i = 0;
                         i < available_colormaps.size();
                         ++i) {
                        if (ImGui::ImageButton(
                            available_colormaps[i].name.c_str(),
			                static_cast<ImTextureID>(colormap_texture(i)),
                            ImVec2(100.0f*s, 8.0f*s))
                            ) {
                            current_colormap_index_   = i;
                            ImGui::CloseCurrentPopup();
                        }
                    }
                    ImGui::EndPopup();
                }

                // The text labels are replaced by icons, as in Graphite: the
                // checkbox box next to the glyph is the toggle, and the glyph
                // shows what the current state does. "smooth" is a smooth
                // gradient (chart area) or flat color bands (chart bars);
                // "flip" points along the direction the colormap is sampled in.
                ImGui::Checkbox(
                    (GEO::icon_UTF8(
                        smooth_colormap_ ? "chart-area" : "chart-bar") +
                     "##smooth").c_str(),
                    &smooth_colormap_);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip(
                        "Smooth: interpolate the colormap between its "
                        "entries. Unchecked, every entry is a flat color "
                        "band, with hard boundaries between the bands.");
                ImGui::SameLine();
                ImGui::Checkbox(
                    (GEO::icon_UTF8(
                        flip_colormap_ ? "left-right" : "right-left") +
                     "##flip").c_str(),
                    &flip_colormap_);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip(
                        "Flip: swap the ends of the colormap: the minimum of "
                        "the range then takes its last color and the maximum "
                        "its first one.");
            }
            else if (painting_mode_ == PaintingMode::TEXTURE) {
                // Layout copied from Graphite's shader properties: the
                // tex_image text field with a folder button on its right, then
                // the tex_coords combo and the tex_repeat number.
                const std::string dialog_label =
                    "##TextureImageDialog_" + unique_id_;

                ImGui::TextUnformatted("tex_image");
                const std::string folder_icon = GEO::icon_UTF8("folder-open");
                const float folder_width =
                    ImGui::CalcTextSize(folder_icon.c_str()).x +
                    2.0f * ImGui::GetStyle().FramePadding.x;
                std::string image_filename = texture_filename_;
                ImGui::PushItemWidth(
                    ImGui::GetContentRegionAvail().x - folder_width -
                    ImGui::GetStyle().ItemSpacing.x);
                if (ImGui::InputText("##tex_image", &image_filename)) {
                    if (image_filename != texture_filename_) {
                        // The file may have been edited on disk: drop the
                        // cached texture so that it is uploaded again.
                        const auto cached = texture_cache.find(image_filename);
                        if (cached != texture_cache.end()) {
                            if (cached->second != 0)
                                glDeleteTextures(1, &cached->second);
                            texture_cache.erase(cached);
                        }
                        texture_filename_ = image_filename;
                    }
                }
                ImGui::PopItemWidth();
                ImGui::SameLine();
                if (ImGui::Button(
                        (folder_icon + "##tex_image_browse").c_str())) {
                    open_texture_file_dialog(
                        dialog_label.c_str(),
                        "png;jpg;jpeg;bmp;tga;xpm",
                        texture_filename_.c_str());
                }

                char selected_filename[GEO::geo_imgui_string_length];
                if (draw_file_dialog(
                        dialog_label.c_str(), selected_filename,
                        sizeof(selected_filename))) {
                    // Selecting a file again reloads it, so that an image
                    // edited on disk is uploaded again. The UI is drawn after
                    // the scene, so no draw is using the old texture now.
                    const auto cached = texture_cache.find(selected_filename);
                    if (cached != texture_cache.end()) {
                        if (cached->second != 0)
                            glDeleteTextures(1, &cached->second);
                        texture_cache.erase(cached);
                    }
                    texture_filename_ = selected_filename;
                }

                // The texture-coordinate, scale and repeat controls only make
                // sense once an image has been chosen: until then only the
                // image picker is shown.
                if (!texture_filename_.empty()) {
                    ImGui::TextUnformatted("tex_coords");
                    std::vector<std::string> attributes;
                    GEO::String::split_string(
                        texture_attributes, ';', attributes);
                    int selected = 0;
                    bool found = false;
                    std::string items;
                    for (std::size_t i = 0; i < attributes.size(); ++i) {
                        items += attributes[i];
                        items.push_back('\0');
                        if (attributes[i] == tex_coord_full_name_) {
                            selected = static_cast<int>(i);
                            found = true;
                        }
                    }
                    // ImGui expects the item list to end with a double '\0'; an
                    // empty list is a single '\0' plus the string terminator.
                    if (attributes.empty())
                        items.push_back('\0');
                    ImGui::PushItemWidth(-10.0f);
                    if (ImGui::Combo(
                            "##tex_coords", &selected, items.c_str())) {
                        set_tex_coords(attributes[selected]);
                    }
                    ImGui::PopItemWidth();
                    // As Graphite does, fall back to the first attribute when
                    // the selected one is not in the mesh any more.
                    if (!found && !attributes.empty())
                        set_tex_coords(attributes[0]);

                    // Independent U/V scale of the parameterization, applied
                    // by pre-multiplying the texture coordinates (MeshGfx only
                    // has a single repeat factor). Like the element sections
                    // below, a checkbox enables the feature and indents its
                    // controls; while it is off, U and V are used as they are.
                    ImGui::Checkbox("uv_scale", &uv_scale_enabled_);
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip(
                            "Enable the U/V scale of the texture coordinates.");
                    if (uv_scale_enabled_) {
                        ImGui::Indent();
                        const float uv_scale_width =
                            0.5f * (ImGui::GetContentRegionAvail().x -
                                    ImGui::GetStyle().ItemSpacing.x);
                        ImGui::PushItemWidth(uv_scale_width);
                        const bool u_changed = ImGui::DragFloat(
                            "##uv_scale_u", &uv_scale_u_,
                            0.001f, 0.001f, 1000.0f, "u %.3f");
                        if (ImGui::IsItemHovered())
                            ImGui::SetTooltip(
                                "Scale applied to the U coordinate.");
                        ImGui::SameLine();
                        const bool v_changed = ImGui::DragFloat(
                            "##uv_scale_v", &uv_scale_v_,
                            0.001f, 0.001f, 1000.0f, "v %.3f");
                        if (ImGui::IsItemHovered())
                            ImGui::SetTooltip(
                                "Scale applied to the V coordinate.");
                        ImGui::PopItemWidth();
                        if (u_changed)
                            uv_scale_u_ = std::max(uv_scale_u_, 0.01f);
                        if (v_changed)
                            uv_scale_v_ = std::max(uv_scale_v_, 0.01f);
                        if (ImGui::Checkbox(
                                "uniform##uv_scale", &uniform_uv_scale_)) {
                            // Locking adopts U as the reference; unlocking
                            // leaves the two scales independent.
                            if (uniform_uv_scale_)
                                uv_scale_v_ = uv_scale_u_;
                        }
                        if (ImGui::IsItemHovered())
                            ImGui::SetTooltip(
                                "Scale U and V by the same factor.");
                        if (uniform_uv_scale_) {
                            if (u_changed)
                                uv_scale_v_ = uv_scale_u_;
                            else if (v_changed)
                                uv_scale_u_ = uv_scale_v_;
                        }
                        ImGui::Unindent();
                    }

                    // Texture repeat: same checkbox + indented controls
                    // pattern. While it is off, the image is sampled once
                    // (repeat of 1).
                    ImGui::Checkbox("tex_repeat", &tex_repeat_enabled_);
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip(
                            "Enable the texture repeat factor.");
                    if (tex_repeat_enabled_) {
                        ImGui::Indent();
                        ImGui::PushItemWidth(-10.0f);
                        ImGui::InputInt("##tex_repeat", &tex_repeat_);
                        ImGui::PopItemWidth();
                        tex_repeat_ = std::max(tex_repeat_, 1);
                        ImGui::Unindent();
                    }
                }
            }

            ImGui::Separator();
            ImGui::PushStyleColor(
                ImGuiCol_Text,
                ImVec4(
                    title_color_.x,
                    title_color_.y,
                    title_color_.z,
                    title_color_.w));
            ImGui::SeparatorText("Elements");
            ImGui::PopStyleColor();

            /* == Vertices ========================================================================================= */
            ImGui::Checkbox("##VertOnOff", &show_vertices_);
            ImGui::SameLine();
            ImGui::ColorEdit3WithPalette("Vert.", vertices_color_.data());

            if (show_vertices_) {
                ImGui::Indent();

                // The size is an absolute (model-space) value: markers are
                // discs of this diameter in the model, so their on-screen
                // size follows the camera distance (near large, far small).
                // The range goes up to the model's bounding-box diagonal and
                // the drag speed is scaled accordingly, so both coarse and
                // fine adjustments are convenient.
                const float diag = bbox_diagonal();
                ImGui::DragFloat(
                    "sz.##vertices", &vertices_size_,
                    diag * 0.0001f,
                    0.0f, diag > 0.0f ? diag : 1.0f, "%.6g");
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip(
                        "Marker diameter in model coordinates (absolute "
                        "size). The on-screen size therefore varies with "
                        "the camera distance.");
                ImGui::SliderFloat("trsp.##vertices", &vertices_transparency_, 0.0f, 1.0f, "%.2f");

                ImGui::Unindent();
            }

            /* == Edges (explicit mesh.edges) ========================================================================= */
            if (mesh_.edges.nb() != 0) {
                ImGui::Checkbox("##EdgeOnOff", &show_edges_);
                ImGui::SameLine();
                ImGui::ColorEdit3WithPalette("Edge.", edges_color_.data());
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip(
                        "Explicit 1-D edges stored in the mesh (mesh.edges), "
                        "e.g. feature edges. Independent from the facet and "
                        "cell wireframes.");
                if (show_edges_)
                    ImGui::SliderFloat("wid.##edges", &edges_width_, 0.1f, 2.0f, "%.1f");
            }

            /* == Facets =========================================================================================== */
            if (mesh_.facets.nb() != 0) {
                ImGui::Checkbox("##SurfOnOff", &show_surface_);
                ImGui::SameLine();
                ImGui::ColorEdit3WithPalette("Surf.", surface_color_.data());

                if (show_surface_) {
                    ImGui::Indent();

                    ImGui::Checkbox("##SidesOnOff", &show_surface_sides_);
                    ImGui::SameLine();
                    ImGui::ColorEdit3WithPalette("2sided", surface_color_2_.data());

                    ImGui::SliderFloat("trsp.##surface", &surface_transparency_, 0.0f, 1.0f, "%.2f");

                    ImGui::Checkbox("##MeshOnOff", &show_surface_mesh_);
                    ImGui::SameLine();
                    ImGui::ColorEdit3WithPalette("mesh", surface_mesh_color_.data());
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip(
                            "Wireframe of the surface facets "
                            "(the edges of mesh.facets).");
                    if (show_surface_mesh_)
                        ImGui::SliderFloat("wid.##mesh", &surface_mesh_width_, 0.1f, 2.0f, "%.1f");

                    // A closed surface has no border edge, so the border pass
                    // has nothing to draw: its controls are hidden then (see
                    // compute_surface_has_borders()).
                    if (surface_has_borders_) {
                        ImGui::Checkbox("##BordersOnOff", &show_surface_borders_);
                        ImGui::SameLine();
                        ImGui::ColorEdit3WithPalette(
                            "borders", surface_borders_color_.data());
                        if (show_surface_borders_)
                            ImGui::SliderFloat(
                                "wid.##borders", &surface_borders_width_,
                                0.1f, 2.0f, "%.1f");
                    }

                    ImGui::Unindent();
                }
            }

            /* == Cells ============================================================================================ */
            if (mesh_.cells.nb() != 0) {
                ImGui::Checkbox("##VolumeOnOff", &show_volume_);
                ImGui::SameLine();
                ImGui::ColorEdit3WithPalette("Volume", volume_color_.data());

                if (show_volume_) {
                    ImGui::Indent();

                    ImGui::Checkbox("##CellMeshOnOff", &show_volume_mesh_);
                    ImGui::SameLine();
                    ImGui::ColorEdit3WithPalette("cell mesh", volume_mesh_color_.data());
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip(
                            "Wireframe of the volume cells "
                            "(the edges of mesh.cells).");
                    if (show_volume_mesh_)
                        ImGui::SliderFloat("wid.##cellmesh", &volume_mesh_width_, 0.1f, 2.0f, "%.1f");

                    ImGui::SliderFloat("shrk.", &cells_shrink_, 0.0f, 1.0f, "%.2f");
                    if (!mesh_.cells.are_simplices()) {
                        ImGui::Checkbox("colored cells", &show_colored_cells_);
                        ImGui::Checkbox("hexes", &show_hexes_);
                    }

                    ImGui::Unindent();
                }
            }
        }

        ImGui::PopID();
    }

    void MeshObject::draw_scene(
        const bool lighting
        ) {
        if (mesh_gfx_.mesh() == nullptr)
            return;

        // Invalid data would make the drawing passes read outside the mesh (see
        // check_validity()): the object stays out of the scene until it is
        // repaired. The application skips it too (see
        // GeoBoxApplication::draw_scene()).
        if (!data_valid_)
            return;

        // An operation may have changed the mesh since the last frame (see
        // mesh_changed()).
        update_mesh_caches();

        mesh_gfx_.set_lighting(lighting);

        // Choose how the elements are colored, as Graphite's painting mode
        // does: a solid color, a scalar attribute through a colormap, or an
        // image through texture coordinates (see PaintingMode).
        switch (painting_mode_) {
        case PaintingMode::SOLID_COLOR:
            mesh_gfx_.unset_scalar_attribute();
            break;

        case PaintingMode::ATTRIBUTE:
            mesh_gfx_.set_scalar_attribute(
                attribute_subelements_,
                colormap_attribute_name(),
                colormap_mapping_min(),
                colormap_mapping_max(),
                colormap_texture(),
                1);
            break;

        case PaintingMode::TEXTURE: {
            // A texture image is sampled through a 2D texture-coordinate
            // attribute (e.g. "facet_corners.tex_coord" loaded from an OBJ
            // file), exactly as Graphite's TEXTURE painting mode does.
            const GLuint image_texture = texture();
            const bool ready =
                image_texture != 0 &&
                tex_coord_subelements_ != GEO::MESH_NONE &&
                attribute_dimension(
                    tex_coord_subelements_, tex_coord_attribute_name_) == 2;
            if (ready) {
                mesh_gfx_.set_texturing(
                    tex_coord_subelements_,
                    tex_coord_attribute_name_for_drawing(),
                    image_texture, 2, texture_repeat());
            }
            else
                mesh_gfx_.unset_scalar_attribute();
            break;
        }
        }

        // Transparent texels are discarded instead of being drawn black: the
        // "transparent" colormap encodes the hidden values in its alpha
        // channel, and a texture image may have transparent pixels too (as
        // Graphite does for its TEXTURE painting mode). The state is global to
        // GLUP, so it is restored after the draws.
        const bool discard_transparent_fragments =
            (painting_mode_ == PaintingMode::ATTRIBUTE &&
             colormap_is_transparent()) ||
            (painting_mode_ == PaintingMode::TEXTURE &&
             surface_transparency_ == 0.0f);
        GLboolean saved_alpha_discard = GL_FALSE;
        GLUPfloat saved_alpha_threshold = 0.0f;
        if (discard_transparent_fragments) {
            saved_alpha_discard = glupIsEnabled(GLUP_ALPHA_DISCARD);
            saved_alpha_threshold = glupGetAlphaThreshold();
            glupEnable(GLUP_ALPHA_DISCARD);
            glupSetAlphaThreshold(0.05f);
        }

        // Opaque geometry first, then the points on top: transparent points do
        // not write depth, so drawing them first would let the surface occlude
        // them; drawing them last keeps them visible on the surface.
        draw_surface();
        draw_edges();
        draw_volume(lighting);
        draw_points();

        if (discard_transparent_fragments) {
            if (!saved_alpha_discard)
                glupDisable(GLUP_ALPHA_DISCARD);
            glupSetAlphaThreshold(saved_alpha_threshold);
        }
    }

    void MeshObject::reload(
        ) {
        if (GEO::Mesh mesh;
            mesh.load(filepath_)) {
            mesh_.copy(mesh);

            // The new mesh has its own connectivity, attributes and element
            // counts, so everything the object derived from the previous one is
            // stale: mesh_changed() invalidates it (the displayed attribute may
            // also be gone from the file, which the drawing tests for before
            // sampling it, see displayed_attribute_is_defined()). The reload is
            // an explicit, rare operation, so the caches are refreshed right
            // away instead of on the next draw, leaving the object consistent
            // as soon as reload() returns.
            mesh_changed();
            update_mesh_caches();
        }
    }

    void MeshObject::get_bbox(
        double* xyzmin,
        double* xyzmax
        ) const {
        if (mesh_.vertices.dimension() == 2) {
            for(GEO::coord_index_t c = 0; c < 2; c++) {
                xyzmin[c] = GEO::Numeric::max_float64();
                xyzmax[c] = GEO::Numeric::min_float64();
            }
            xyzmin[2] = 0;
            xyzmax[2] = 0;
            for(const GEO::vec2& p: mesh_.vertices.points<2>()) {
                for(GEO::coord_index_t c = 0; c < 2; c++) {
                    xyzmin[c] = std::min(xyzmin[c], p[c]);
                    xyzmax[c] = std::max(xyzmax[c], p[c]);
                }
            }
        }
        else
            GEO::get_bbox(mesh_, xyzmin, xyzmax);
    }

    float MeshObject::bbox_diagonal(
        ) const {
        if (bbox_diag_ < 0.0f) {
            double xyzmin[3] = {0.0, 0.0, 0.0};
            double xyzmax[3] = {0.0, 0.0, 0.0};
            get_bbox(xyzmin, xyzmax);
            double d2 = 0.0;
            for (GEO::coord_index_t c = 0; c < 3; ++c) {
                const double d = xyzmax[c] - xyzmin[c];
                d2 += d * d;
            }
            bbox_diag_ = static_cast<float>(std::sqrt(d2));
        }
        return bbox_diag_;
    }

    bool MeshObject::check_validity(
        ) {
        const GEO::index_t VERTICES_NB = mesh_.vertices.nb();
        const GEO::index_t DIM = mesh_.vertices.dimension();

        // The elements to remove are recorded while the mesh is scanned: the
        // dialog lets the user decide later (see draw_validity_dialog()), and
        // repairing then only applies the vectors below.
        invalid_counts_ = InvalidElementCounts();
        vertices_to_delete_.assign(mesh_.vertices.nb(), 0);
        edges_to_delete_.assign(mesh_.edges.nb(), 0);
        facets_to_delete_.assign(mesh_.facets.nb(), 0);
        cells_to_delete_.assign(mesh_.cells.nb(), 0);

        // Vertices whose coordinates are not finite: they poison every
        // computation that reads them (bounding box, normals, ...).
        for (const auto& v : mesh_.vertices) {
            const auto p = mesh_.vertices.point_ptr(v);
            for (GEO::index_t d = 0; d < DIM; ++d) {
                if (std::isnan(p[d]) || std::isinf(p[d])) {
                    ++invalid_counts_.vertices;
                    vertices_to_delete_[v] = 1;
                    break;
                }
            }
        }

        // Elements that reference a vertex the mesh does not have: the drawing
        // passes would read outside the vertex array.
        for (const auto& e : mesh_.edges) {
            if (mesh_.edges.vertex(e, 0) >= VERTICES_NB ||
                mesh_.edges.vertex(e, 1) >= VERTICES_NB
                ) {
                ++invalid_counts_.edges;
                edges_to_delete_[e] = 1;
            }
        }

        for (const auto& f : mesh_.facets) {
            for (GEO::index_t lv = 0, lv_end = mesh_.facets.nb_vertices(f);
                 lv < lv_end; ++lv) {
                if (mesh_.facets.vertex(f, lv) >= VERTICES_NB) {
                    ++invalid_counts_.facets;
                    facets_to_delete_[f] = 1;
                    break;
                }
            }
        }

        for (const auto& c : mesh_.cells) {
            for (GEO::index_t lv = 0, lv_end = mesh_.cells.nb_vertices(c);
                 lv < lv_end; ++lv) {
                if (mesh_.cells.vertex(c, lv) >= VERTICES_NB) {
                    ++invalid_counts_.cells;
                    cells_to_delete_[c] = 1;
                    break;
                }
            }
        }

        // Invalid data cannot be handed to the drawing passes at all: they
        // would read outside the mesh (which crashes the application), so the
        // object stops being drawn until the mesh is repaired (see
        // renderable() and draw_scene()).
        data_valid_ = !invalid_counts_.any();

        // The warning is due for as long as the invalid data is there: it can
        // only be dismissed by keeping the mesh as it is or by repairing it
        // (see draw_validity_dialog()).
        validity_dialog_pending_ = !data_valid_;

        if (validity_dialog_pending_) {
            LOG::WARN(
                "Mesh `{}` is invalid: {} invalid vertices, {} edges, "
                "{} facets, {} cells.",
                name_,
                invalid_counts_.vertices,
                invalid_counts_.edges,
                invalid_counts_.facets,
                invalid_counts_.cells);
        }

        return !validity_dialog_pending_;
    }

    void MeshObject::draw_validity_dialog(
        ) {
        if (!validity_dialog_pending_)
            return;

        // The popup is opened again on every frame while the invalid data was
        // not dealt with: it has no close button, and the Escape key would
        // otherwise dismiss it, which must not be possible -- the user has to
        // choose between keeping the mesh as it is and repairing it. Its name
        // carries the object id, so that two invalid objects do not share one.
        const std::string popup_name = "Invalid mesh##" + unique_id_;
        ImGui::OpenPopup(popup_name.c_str());
        if (!ImGui::BeginPopupModal(
                popup_name.c_str(), nullptr,
                ImGuiWindowFlags_AlwaysAutoResize |
                ImGuiWindowFlags_NoSavedSettings |
                ImGuiWindowFlags_NoMove)) {
            return;
        }

        ImGui::PushStyleColor(
            ImGuiCol_Text,
            ImVec4(
                title_color_.x, title_color_.y, title_color_.z, title_color_.w));
        ImGui::Text("The mesh of \"%s\"", name_.c_str());
        ImGui::PopStyleColor();
        ImGui::TextUnformatted("cannot be rendered as it is:");

        ImGui::Spacing();
        // One entry per kind of invalid element, in the plural of its actual
        // count; the kinds the mesh has none of are left out.
        const auto invalid_kind = [](
            const GEO::index_t count,
            const char* singular,
            const char* plural,
            const char* reason
            ) {
            ImGui::BulletText(
                "%u invalid %s (%s)",
                static_cast<unsigned>(count), count == 1 ? singular : plural, reason);
        };
        if (invalid_counts_.vertices != 0) {
            invalid_kind(
                invalid_counts_.vertices, "vertex", "vertices",
                "NaN/non-finite coordinates");
        }
        if (invalid_counts_.edges != 0) {
            invalid_kind(
                invalid_counts_.edges, "edge", "edges", "missing vertices");
        }
        if (invalid_counts_.facets != 0) {
            invalid_kind(
                invalid_counts_.facets, "facet", "facets", "missing vertices");
        }
        if (invalid_counts_.cells != 0) {
            invalid_kind(
                invalid_counts_.cells, "cell", "cells", "missing vertices");
        }

        ImGui::Spacing();
        ImGui::Separator();

        // What the repair removes (see
        // repair_invalid_vertices_and_elements()). They only apply when the mesh
        // is repaired: skipping it keeps the mesh as it was loaded.
        ImGui::Checkbox("remove invalid vertices", &remove_invalid_vertices_);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip(
                "Remove the vertices whose coordinates are NaN/not finite.\n"
                "Used when the mesh is repaired.");
        }
        ImGui::Checkbox("remove invalid elements", &remove_invalid_elements_);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip(
                "Remove the edges, facets and cells that reference\n"
                "missing vertices. Used when the mesh is repaired.");
        }

        ImGui::Separator();

        // The button says what closing the dialog does: repairing the mesh
        // removes what the checkboxes ask for, skipping it keeps the mesh as it
        // is.
        const char* const action = remove_invalid_elements_
            ? "Repair this mesh"
            : "Skip this mesh";
        if (ImGui::Button(action, ImVec2(-1.0f, 0.0f))) {
            if (remove_invalid_elements_)
                repair_invalid_vertices_and_elements();
            validity_dialog_pending_ = false;
            ImGui::CloseCurrentPopup();
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip(
                remove_invalid_elements_
                    ? "Remove the invalid elements (and the invalid vertices if\n"
                      "selected), then display the mesh."
                    : "Keep the mesh as it was loaded, and leave it out of the\n"
                      "3D view. Check \"remove invalid elements\" to repair it\n"
                      "instead.");
        }

        ImGui::EndPopup();
    }

    void MeshObject::repair_invalid_vertices_and_elements(
        ) {
        // Elements first: deleting vertices re-indexes the elements that
        // reference them (see GEO::MeshVertices::delete_elements()).
        if (remove_invalid_elements_) {
            if (invalid_counts_.edges != 0)
                mesh_.edges.delete_elements(edges_to_delete_, false);
            if (invalid_counts_.facets != 0)
                mesh_.facets.delete_elements(facets_to_delete_, false);
            if (invalid_counts_.cells != 0)
                mesh_.cells.delete_elements(cells_to_delete_, false);
        }
        if (remove_invalid_vertices_ && invalid_counts_.vertices != 0)
            mesh_.vertices.delete_elements(vertices_to_delete_, false);

        // Removing a vertex leaves NO_VERTEX in the elements that referenced it,
        // so the mesh may still be invalid: the check tells, and keeps the
        // dialog (with the new counts) open if it is.
        mesh_changed();
        check_validity();
    }

    void MeshObject::draw_points(
        ) {
        if (!show_vertices_ || mesh_.vertices.nb() == 0 ||
            vertices_size_ <= 0.0f)
            return;

        // Vertex markers have an absolute (model-space) diameter
        // (vertices_size_), so their on-screen size must follow the camera
        // distance (near large, far small). MeshGfx::draw_vertices() cannot
        // express that: GLUP points rasterize with a single point-sprite size
        // for the whole draw call, i.e. a constant size on screen. Instead,
        // each vertex is drawn as a camera-facing disc of the requested world
        // diameter using GLUP's sphere impostor primitive (per-vertex radius
        // in the w component of glupVertex4d), which gives the round marker
        // silhouette with correct perspective scaling.
        const bool blended = vertices_transparency_ != 0.0f;
        if (blended) {
            glDepthMask(GL_FALSE);
            glEnable(GL_BLEND);
            glBlendEquation(GL_FUNC_ADD);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        }

        // Render flat markers: without shading, the impostor spheres look
        // like round discs. Restore the previous lighting state afterwards
        // so the next object's surface pass is unaffected.
        const bool lighting = glupIsEnabled(GLUP_LIGHTING) != GL_FALSE;
        glupDisable(GLUP_LIGHTING);

        // Vertices sit exactly on the surface, so the default GL_LESS depth
        // test would cull them at equal depth; use an inclusive test so the
        // markers stay visible on top of the model.
        glDepthFunc(GL_LEQUAL);

        const double radius = 0.5 * static_cast<double>(vertices_size_);

        // When a painting mode colors the vertices, color each marker through
        // GLUP's texturing (the same state MeshGfx::begin_attributes() sets
        // up): a 1-D colormap for scalar attributes, or the image for texture
        // coordinates. Otherwise use the plain vertex color.
        GEO::ReadOnlyScalarAttributeAdapter scalar_attribute;
        GEO::ReadOnlyScalarAttributeAdapter tex_coord_attributes[2];
        int marker_texture_dim = 0;
        GLuint marker_texture = 0;
        int marker_repeat = 1;

        if (painting_mode_ == PaintingMode::ATTRIBUTE &&
            attribute_subelements_ == GEO::MESH_VERTICES) {
            scalar_attribute.bind_if_is_defined(
                mesh_.vertices.attributes(), colormap_attribute_name()
            );
            if (scalar_attribute.is_bound()) {
                marker_texture_dim = 1;
                marker_texture = colormap_texture();
            }
        }
        else if (painting_mode_ == PaintingMode::TEXTURE &&
                 tex_coord_subelements_ == GEO::MESH_VERTICES &&
                 attribute_dimension(
                     tex_coord_subelements_, tex_coord_attribute_name_) == 2) {
            const std::string tex_coord_name =
                tex_coord_attribute_name_for_drawing();
            for (GEO::index_t c = 0; c < 2; ++c) {
                tex_coord_attributes[c].bind_if_is_defined(
                    mesh_.vertices.attributes(),
                    tex_coord_name + "[" + std::to_string(c) + "]");
            }
            if (tex_coord_attributes[0].is_bound() &&
                tex_coord_attributes[1].is_bound()) {
                marker_texture_dim = 2;
                marker_texture = texture();
                marker_repeat = static_cast<int>(texture_repeat());
            }
        }

        const bool textured = marker_texture_dim != 0 && marker_texture != 0;
        if (textured) {
            glupEnable(GLUP_TEXTURING);
            glupTextureMode(GLUP_TEXTURE_REPLACE);
            switch (marker_texture_dim) {
            case 1:
                glupTextureType(GLUP_TEXTURE_1D);
                glActiveTexture(GL_TEXTURE0 + GLUP_TEXTURE_1D_UNIT);
                glBindTexture(GLUP_TEXTURE_1D_TARGET, marker_texture);
                GEO::glupMapTexCoords1d(
                    colormap_mapping_min(),
                    colormap_mapping_max(),
                    1
                );
                break;
            case 2:
                glupTextureType(GLUP_TEXTURE_2D);
                glActiveTexture(GL_TEXTURE0 + GLUP_TEXTURE_2D_UNIT);
                glBindTexture(GLUP_TEXTURE_2D_TARGET, marker_texture);
                break;
            default:
                break;
            }
            if (marker_texture_dim > 1) {
                glupMatrixMode(GLUP_TEXTURE_MATRIX);
                glupLoadIdentity();
                if (marker_repeat > 1) {
                    glupScalef(
                        GLUPfloat(marker_repeat),
                        GLUPfloat(marker_repeat),
                        GLUPfloat(marker_repeat));
                }
                glupMatrixMode(GLUP_MODELVIEW_MATRIX);
            }
            glupSetColor3f(GLUP_FRONT_AND_BACK_COLOR, 1.0f, 1.0f, 1.0f);
        }
        else {
            const float rgba[4] = {
                vertices_color_.x,
                vertices_color_.y,
                vertices_color_.z,
                1.0f - vertices_transparency_
            };
            glupSetColor4fv(GLUP_FRONT_COLOR, rgba);
        }

        glupBegin(GLUP_SPHERES);
        {
            const bool single_precision = mesh_.vertices.single_precision();
            const GEO::coord_index_t dim = mesh_.vertices.dimension();
            const GEO::index_t nb = mesh_.vertices.nb();
            for (GEO::index_t v = 0; v < nb; ++v) {
                double p[3] = {0.0, 0.0, 0.0};
                if (single_precision) {
                    const float* pp =
                        mesh_.vertices.single_precision_point_ptr(v);
                    for (GEO::coord_index_t c = 0; c < dim && c < 3; ++c)
                        p[c] = static_cast<double>(pp[c]);
                }
                else {
                    const double* pp = mesh_.vertices.point_ptr(v);
                    for (GEO::coord_index_t c = 0; c < dim && c < 3; ++c)
                        p[c] = pp[c];
                }
                if (textured) {
                    switch (marker_texture_dim) {
                    case 1:
                        glupTexCoord1d(scalar_attribute[v]);
                        break;
                    case 2:
                        glupTexCoord2d(
                            tex_coord_attributes[0][v],
                            tex_coord_attributes[1][v]);
                        break;
                    default:
                        break;
                    }
                }
                glupVertex4d(p[0], p[1], p[2], radius);
            }
        }
        glupEnd();

        if (textured) {
            glupDisable(GLUP_TEXTURING);
            // Reset the texture matrix, as MeshGfx::end_attributes() does.
            glupMatrixMode(GLUP_TEXTURE_MATRIX);
            glupLoadIdentity();
            glupMatrixMode(GLUP_MODELVIEW_MATRIX);
        }
        if (lighting)
            glupEnable(GLUP_LIGHTING);

        glDepthFunc(GL_LESS);

        if (blended) {
            glDisable(GL_BLEND);
            glDepthMask(GL_TRUE);
        }
    }

    bool MeshObject::compute_surface_has_borders(
        ) const {
        // MeshGfx::draw_surface_borders() emits, for every facet, the edges
        // whose facet corner has no adjacent facet (see its implementation):
        // the same test tells whether that pass would draw anything at all.
        for (const GEO::index_t f : mesh_.facets) {
            for (const GEO::index_t c : mesh_.facets.corners(f)) {
                if (mesh_.facet_corners.adjacent_facet(c) == GEO::NO_FACET)
                    return true;
            }
        }
        return false;
    }

    void MeshObject::mesh_changed(
        ) {
        // What the object caches about the mesh is stale from now on; the parts
        // that scan it are recomputed lazily (see update_mesh_caches()).
        mesh_caches_dirty_ = true;
        bbox_diag_ = -1.0f;

        // The display copies are built on the attribute stores of the mesh,
        // which the operation may have replaced (see update_display_attribute()
        // and update_display_tex_coords()).
        display_attribute_ = DisplayAttribute();
        display_tex_coords_ = DisplayTexCoords();

        // MeshGfx caches the geometry and the attributes it draws in buffer
        // objects, which it only re-uploads once it is told the mesh changed:
        // without this, the object would keep drawing the mesh as it was
        // before the operation.
        mesh_gfx_.set_mesh(&mesh_);
    }

    void MeshObject::update_mesh_caches(
        ) {
        if (!mesh_caches_dirty_)
            return;
        mesh_caches_dirty_ = false;

        // Both walk the whole mesh, which is why they are not done in
        // mesh_changed() but here, when the caches are actually read.
        surface_has_borders_ = compute_surface_has_borders();
        element_counts_ = count_elements();
    }

    MeshObject::ElementCounts MeshObject::count_elements(
        ) const {
        ElementCounts counts;

        // Facets are classified by arity: GeoGram stores triangles/quads
        // compactly but does not record the type, the number of vertices of
        // the facet is what tells them apart. Facets with more than 4
        // vertices (and the degenerate ones with fewer than 3, which a valid
        // mesh should not have) are reported as polygons.
        for (const GEO::index_t f : mesh_.facets) {
            switch (mesh_.facets.nb_vertices(f)) {
            case 3:
                ++counts.triangles;
                break;
            case 4:
                ++counts.quads;
                break;
            default:
                ++counts.polygons;
                break;
            }
        }

        // Cells are classified by their GeoGram type (tets, hexes, prisms,
        // pyramids, connectors). MeshCells::type() resolves the simplicial
        // storage case to MESH_TET, so it yields the right type for every
        // cell whatever the storage.
        for (const GEO::index_t c : mesh_.cells) {
            const GEO::MeshCellType type = mesh_.cells.type(c);
            if (type < GEO::MESH_NB_CELL_TYPES)
                ++counts.cells[type];
        }

        return counts;
    }

    void MeshObject::draw_surface(
        ) {
        mesh_gfx_.set_mesh_color(0.0, 0.0, 0.0);

        const float alpha = 1.0f - surface_transparency_;
        if (surface_transparency_ != 0.0f) {
            glDepthMask(GL_FALSE);
            glEnable(GL_BLEND);
            glBlendEquation(GL_FUNC_ADD);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        }

        mesh_gfx_.set_surface_color(
            surface_color_.x, surface_color_.y, surface_color_.z, alpha);
        if (show_surface_sides_) {
            mesh_gfx_.set_backface_surface_color(
                surface_color_2_.x, surface_color_2_.y, surface_color_2_.z, alpha);
        }

        mesh_gfx_.set_show_mesh(show_surface_mesh_);
        mesh_gfx_.set_mesh_color(
            surface_mesh_color_.x, surface_mesh_color_.y, surface_mesh_color_.z);
        mesh_gfx_.set_mesh_width(
            static_cast<GEO::index_t>(surface_mesh_width_ * 10.0f));

        if (show_surface_) {
            const float specular_backup = glupGetSpecular();
            glupSetSpecular(0.4f);
            mesh_gfx_.draw_surface();
            glupSetSpecular(specular_backup);

            // Nothing to draw when the surface is closed (see
            // compute_surface_has_borders()).
            if (show_surface_borders_ && surface_has_borders_) {
                mesh_gfx_.set_mesh_color(
                    surface_borders_color_.x,
                    surface_borders_color_.y,
                    surface_borders_color_.z);
                mesh_gfx_.set_mesh_border_width(
                    static_cast<GEO::index_t>(surface_borders_width_ * 10.0f));
                mesh_gfx_.draw_surface_borders();

                // The border pass above leaves MeshGfx's mesh color set to the border
                // color; restore it so later passes (edges, volume wireframe) use the
                // surface mesh color.
                mesh_gfx_.set_mesh_color(
                    surface_mesh_color_.x, surface_mesh_color_.y, surface_mesh_color_.z);
            }
        }

        if (surface_transparency_ != 0.0f) {
            glDisable(GL_BLEND);
            glDepthMask(GL_TRUE);
        }
    }

    void MeshObject::draw_edges(
        ) {
        if (!show_edges_ || mesh_.edges.nb() == 0)
            return;

        // MeshGfx::draw_edges() renders only the explicit mesh.edges store and
        // picks up the color/width from its mesh_color_/mesh_width_ members;
        // borrow them for this pass, then restore them so the facet and cell
        // wireframes keep their own styles.
        mesh_gfx_.set_mesh_color(
            edges_color_.x, edges_color_.y, edges_color_.z);
        mesh_gfx_.set_mesh_width(
            static_cast<GEO::index_t>(edges_width_ * 10.0f));
        mesh_gfx_.draw_edges();
        mesh_gfx_.set_mesh_color(
            surface_mesh_color_.x, surface_mesh_color_.y, surface_mesh_color_.z);
        mesh_gfx_.set_mesh_width(
            static_cast<GEO::index_t>(surface_mesh_width_ * 10.0f));
    }

    void MeshObject::draw_volume(
        const bool lighting
        ) {
        if (show_volume_) {
            if (glupIsEnabled(GLUP_CLIPPING) &&
                glupGetClipMode() == GLUP_CLIP_SLICE_CELLS)
                mesh_gfx_.set_lighting(false);

            mesh_gfx_.set_shrink(static_cast<double>(cells_shrink_));
            mesh_gfx_.set_draw_cells(GEO::MESH_HEX, show_hexes_);
            mesh_gfx_.set_draw_cells(GEO::MESH_CONNECTOR, show_connectors_);

            if(show_colored_cells_)
                mesh_gfx_.set_cells_colors_by_type();
            else
                mesh_gfx_.set_cells_color(
                    volume_color_.x, volume_color_.y, volume_color_.z);

            // The cell wireframe is emitted by GLUP itself (GLUP_DRAW_MESH),
            // whose state MeshGfx derives from its show_mesh_/mesh_color_/
            // mesh_width_ members at the start of draw_volume(); override
            // them for this pass and restore them afterwards, so the cell
            // wireframe is independent from the facet wireframe and from the
            // explicit mesh.edges.
            const bool saved_show_mesh = mesh_gfx_.get_show_mesh();
            mesh_gfx_.set_show_mesh(show_volume_mesh_);
            mesh_gfx_.set_mesh_color(
                volume_mesh_color_.x, volume_mesh_color_.y, volume_mesh_color_.z);
            mesh_gfx_.set_mesh_width(
                static_cast<GEO::index_t>(volume_mesh_width_ * 10.0f));

            // A scalar attribute defined on the cell facets is attached to the
            // half-facets of the cells, so MeshGfx::draw_volume() cannot render
            // it with whole-cell primitives: the facets are drawn one by one
            // instead (see draw_volume_cell_facets_attribute(), GeoGram PR #386).
            if (cell_facets_attribute_active())
                draw_volume_cell_facets_attribute();
            else
                mesh_gfx_.draw_volume();

            mesh_gfx_.set_show_mesh(saved_show_mesh);
            mesh_gfx_.set_mesh_color(
                surface_mesh_color_.x, surface_mesh_color_.y, surface_mesh_color_.z);
            mesh_gfx_.set_mesh_width(
                static_cast<GEO::index_t>(surface_mesh_width_ * 10.0f));

            mesh_gfx_.set_lighting(lighting);
        }
    }

    bool MeshObject::displayed_attribute_is_defined(
        ) {
        if (attribute_subelements_ == GEO::MESH_NONE)
            return false;

        // The name tested is the one draw_scene() hands to MeshGfx and
        // begin_attributes() binds: the display copy while it is up to date, the
        // displayed attribute otherwise (see colormap_attribute_name()).
        const GEO::MeshSubElementsStore& subelements =
            mesh_.get_subelements_by_type(attribute_subelements_);
        return GEO::ReadOnlyScalarAttributeAdapter::is_defined(
            subelements.attributes(), colormap_attribute_name());
    }

    bool MeshObject::cell_facets_attribute_active(
        ) {
        // GeoGram PR #386 also requires MeshGfx's picking mode to be MESH_NONE;
        // GeoBox never picks in the mesh (MeshGfx::set_picking_mode() is not
        // called anywhere in the application), so there is nothing to test here.
        return painting_mode_ == PaintingMode::ATTRIBUTE &&
               attribute_subelements_ == GEO::MESH_CELL_FACETS &&
               // The mesh may have lost the attribute since it was selected
               // (see reload()): the pass then has no value to color the facets
               // with, and drawing the volume through MeshGfx, as when no
               // attribute is displayed, is both faster and the behavior
               // GeoGram has (it drops the display in set_scalar_attribute()).
               displayed_attribute_is_defined();
    }

    double MeshObject::cells_shrink_factor(
        ) const {
        // GLUP shrinks a primitive by moving each of its vertices toward the
        // average of the primitive's vertices (see
        // GLUP::Context::shrink_cells_in_immediate_buffers()), and it skips the
        // shrink entirely when the cells are cut by a slice, which the clip
        // modes below test. The facet passes apply the same shrink to the cells
        // the facets belong to, so they have to skip it in the same cases.
        if (cells_shrink_ == 0.0f)
            return 0.0;
        if (glupIsEnabled(GLUP_CLIPPING) &&
            glupGetClipMode() == GLUP_CLIP_SLICE_CELLS)
            return 0.0;
        return static_cast<double>(cells_shrink_);
    }

    void MeshObject::draw_volume_cell_facets_attribute(
        ) {
        // GeoBox implementation of the GeoGram PR #386 ("render volume-mesh
        // cell_facets attributes") drawing paths, i.e. of what
        // MeshGfx::draw_tets_immediate_attrib() and
        // MeshGfx::draw_hybrid_immediate_attrib() do when the displayed
        // attribute is on the cell facets.
        //
        // The pass cannot be delegated to MeshGfx::draw_volume(): the helpers
        // it is built on (begin_attributes(), draw_vertex(),
        // draw_sequences_if(), draw_volume_vertex_with_attribute()...) are
        // protected members of MeshGfx, and MeshObject owns a MeshGfx instead of
        // deriving from it. The GLUP state that MeshGfx::draw_volume() sets up
        // is therefore reproduced here:
        //   MeshGfx::set_GLUP_parameters() -> GLUP_DRAW_MESH, GLUP_MESH_COLOR,
        //     GLUP_MESH_WIDTH and GLUP_LIGHTING;
        //   MeshGfx::draw_tets()/draw_hybrid() -> the cells color;
        //   MeshGfx::draw_volume() -> glupSetCellsShrink(shrink_), which is
        //     replaced by a per facet vertex shrink (see
        //     draw_volume_facet_vertex()).

        // == MeshGfx::set_GLUP_parameters() ==================================
        if (mesh_gfx_.get_show_mesh())
            glupEnable(GLUP_DRAW_MESH);
        else
            glupDisable(GLUP_DRAW_MESH);

        // The mesh color/width were set by draw_volume() for this pass (they are
        // restored right after), so they are read back from MeshGfx rather than
        // duplicated here.
        float mesh_color[4] = {0.0f, 0.0f, 0.0f, 1.0f};
        mesh_gfx_.get_mesh_color(
            mesh_color[0], mesh_color[1], mesh_color[2], mesh_color[3]);
        glupSetColor4fv(GLUP_MESH_COLOR, mesh_color);
        glupSetMeshWidth(GLUPint(mesh_gfx_.get_mesh_width()));

        if (mesh_gfx_.get_lighting())
            glupEnable(GLUP_LIGHTING);
        else
            glupDisable(GLUP_LIGHTING);

        // Cells color, as MeshGfx::draw_tets()/draw_hybrid() set it from their
        // cells_color_ table before emitting the cells. It is only visible when
        // no attribute is bound, since the GLUP_TEXTURE_REPLACE mode set up by
        // begin_attributes() replaces it.
        glupSetColor4fv(GLUP_FRONT_AND_BACK_COLOR, volume_color_.data());

        // == MeshGfx::draw_volume() ==========================================
        // GLUP shrinks whole-cell primitives only, and the facets are drawn as
        // plain triangles/quads: disable GLUP's shrink and replicate it per
        // facet vertex (see draw_volume_facet_vertex()).
        const GLUPfloat saved_shrink = GLUPfloat(cells_shrink_);
        glupSetCellsShrink(0.0f);

        begin_attributes();

        if (mesh_.cells.are_simplices()) {
            // MeshGfx::draw_tets_immediate_attrib(): all the cells are
            // tetrahedra, so all their facets are triangles.
            if (mesh_gfx_.get_draw_cells(GEO::MESH_TET)) {
                draw_volume_cell_facets(0, mesh_.cells.nb(), GLUP_TRIANGLES);
                draw_volume_cell_facets(0, mesh_.cells.nb(), GLUP_QUADS);
            }
        } else {
            // MeshGfx::draw_hybrid_immediate_attrib(): one pass per cell type,
            // for the types that are displayed and present in the mesh.
            bool has_cells[GEO::MESH_NB_CELL_TYPES] = {};
            for (GEO::index_t c = 0; c < mesh_.cells.nb(); ++c)
                has_cells[mesh_.cells.type(c)] = true;

            for (GEO::index_t type = GEO::MESH_TET;
                 type < GEO::MESH_NB_CELL_TYPES;
                 ++type) {
                if (!mesh_gfx_.get_draw_cells(
                        static_cast<GEO::MeshCellType>(type)) ||
                    !has_cells[type])
                    continue;

                // Equivalent of MeshGfx::draw_sequences_if(): the cells of a
                // given type are drawn by runs of consecutive cells of that
                // type (a hybrid mesh usually stores its types in blocks).
                GEO::index_t c = 0;
                while (c < mesh_.cells.nb()) {
                    while (c < mesh_.cells.nb() &&
                           GEO::index_t(mesh_.cells.type(c)) != type)
                        ++c;
                    const GEO::index_t begin_c = c;
                    while (c < mesh_.cells.nb() &&
                           GEO::index_t(mesh_.cells.type(c)) == type)
                        ++c;
                    if (begin_c == c)
                        break;
                    draw_volume_cell_facets(begin_c, c, GLUP_TRIANGLES);
                    draw_volume_cell_facets(begin_c, c, GLUP_QUADS);
                }
            }
        }

        end_attributes();

        glupSetCellsShrink(saved_shrink);
    }

    void MeshObject::draw_volume_cell_facets(
        const GEO::index_t begin_c,
        const GEO::index_t end_c,
        const GLUPprimitive prim
        ) {
        glupBegin(prim);
        for (GEO::index_t c = begin_c; c < end_c; ++c) {
            // Center of the cell, toward which its facets are pulled when they
            // are shrunk; only needed (and only computed) when shrink is set.
            double centroid[3] = {0.0, 0.0, 0.0};
            if (cells_shrink_factor() != 0.0) {
                const GEO::index_t nb_vertices = mesh_.cells.nb_vertices(c);
                for (GEO::index_t lv = 0; lv < nb_vertices; ++lv) {
                    double p[3] = {0.0, 0.0, 0.0};
                    get_vertex_position(mesh_.cells.vertex(c, lv), p);
                    centroid[0] += p[0];
                    centroid[1] += p[1];
                    centroid[2] += p[2];
                }
                centroid[0] /= double(nb_vertices);
                centroid[1] /= double(nb_vertices);
                centroid[2] /= double(nb_vertices);
            }

            const GEO::index_t nb_facets = mesh_.cells.nb_facets(c);
            for (GEO::index_t lf = 0; lf < nb_facets; ++lf) {
                // Index of the half-facet (cell, facet) in mesh.cell_facets: the
                // attribute of the facet is looked up with it.
                const GEO::index_t cell_facet = mesh_.cells.facet(c, lf);
                const GEO::index_t nb_vertices =
                    mesh_.cells.facet_nb_vertices(c, lf);
                if (prim == GLUP_QUADS) {
                    if (nb_vertices == 4) {
                        for (GEO::index_t lv = 0; lv < 4; ++lv)
                            draw_volume_facet_vertex(
                                c, lf, lv, cell_facet, centroid);
                    }
                }
                else if (nb_vertices == 3) {
                    for (GEO::index_t lv = 0; lv < 3; ++lv)
                        draw_volume_facet_vertex(
                            c, lf, lv, cell_facet, centroid);
                }
                else if (nb_vertices > 4) {
                    // Defensive fan for unexpected facet sizes (a cell facet is
                    // a triangle or a quad in GeoGram's cell descriptors).
                    for (GEO::index_t lv = 1; lv + 1 < nb_vertices; ++lv) {
                        for (const GEO::index_t k :
                             {GEO::index_t(0), lv, lv + 1})
                            draw_volume_facet_vertex(
                                c, lf, k, cell_facet, centroid);
                    }
                }
            }
        }
        glupEnd();
    }

    void MeshObject::draw_volume_facet_vertex(
        const GEO::index_t cell,
        const GEO::index_t lf,
        const GEO::index_t lv,
        const GEO::index_t cell_facet,
        const double* centroid
        ) {
        // The color comes from the attribute of the cell facet, so the two
        // facets shared by two adjacent cells can be colored differently.
        draw_attribute_as_tex_coord(cell_facet);

        const GEO::index_t v = mesh_.cells.facet_vertex(cell, lf, lv);

        // Without shrink, the vertex is emitted as-is, so that the facets fit
        // the mesh vertices exactly (and 2-D meshes are handled).
        const double shrink = cells_shrink_factor();
        if (shrink == 0.0) {
            draw_vertex(v);
            return;
        }

        // With shrink, the vertex is moved toward the center of its cell by the
        // same amount as GLUP applies to whole-cell primitives.
        double p[3] = {0.0, 0.0, 0.0};
        if (mesh_gfx_.get_animate() && mesh_.vertices.dimension() >= 6) {
            // Position of the vertex at the current animation time: the first
            // three coordinates are the position at t=0, the last three at t=1
            // (see MeshGfx::set_animate()).
            const double t = mesh_gfx_.get_time();
            const double s = 1.0 - t;
            for (GEO::coord_index_t coords = 0; coords < 3; ++coords) {
                if (mesh_.vertices.single_precision()) {
                    const float* q =
                        mesh_.vertices.single_precision_point_ptr(v);
                    p[coords] = s * double(q[coords]) + t * double(q[coords + 3]);
                }
                else {
                    const double* q = mesh_.vertices.point_ptr(v);
                    p[coords] = s * q[coords] + t * q[coords + 3];
                }
            }
        }
        else
            get_vertex_position(v, p);

        glupPrivateVertex3d(
            (1.0 - shrink) * p[0] + shrink * centroid[0],
            (1.0 - shrink) * p[1] + shrink * centroid[1],
            (1.0 - shrink) * p[2] + shrink * centroid[2]
        );
    }

    void MeshObject::draw_vertex(
        const GEO::index_t v
        ) {
        // Same as MeshGfx::draw_vertex(): when animation is active, the vertex
        // moves between the two positions stored in its six coordinates.
        if (mesh_gfx_.get_animate() && mesh_.vertices.dimension() >= 6) {
            const double t = mesh_gfx_.get_time();
            const double s = 1.0 - t;
            if (mesh_.vertices.single_precision()) {
                const float* p = mesh_.vertices.single_precision_point_ptr(v);
                glupPrivateVertex3f(
                    GLUPfloat(s * double(p[0]) + t * double(p[3])),
                    GLUPfloat(s * double(p[1]) + t * double(p[4])),
                    GLUPfloat(s * double(p[2]) + t * double(p[5]))
                );
            }
            else {
                const double* p = mesh_.vertices.point_ptr(v);
                glupPrivateVertex3d(
                    s * p[0] + t * p[3],
                    s * p[1] + t * p[4],
                    s * p[2] + t * p[5]
                );
            }
        }
        else if (mesh_.vertices.single_precision()) {
            if (mesh_.vertices.dimension() < 3)
                glupPrivateVertex2fv(
                    mesh_.vertices.single_precision_point_ptr(v));
            else
                glupPrivateVertex3fv(
                    mesh_.vertices.single_precision_point_ptr(v));
        }
        else {
            if (mesh_.vertices.dimension() < 3)
                glupPrivateVertex2dv(mesh_.vertices.point_ptr(v));
            else
                glupPrivateVertex3dv(mesh_.vertices.point_ptr(v));
        }
    }

    void MeshObject::get_vertex_position(
        const GEO::index_t v,
        double* p
        ) const {
        // 2-D meshes have no z coordinate (see get_bbox()).
        const GEO::coord_index_t dim = mesh_.vertices.dimension();
        p[0] = p[1] = p[2] = 0.0;
        if (mesh_.vertices.single_precision()) {
            const float* q = mesh_.vertices.single_precision_point_ptr(v);
            for (GEO::coord_index_t c = 0; c < dim && c < 3; ++c)
                p[c] = static_cast<double>(q[c]);
        }
        else {
            const double* q = mesh_.vertices.point_ptr(v);
            for (GEO::coord_index_t c = 0; c < dim && c < 3; ++c)
                p[c] = q[c];
        }
    }

    void MeshObject::draw_attribute_as_tex_coord(
        const GEO::index_t element
        ) {
        // begin_attributes() leaves the attribute unbound when the mesh does not
        // have it (see displayed_attribute_is_defined()): there is then no value
        // to feed the colormap with, and reading the adapter would abort on the
        // element type of an unbound attribute (ET_NONE).
        if (!scalar_attribute_.is_bound())
            return;

        // GeoBox only displays scalar (1-D) attributes: draw_scene() always
        // calls MeshGfx::set_scalar_attribute(), which sets attribute_dim_ = 1.
        glupPrivateTexCoord1d(scalar_attribute_[element]);
    }

    void MeshObject::begin_attributes(
        ) {
        // Same as MeshGfx::begin_attributes() (a protected member there), for
        // the 1-D scalar attributes displayed by GeoBox.
        if (attribute_subelements_ == GEO::MESH_NONE)
            return;

        const GEO::MeshSubElementsStore& subelements =
            mesh_.get_subelements_by_type(attribute_subelements_);
        scalar_attribute_.bind_if_is_defined(
            subelements.attributes(), colormap_attribute_name());
        if (!scalar_attribute_.is_bound()) {
            // No attribute to display: keep the plain element color.
            return;
        }

        glupEnable(GLUP_TEXTURING);
        glupTextureMode(GLUP_TEXTURE_REPLACE);
        glupTextureType(GLUP_TEXTURE_1D);

        // The colormap selected in the UI is a 1-D texture (stored as a 2-D
        // texture with a single row, see GLUP_TEXTURE_1D_TARGET).
        glActiveTexture(GL_TEXTURE0 + GLUP_TEXTURE_1D_UNIT);
        glBindTexture(
            GLUP_TEXTURE_1D_TARGET,
            colormap_texture()
        );

        // Rescale the displayed range [colormap_mapping_min(),
        // colormap_mapping_max()] to [0,1], i.e. onto the whole colormap. The
        // values sampled here are the clamped ones (see
        // update_display_attribute()), so the range is never exceeded and the
        // elements outside of it have the color of its nearest end. The bounds
        // are swapped while the colormap is flipped, which maps the range back
        // to front (see colormap_mapping_min()).
        GEO::glupMapTexCoords1d(
            colormap_mapping_min(),
            colormap_mapping_max(),
            1
        );

        if (!glupIsEnabled(GLUP_NORMAL_MAPPING))
            glupSetColor3f(GLUP_FRONT_AND_BACK_COLOR, 1.0f, 1.0f, 1.0f);
    }

    void MeshObject::end_attributes(
        ) {
        // Same as MeshGfx::end_attributes().
        if (scalar_attribute_.is_bound()) {
            glupDisable(GLUP_TEXTURING);
            scalar_attribute_.unbind();
        }
        glupMatrixMode(GLUP_TEXTURE_MATRIX);
        glupLoadIdentity();
        glupMatrixMode(GLUP_MODELVIEW_MATRIX);
    }

    void MeshObject::autorange(
        ) {
        if (attribute_subelements_ == GEO::MESH_NONE)
            return;

        const GEO::MeshSubElementsStore& subelements =
            mesh_.get_subelements_by_type(attribute_subelements_);
            GEO::ReadOnlyScalarAttributeAdapter attribute(
            subelements.attributes(), point_attribute_name_
            );

        attribute_min_ = 0.0;
        attribute_max_ = 0.0;
        if (attribute.is_bound()) {
            attribute_min_ = GEO::Numeric::max_float32();
            attribute_max_ = GEO::Numeric::min_float32();
            for (GEO::index_t i = 0; i < subelements.nb(); ++i) {
                attribute_min_ =
                    std::min(attribute_min_, static_cast<float>(attribute[i]));
                attribute_max_ =
                    std::max(attribute_max_, static_cast<float>(attribute[i]));
            }
        }
    }

    void MeshObject::update_display_attribute(
        ) {
        // Nothing to sample the colormap with: the display copy would have no
        // source to be built from, and the displayed attribute has to be
        // sampled as it is.
        if (attribute_subelements_ == GEO::MESH_NONE) {
            display_attribute_.up_to_date = false;
            return;
        }

        GEO::MeshSubElementsStore& subelements =
            mesh_.get_subelements_by_type(attribute_subelements_);
        GEO::ReadOnlyScalarAttributeAdapter source(
            subelements.attributes(), point_attribute_name_
        );
        // A mesh whose displayed attribute is named like the display copy would
        // make that copy read itself as its source: there is nothing to copy
        // from then, and copying would destroy the values it has to hold.
        if (!source.is_bound() ||
            point_attribute_name_ == DISPLAY_ATTRIBUTE_NAME) {
            display_attribute_.up_to_date = false;
            return;
        }

        const GEO::index_t nb_elements = subelements.nb();
        const double min_value = colormap_range_min();
        const double max_value = colormap_range_max();

        // Scanning the attribute (and having MeshGfx re-upload it) is only
        // worth it when something the copy depends on changed: the attribute it
        // is built from, or the range its values are clamped to.
        if (display_attribute_.up_to_date &&
            display_attribute_.store == source.attribute_store() &&
            display_attribute_.name == point_attribute_name_ &&
            display_attribute_.element_index == source.element_index() &&
            display_attribute_.nb_elements == nb_elements &&
            display_attribute_.min == min_value &&
            display_attribute_.max == max_value) {
            return;
        }

        // The copy is GeoBox's own: an attribute of the same name the mesh
        // happens to carry (a mesh file, say) is replaced whatever its type, so
        // that the name always refers to a scalar attribute of ours.
        if (subelements.attributes().is_defined(DISPLAY_ATTRIBUTE_NAME))
            subelements.attributes().delete_attribute_store(
                DISPLAY_ATTRIBUTE_NAME);

        // A float attribute: the values only reach the colormap through the
        // texture coordinates GLUP interpolates, which are single precision
        // whatever the attribute holds.
        GEO::Attribute<float> display(
            subelements.attributes(), DISPLAY_ATTRIBUTE_NAME
        );
        for (GEO::index_t i = 0; i < nb_elements; ++i) {
            display[i] = static_cast<float>(std::clamp(
                static_cast<double>(source[i]), min_value, max_value
            ));
        }

        // MeshGfx samples the attribute through a buffer object, which it only
        // re-uploads when it is told the mesh changed: the values just written
        // would not be the ones drawn otherwise, since set_scalar_attribute()
        // only marks them dirty when the attribute it samples changes.
        // set_mesh() invalidates more than that (the geometry buffers too), but
        // it is the only public way to hand MeshGfx a modified mesh.
        mesh_gfx_.set_mesh(&mesh_);

        display_attribute_.store = source.attribute_store();
        display_attribute_.name = point_attribute_name_;
        display_attribute_.element_index = source.element_index();
        display_attribute_.nb_elements = nb_elements;
        display_attribute_.min = min_value;
        display_attribute_.max = max_value;
        display_attribute_.up_to_date = true;
    }

    std::string MeshObject::colormap_attribute_name(
        ) {
        update_display_attribute();
        return display_attribute_.up_to_date ?
            DISPLAY_ATTRIBUTE_NAME : point_attribute_name_;
    }

    void MeshObject::update_display_tex_coords(
        ) {
        // Nothing to scale, or no scale to apply: MeshGfx can sample the
        // selected attribute directly.
        if (tex_coord_subelements_ == GEO::MESH_NONE ||
            tex_coord_attribute_name_.empty() ||
            !uv_scale_enabled_ ||
            (uv_scale_u_ == 1.0f && uv_scale_v_ == 1.0f)) {
            display_tex_coords_.up_to_date = false;
            return;
        }

        GEO::MeshSubElementsStore& subelements =
            mesh_.get_subelements_by_type(tex_coord_subelements_);
        GEO::ReadOnlyScalarAttributeAdapter source_u(
            subelements.attributes(), tex_coord_attribute_name_ + "[0]");
        GEO::ReadOnlyScalarAttributeAdapter source_v(
            subelements.attributes(), tex_coord_attribute_name_ + "[1]");
        // Reading the copy as its own source would destroy the values it has to
        // hold; a mesh that does not have the attribute cannot be scaled either.
        if (!source_u.is_bound() || !source_v.is_bound() ||
            tex_coord_attribute_name_ == DISPLAY_TEX_COORD_NAME) {
            display_tex_coords_.up_to_date = false;
            return;
        }

        const GEO::index_t nb_elements = subelements.nb();
        if (display_tex_coords_.up_to_date &&
            display_tex_coords_.store == source_u.attribute_store() &&
            display_tex_coords_.name == tex_coord_attribute_name_ &&
            display_tex_coords_.nb_elements == nb_elements &&
            display_tex_coords_.scale_u == uv_scale_u_ &&
            display_tex_coords_.scale_v == uv_scale_v_) {
            return;
        }

        // The copy is GeoBox's own: replace an attribute of the same name the
        // mesh happens to carry, whatever its type.
        if (subelements.attributes().is_defined(DISPLAY_TEX_COORD_NAME))
            subelements.attributes().delete_attribute_store(
                DISPLAY_TEX_COORD_NAME);

        // A float vector attribute: MeshGfx samples its two components as
        // "name[0]" and "name[1]", like any 2D texture-coordinate attribute.
        GEO::Attribute<float> display;
        display.create_vector_attribute(
            subelements.attributes(), DISPLAY_TEX_COORD_NAME, 2);
        for (GEO::index_t i = 0; i < nb_elements; ++i) {
            display[2 * i] = uv_scale_u_ * static_cast<float>(source_u[i]);
            display[2 * i + 1] =
                uv_scale_v_ * static_cast<float>(source_v[i]);
        }

        // As in update_display_attribute(): MeshGfx only re-uploads the
        // attribute through its buffer objects when it is told the mesh changed.
        mesh_gfx_.set_mesh(&mesh_);

        display_tex_coords_.store = source_u.attribute_store();
        display_tex_coords_.name = tex_coord_attribute_name_;
        display_tex_coords_.nb_elements = nb_elements;
        display_tex_coords_.scale_u = uv_scale_u_;
        display_tex_coords_.scale_v = uv_scale_v_;
        display_tex_coords_.up_to_date = true;
    }

    std::string MeshObject::tex_coord_attribute_name_for_drawing(
        ) {
        update_display_tex_coords();
        return display_tex_coords_.up_to_date ?
            DISPLAY_TEX_COORD_NAME : tex_coord_attribute_name_;
    }

    std::string MeshObject::attribute_names(
        ) const {
        std::vector<std::string> attributes;
        GEO::String::split_string(
            mesh_.get_scalar_attributes(), ';', attributes);

        // The display copies are not attributes of the mesh: they only carry
        // the values the current display settings ask for (see
        // update_display_attribute() and update_display_tex_coords()) and must
        // not be selectable. get_scalar_attributes() expands a vector attribute
        // into its components ("...__geobox_uv[0]" and "[1]"), so the base name
        // of the scaled texture coordinates is matched rather than the whole
        // name.
        const std::string hidden =
            std::string(".") + DISPLAY_ATTRIBUTE_NAME;
        const std::string hidden_uv =
            std::string(".") + DISPLAY_TEX_COORD_NAME;
        std::string result;
        for (const std::string& attribute : attributes) {
            if (attribute.size() >= hidden.size() &&
                attribute.compare(
                    attribute.size() - hidden.size(),
                    hidden.size(), hidden) == 0) {
                continue;
            }
            const std::size_t uv_position = attribute.find(hidden_uv);
            if (uv_position != std::string::npos) {
                const std::size_t after =
                    uv_position + hidden_uv.size();
                if (after == attribute.size() ||
                    attribute[after] == '[') {
                    continue;
                }
            }
            if (!result.empty())
                result += ';';
            result += attribute;
        }
        return result;
    }

    std::vector<MeshObject::AttributeInfo> MeshObject::attribute_infos(
        ) const {
        // Every store of the mesh that can carry attributes, in the order the
        // applications see them in their selectors.
        static const GEO::MeshElementsFlags SUBELEMENTS[] = {
            GEO::MESH_VERTICES,
            GEO::MESH_EDGES,
            GEO::MESH_FACETS,
            GEO::MESH_FACET_CORNERS,
            GEO::MESH_CELLS,
            GEO::MESH_CELL_CORNERS,
            GEO::MESH_CELL_FACETS
        };

        // The coordinates of the vertices are stored as an attribute of the
        // vertices store ("point", or "point_fp32" with single precision
        // vertices, see MeshVertices::single_precision()): they are the
        // geometry of the mesh, not data attached to it, so they are not
        // offered as an attribute an application can pick.
        const std::string coordinates_name =
            mesh_.vertices.single_precision() ? "point_fp32" : "point";

        std::vector<AttributeInfo> result;
        for (const GEO::MeshElementsFlags subelements : SUBELEMENTS) {
            const GEO::MeshSubElementsStore& store =
                mesh_.get_subelements_by_type(subelements);
            GEO::vector<std::string> names;
            store.attributes().list_attribute_names(names);
            for (const std::string& name : names) {
                if (subelements == GEO::MESH_VERTICES &&
                    name == coordinates_name)
                    continue;
                // The display copies are built by this object for the
                // renderer, not written by the user's data (see
                // DISPLAY_ATTRIBUTE_NAME and DISPLAY_TEX_COORD_NAME).
                if (name == DISPLAY_ATTRIBUTE_NAME ||
                    name == DISPLAY_TEX_COORD_NAME)
                    continue;
                const GEO::AttributeStore* attribute =
                    store.attributes().find_attribute_store(name);
                if (attribute == nullptr)
                    continue;

                AttributeInfo info;
                info.subelements = subelements;
                info.name = name;
                info.element_type = attribute->element_typeid_name();
                info.dimension = attribute->dimension();
                info.nb_elements = store.nb();
                result.push_back(std::move(info));
            }
        }
        return result;
    }

    bool MeshObject::can_write_extension(
        const std::string& extension
        ) const {
        // The formats GeoLio registers to read the files of other tools have no
        // save() (see geolio/io/io.h): they are read-only, whatever the mesh.
        static const char* const CAN_WRITE_EXTENSIONS[] = {
            "geogram", "mesh", "msh", "obj", "off", "pts", "stl"
        };
        return std::ranges::find(CAN_WRITE_EXTENSIONS, extension) != std::end(CAN_WRITE_EXTENSIONS);
    }

    bool MeshObject::save(
        const std::string& filename
        ) {
        // The data an application computed lives in the attributes of the mesh
        // (e.g. the "weight" of a sizing field): unlike the default flags, which
        // save none, all of them are written.
        GEO::MeshIOFlags flags;
        flags.set_attributes(GEO::MESH_ALL_ATTRIBUTES);

        // A .geogram file stores double precision coordinates: promote them
        // first, as GeoGram's mesh applications do, so that a mesh loaded in
        // single precision is written faithfully.
        if (GEO::FileSystem::extension(filename) == "geogram")
            mesh_.vertices.set_double_precision();

        return GEO::mesh_save(mesh_, filename, flags);
    }

    bool MeshObject::display_attribute(
        const std::string& attribute
        ) {
        // The name comes in the "subelements.name" form (see attribute_names()).
        std::string subelements_name;
        std::string name;
        GEO::String::split_string(attribute, '.', subelements_name, name);
        const GEO::MeshElementsFlags subelements =
            GEO::Mesh::name_to_subelements_type(subelements_name);
        if (subelements == GEO::MESH_NONE || name.empty())
            return false;

        // Asking for an attribute the mesh does not have would leave the
        // colormap sampling nothing: the request is refused, and what the object
        // displays stays as it is.
        if (!mesh_.get_subelements_by_type(subelements).attributes().
                is_defined(name))
            return false;

        set_attribute(attribute);
        painting_mode_ = PaintingMode::ATTRIBUTE;

        autorange();

        return true;
    }

    void MeshObject::set_attribute(
        const std::string& attribute
        ) {
        vertices_point_attribute_name_ = attribute;
        std::string subelements_name;
        GEO::String::split_string(
            vertices_point_attribute_name_, '.',
            subelements_name,
            point_attribute_name_);

        attribute_subelements_ = GEO::Mesh::name_to_subelements_type(subelements_name);

        // Same as GeoGram's SimpleMeshApplication::set_attribute(): the range
        // is only recomputed while the min/max fields are still untouched (both
        // zero). A range the user set therefore describes the values they want
        // to look at and survives selecting another attribute, which is what
        // makes the fields and the coloring agree: the colormap always spans
        // exactly the range the fields read, whatever the attribute, and the
        // values outside of it take the color of the nearest end (see
        // update_display_attribute()). The "autorange" button restores the raw
        // range (GEO::NO_INDEX included) at any time.
        if (attribute_min_ == 0.0f && attribute_max_ == 0.0f)
            autorange();
    }

    GLuint MeshObject::texture(
        ) {
        if (texture_filename_.empty())
            return 0;

        auto it = texture_cache.find(texture_filename_);
        if (it == texture_cache.end()) {
            it = texture_cache.emplace(
                texture_filename_,
                create_texture_from_image(texture_filename_)).first;
        }
        return it->second;
    }

    void MeshObject::set_tex_coords(
        const std::string& attribute
        ) {
        tex_coord_full_name_ = attribute;
        split_attribute_name(
            attribute, tex_coord_subelements_, tex_coord_attribute_name_);
        // The scaled copy is built on the store of the selected attribute (see
        // update_display_tex_coords()).
        display_tex_coords_ = DisplayTexCoords();
    }

    GEO::index_t MeshObject::attribute_dimension(
        const GEO::MeshElementsFlags subelements,
        const std::string& name
        ) const {
        if (subelements == GEO::MESH_NONE || name.empty())
            return 0;

        const GEO::MeshSubElementsStore& store =
            mesh_.get_subelements_by_type(subelements);
        const GEO::AttributeStore* attribute =
            store.attributes().find_attribute_store(name);
        if (attribute == nullptr)
            return 0;

        // A vector<vec3> store holds three scalar components per element, and
        // MeshGfx addresses them as "name[0]", "name[1]" and "name[2]" (see
        // ReadOnlyScalarAttributeAdapter), so it counts as a 3D attribute.
        GEO::index_t dimension = attribute->dimension();
        const std::string type = attribute->element_typeid_name();
        if (type == typeid(GEO::vec2).name())
            dimension *= 2;
        else if (type == typeid(GEO::vec3).name())
            dimension *= 3;
        return dimension;
    }

    std::string MeshObject::texture_attribute_names(
        ) const {
        std::vector<std::string> attributes;
        GEO::String::split_string(
            mesh_.get_vector_attributes(), ';', attributes);

        std::vector<std::string> kept;
        for (const std::string& attribute : attributes) {
            GEO::MeshElementsFlags subelements = GEO::MESH_NONE;
            std::string name;
            split_attribute_name(attribute, subelements, name);
            // The scaled copy is a display artefact, not a texture-coordinate
            // attribute the user can select (see DISPLAY_TEX_COORD_NAME).
            if (subelements == GEO::MESH_NONE ||
                !can_be_textured(subelements) ||
                name == DISPLAY_TEX_COORD_NAME ||
                attribute_dimension(subelements, name) != 2)
                continue;
            if (std::ranges::find(kept, attribute) != kept.end())
                continue;
            kept.push_back(attribute);
        }

        std::string result;
        for (const std::string& attribute : kept) {
            if (!result.empty())
                result += ';';
            result += attribute;
        }
        return result;
    }
}
