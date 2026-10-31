//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/23.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_BASE_APPLICATION_H
#define GEOLIO_BASE_APPLICATION_H
#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <geogram_gfx/third_party/imgui/imgui.h>
#include <geolio/common/utils.h>
#include <geolio/geobox/object/base_object.h>
#include <geolio/geobox/object/directional_object.h>
#include <geolio/geobox/object/mesh_object.h>

namespace geolio::geobox
{
    /**
     * @brief Label of the sub-menu that collects the applications which do
     *        not specify any menu path (i.e. do not say where they belong).
     */
    inline constexpr auto UNGROUPED_APPLICATIONS_MENU_NAME = "ungrouped";

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
     *          An application with no menu path set lands in the "ungrouped"
     *          sub-menu (see UNGROUPED_APPLICATIONS_MENU_NAME).
     *          Besides the window skeleton, the class provides the selection
     *          interactions many applications need -- picking one of the mesh
     *          objects of the scene, and picking one of its attributes -- so
     *          that an application only says which meshes/attributes it
     *          accepts (see draw_mesh_selector() and
     *          draw_attribute_selector()).
     */
    class BaseApplication {
    public:
        /**
         * @brief Constructs a sub-application.
         * @param[in] application_name Name shown in the menu and the window
         *            title.
         * @param[in] objects Objects of the main application; kept by
         *            reference, so that the selections offered below and the
         *            objects an application adds (e.g. the result of an
         *            operation) are shared by the whole GeoBox session.
         */
        explicit BaseApplication(
            std::string application_name,
            ApplicationObjects& objects
            ) : id_(generate_random_string(22)),
                application_name_(std::move(application_name)),
                objects_(objects)
        {}

        virtual ~BaseApplication() = default;

        /**
         * @brief Unique identifier of this application instance.
         * @details Generated when the instance is constructed, so neither the
         *          concrete class nor the registration site has to provide
         *          one.
         */
        [[nodiscard]] const std::string& id() const { return id_; }

        /**
         * @brief User-facing name of this application.
         * @details Used as the menu item label and as the window title.
         *          Decided by the concrete class in its constructor.
         */
        [[nodiscard]] const std::string& application_name() const { return application_name_; }

        /**
         * @brief Labels of the sub-menus this application lives in.
         * @details A path like {"Mesh", "Optimization"} yields
         *          Applications > Mesh > Optimization > entry. It is never
         *          empty: when no path is set, it is
         *          {UNGROUPED_APPLICATIONS_MENU_NAME}, so such applications are
         *          grouped under Applications > ungrouped instead of being
         *          mixed with the top level entries.
         *          Filled in by the application factory from the registration
         *          data (see application_registry.h).
         */
        [[nodiscard]] const std::vector<std::string>& menu_path() const { return menu_path_; }

        /**
         * @brief Sets the sub-menu chain this application lives in.
         * @details Called once by the registry-based factory right after
         *          construction, so concrete applications do not need to
         *          know about menu placement. Empty labels are dropped; if no
         *          label is left, the path defaults to
         *          {UNGROUPED_APPLICATIONS_MENU_NAME}.
         */
        void set_menu_path(std::vector<std::string> menu_path) {
            std::vector<std::string> labels;
            labels.reserve(menu_path.size());
            for (auto& label : menu_path) {
                if (!label.empty())
                    labels.push_back(std::move(label));
            }
            if (labels.empty())
                labels.emplace_back(UNGROUPED_APPLICATIONS_MENU_NAME);
            menu_path_ = std::move(labels);
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
            ImGui::SetNextWindowBgAlpha(0.9f);
            if (ImGui::Begin(application_name_.c_str(), &window_visible_)) {
                // Width shared by the numeric sliders/drags of the
                // applications: 75% of the default item width. It is computed
                // here, at draw time, and not in the constructor:
                // ImGui::CalcItemWidth() needs an active window (it
                // dereferences the current window), which does not exist yet
                // while the applications are being constructed, and calling it
                // there crashes at startup. Refreshing it on each frame makes
                // it follow window resizes.
                slider_width_ = ImGui::CalcItemWidth() * 0.75f;
                draw_window_contents();
            }

            ImGui::End();
        }

    protected:
        /**
         * @brief Hook: derived classes draw the window body here.
         */
        virtual void draw_window_contents() = 0;

        // ---------------------------------------------------------------------
        // Selections
        // ---------------------------------------------------------------------

        /**
         * @brief Predicate deciding which meshes an application can work on.
         * @details Called with the mesh of every candidate object; an object
         *          whose mesh it rejects is not offered by
         *          draw_mesh_selector(). The mesh of a selection is filtered
         *          too: a mesh that stops matching (e.g. because a previous run
         *          of the application changed its connectivity) clears that
         *          selection.
         */
        using MeshFilter = std::function<bool(const GEO::Mesh&)>;

        /**
         * @brief Predicate deciding which attributes an application can work
         *        on.
         * @details Called with the description of every attribute of the
         *          selected mesh; draw_attribute_selector() only offers the
         *          ones it accepts.
         * @ref MeshRandomSamplingApplication::is_weight_attribute() for an
         *      example (one double per vertex).
         */
        using AttributeFilter =
            std::function<bool(const MeshObject::AttributeInfo&)>;

        /**
         * @brief Predicate deciding which directional fields an application can
         *        work on.
         * @details Called with every DirectionalObject of the scene;
         *          draw_directional_selector() only offers the ones it accepts.
         *          It is given the object rather than its vectors, since what
         *          an application needs to know is where the field is
         *          attached and how many frames it has, e.g.
         *          @code
         *          [&mesh](const DirectionalObject& field) {
         *              return field.attached() &&
         *                     field.nb_frames() == mesh.cells.nb();
         *          }
         *          @endcode
         */
        using DirectionalFilter =
            std::function<bool(const DirectionalObject&)>;

        /**
         * @brief Draws a mesh object picker, i.e. lets the user pick one of
         *        the meshes of the scene.
         * @details Offers every MeshObject of the scene whose mesh matches
         *          @p filter, and stores what the user picks in @p selection.
         *          A single candidate is selected automatically, so that an
         *          application whose scene holds one mesh is usable right away,
         *          and a selection whose mesh no longer matches @p filter is
         *          cleared.
         *          The selection belongs to the caller, so an application that
         *          works on several meshes declares one selection per mesh and
         *          draws one picker for each of them:
         *          @code
         *          draw_mesh_selector(base_surface_, facets_filter, "Surface");
         *          draw_mesh_selector(samples_, vertices_filter, "Samples");
         *          @endcode
         * @param[in,out] selection The selected mesh object: set when the user
         *            picks one, cleared when the mesh it names cannot be picked
         *            any more. Lock it to use it: the scene owns the
         *            objects, so a weak pointer never keeps a mesh the scene
         *            dropped alive.
         * @param[in] filter Which meshes can be picked; all of them when
         *            empty.
         * @param[in] label Text shown on the left of the combo box. It also
         *            names the ImGui item, so it must differ from the label of
         *            another picker drawn in the same window.
         * @return true when the selection changed, i.e. when the application
         *         has to refresh what it derived from the selected mesh.
         */
        bool draw_mesh_selector(
            std::weak_ptr<MeshObject>& selection,
            const MeshFilter& filter = {},
            const char* label = "Mesh"
            ) {
            bool changed = false;

            // The objects the application can work on: its filter decides.
            std::vector<std::shared_ptr<MeshObject>> mesh_objects;
            for (const auto& object : objects_) {
                const auto mesh_object =
                    std::dynamic_pointer_cast<MeshObject>(object);
                if (mesh_object == nullptr)
                    continue;
                if (filter && !filter(mesh_object->mesh()))
                    continue;
                mesh_objects.push_back(mesh_object);
            }

            // A single candidate is picked automatically.
            if (mesh_objects.size() == 1 && selection.expired()) {
                selection = mesh_objects[0];
                changed = true;
            }

            // The selected mesh may have stopped matching the filter (a
            // previous run of the application may have changed it): drop the
            // selection, which also disables the parameters of the
            // application until another mesh is picked.
            if (const auto selected = selection.lock();
                selected != nullptr &&
                std::ranges::find(mesh_objects, selected) ==
                    mesh_objects.end()) {
                selection.reset();
                changed = true;
            }

            if (mesh_objects.empty()) {
                const std::string no_object =
                    std::string("No `") + label + "` object available.";
                ImGui::TextDisabled("%s", no_object.c_str());
                return changed;
            }

            // Index of the currently selected mesh object, -1 when none.
            const auto selected = selection.lock();
            int current_index = -1;
            for (std::size_t i = 0; i < mesh_objects.size(); ++i) {
                if (mesh_objects[i] == selected) {
                    current_index = static_cast<int>(i);
                    break;
                }
            }

            std::string preview = "Select a mesh object";
            if (current_index >= 0)
                preview = mesh_objects[static_cast<std::size_t>(current_index)]
                              ->name();

            // The label on the left, the combo box on its right, on the same
            // line (the "##" prefix hides the combo's duplicate label).
            // AlignTextToFramePadding() centers the text baseline against the
            // combo box's frame padding, so both stay vertically centered.
            ImGui::AlignTextToFramePadding();
            ImGui::TextUnformatted(label);
            ImGui::SameLine();
            const std::string combo_id = std::string("##") + label;
            if (ImGui::BeginCombo(combo_id.c_str(), preview.c_str())) {
                for (std::size_t i = 0; i < mesh_objects.size(); ++i) {
                    const bool is_selected =
                        (current_index == static_cast<int>(i));
                    if (ImGui::Selectable(
                            mesh_objects[i]->name().c_str(), is_selected)) {
                        selection = mesh_objects[i];
                        changed = true;
                    }
                    if (is_selected)
                        ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();
            }
            return changed;
        }

        /**
         * @brief Draws a directional field picker, i.e. lets the user pick one
         *        of the fields of the scene.
         * @details Offers every DirectionalObject of the scene that matches
         *          @p filter, and stores what the user picks in @p selection.
         *          A single candidate is selected automatically, so that an
         *          application whose scene holds one field is usable right
         *          away, and a selection whose field no longer matches
         *          @p filter (or left the scene) is cleared.
         *          The candidates are listed by name; the one that is selected
         *          keeps it, like the one of draw_mesh_selector().
         * @param[in,out] selection The selected field: set when the user picks
         *            one, cleared when the field it names cannot be
         *            picked any more. Lock it to use it: the scene owns the
         *            objects, so a weak pointer never keeps a field the scene
         *            dropped alive.
         * @param[in] filter Which fields can be picked; all of them when empty.
         * @param[in] label Text shown on the left of the combo box. It also
         *            names the ImGui item, so it must differ from the label of
         *            another picker drawn in the same window.
         * @return true when the selection changed, i.e. when the application
         *         has to refresh what it derived from the selected field.
         */
        bool draw_directional_selector(
            std::weak_ptr<DirectionalObject>& selection,
            const DirectionalFilter& filter = {},
            const char* label = "Field"
            ) {
            bool changed = false;

            // The fields the application can work on: its filter decides.
            std::vector<std::shared_ptr<DirectionalObject>> directional_objects;
            for (const auto& object : objects_) {
                const auto directional_object =
                    std::dynamic_pointer_cast<DirectionalObject>(object);
                if (directional_object == nullptr)
                    continue;
                if (filter && !filter(*directional_object))
                    continue;
                directional_objects.push_back(directional_object);
            }

            // A single candidate is picked automatically.
            if (directional_objects.size() == 1 && selection.expired()) {
                selection = directional_objects[0];
                changed = true;
            }

            // The selected field may have stopped matching the filter, or left
            // the scene: drop the selection, which also disables the parameters
            // of the application until another field is picked.
            if (const auto selected = selection.lock();
                selected != nullptr &&
                std::ranges::find(directional_objects, selected) ==
                    directional_objects.end()) {
                selection.reset();
                changed = true;
            }

            if (directional_objects.empty()) {
                const std::string no_object =
                    std::string("No `") + label + "` object available.";
                ImGui::TextDisabled("%s", no_object.c_str());
                return changed;
            }

            // Index of the currently selected field, -1 when none.
            const auto selected = selection.lock();
            int current_index = -1;
            for (std::size_t i = 0; i < directional_objects.size(); ++i) {
                if (directional_objects[i] == selected) {
                    current_index = static_cast<int>(i);
                    break;
                }
            }

            std::string preview = "Select a directional field";
            if (current_index >= 0) {
                preview = directional_objects[static_cast<std::size_t>(current_index)]
                              ->name();
            }

            // The label on the left, the combo box on its right, on the same
            // line (see draw_mesh_selector()).
            ImGui::AlignTextToFramePadding();
            ImGui::TextUnformatted(label);
            ImGui::SameLine();
            const std::string combo_id = std::string("##") + label;
            if (ImGui::BeginCombo(combo_id.c_str(), preview.c_str())) {
                for (std::size_t i = 0; i < directional_objects.size(); ++i) {
                    const bool is_selected =
                        (current_index == static_cast<int>(i));
                    if (ImGui::Selectable(
                            directional_objects[i]->name().c_str(), is_selected)) {
                        selection = directional_objects[i];
                        changed = true;
                    }
                    if (is_selected)
                        ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();
            }
            return changed;
        }

        /**
         * @brief Draws the attribute picker of a mesh object.
         * @details Offers the attributes of @p mesh_object that match
         *          @p filter (see MeshObject::attribute_infos()) and stores
         *          what the user picks in @p selection. The attributes the
         *          object builds only to draw the mesh are not offered (they
         *          are not attributes of the mesh). The first matching
         *          attribute is selected automatically; a selection that is
         *          not offered any more (another mesh picked, mesh reloaded)
         *          falls back to it. The selection belongs to the caller, like
         *          the one of draw_mesh_selector().
         * @param[in,out] selection The selected attribute; its default value
         *            (GEO::MESH_NONE sub-elements, empty name) stands for "no
         *            attribute". Its \c name is the name the GeoGram attribute
         *            API uses, and \c subelements the store it lives on.
         * @param[in] mesh_object Mesh whose attributes are offered.
         * @param[in] filter Which attributes can be picked; all of them when
         *            empty.
         * @param[in] label Text shown on the left of the combo box; it also
         *            names the ImGui item, so two pickers drawn in the same
         *            window need different labels.
         * @return true when the selection changed.
         */
        bool draw_attribute_selector(
            MeshObject::AttributeInfo& selection,
            const MeshObject& mesh_object,
            const AttributeFilter& filter = {},
            const char* label = "Attr."
            ) {
            bool changed = false;

            // The attributes the application can work on.
            std::vector<MeshObject::AttributeInfo> attributes;
            for (auto& attribute : mesh_object.attribute_infos()) {
                if (!filter || filter(attribute))
                    attributes.push_back(std::move(attribute));
            }

            if (attributes.empty()) {
                const std::string no_match =
                    std::string("No matching `") + label + "`.";
                ImGui::TextDisabled("%s", no_match.c_str());
                if (selection.subelements != GEO::MESH_NONE) {
                    selection = MeshObject::AttributeInfo();
                    changed = true;
                }
                return changed;
            }

            // The attribute the selection names may not be offered any more:
            // fall back to the first attribute that matched the filter.
            std::size_t selected = 0;
            for (std::size_t i = 0; i < attributes.size(); ++i) {
                if (attributes[i].full_name() == selection.full_name())
                    selected = i;
            }
            if (attributes[selected].full_name() != selection.full_name()) {
                selected = 0;
                changed = true;
            }
            selection = attributes[selected];

            std::string items;
            for (const auto& attribute : attributes) {
                items += attribute.full_name();
                items.push_back('\0');
            }

            ImGui::AlignTextToFramePadding();
            ImGui::TextUnformatted(label);
            ImGui::SameLine();
            const std::string combo_id = std::string("##") + label;
            int current_index = static_cast<int>(selected);
            if (ImGui::Combo(combo_id.c_str(), &current_index, items.c_str())) {
                selection = attributes[static_cast<std::size_t>(current_index)];
                changed = true;
            }
            return changed;
        }

        const std::string id_;
        const std::string application_name_;
        /** Objects of the GeoBox session; an application may add to them (e.g.
         *  the mesh an operation produced). */
        ApplicationObjects& objects_;
        std::string menu_tooltip_;
        /** Parent sub-menu labels, outermost first (see menu_path()). Never
         *  empty: defaults to {UNGROUPED_APPLICATIONS_MENU_NAME}. */
        std::vector<std::string> menu_path_{UNGROUPED_APPLICATIONS_MENU_NAME};

        bool window_visible_ = false;

        /** Width used by every slider/drag of the applications: 75% of the
         *  default item width. Refreshed at the beginning of each
         *  draw_window(), so it follows window resizes; it must not be
         *  computed in a constructor (no ImGui frame is active then). */
        float slider_width_ = 0.0f;
    };
}

#endif //GEOLIO_BASE_APPLICATION_H
