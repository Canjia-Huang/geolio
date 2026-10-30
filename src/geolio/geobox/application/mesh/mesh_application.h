//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/29.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_MESH_APPLICATION_H
#define GEOLIO_MESH_APPLICATION_H
#include <geolio/geobox/application/base_application.h>
#include <geolio/geobox/object/mesh_object.h>

namespace geolio::geobox
{
    class MeshApplication : public BaseApplication {
    public:
        MeshApplication(
            std::string application_name,
            std::vector<std::shared_ptr<BaseObject>>& objects
            ) : BaseApplication(std::move(application_name)),
                objects_(objects)
        {}

    protected:
        virtual bool is_mesh_valid(const GEO::Mesh& mesh) { return true; }

        virtual void after_selecting_mesh() {}

        virtual void draw_window_contents_details() = 0;

        void draw_window_contents() override {
            // Width shared by all the numeric sliders/drags of the derived
            // windows: 75% of the default item width. It is computed here, at
            // draw time, and not in the constructor: ImGui::CalcItemWidth()
            // needs an active context and window (it dereferences the current
            // window), which do not exist yet while the applications are
            // being constructed, and calling it there crashes at startup.
            slider_width_ = ImGui::CalcItemWidth() * 0.75f;

            // Collect the objects whose type is a mesh ("Mesh", see MeshObject).
            std::vector<std::shared_ptr<MeshObject>> mesh_objects;
            for (const auto& object : objects_) {
                const auto mesh_object = std::dynamic_pointer_cast<MeshObject>(object);
                if (mesh_object == nullptr)
                    continue;
                if (const GEO::Mesh& mesh = mesh_object->mesh();
                    is_mesh_valid(mesh))
                    mesh_objects.push_back(mesh_object);
            }

            // When only one triangular mesh object exists, select it automatically
            if (mesh_objects.size() == 1 && selected_mesh_object_.expired()) {
                selected_mesh_object_ = mesh_objects[0];
                after_selecting_mesh();
            }

            // Selection combo box: pick one mesh object.
            if (const bool has_mesh_object = !mesh_objects.empty();
                !has_mesh_object)
                ImGui::TextUnformatted("No mesh object available.");
            else {
                // Index of the currently selected mesh object, -1 when none.
                int current_index = -1;
                const auto selected = selected_mesh_object_.lock();
                for (size_t i = 0; i < mesh_objects.size(); ++i) {
                    if (mesh_objects[i] == selected) {
                        current_index = static_cast<int>(i);
                        break;
                    }
                }

                const char* preview = current_index >= 0
                    ? mesh_objects[static_cast<size_t>(current_index)]->name().c_str()
                    : "Select a mesh object";
                // "Mesh" text on the left, combo box to its right on the same
                // line (the "##" label hides ImGui's built-in duplicate label).
                // AlignTextToFramePadding() centers the text baseline against the
                // combo box's frame padding, so both stay vertically centered.
                ImGui::AlignTextToFramePadding();
                ImGui::Text("Mesh");
                ImGui::SameLine();
                if (ImGui::BeginCombo("##mesh_object", preview)) {
                    for (size_t i = 0; i < mesh_objects.size(); ++i) {
                        const bool is_selected = (current_index == static_cast<int>(i));
                        if (ImGui::Selectable(mesh_objects[i]->name().c_str(), is_selected)) {
                            selected_mesh_object_ = mesh_objects[i];
                            after_selecting_mesh();
                        }

                        if (is_selected)
                            ImGui::SetItemDefaultFocus();
                    }
                    ImGui::EndCombo();
                }
            }

            ImGui::Separator();

            // Drop the selection when the selected mesh is no longer valid for
            // this application (e.g. Perform changed its connectivity): the
            // combo box then falls back to "Select a mesh object" and the
            // widgets below stay disabled until a valid mesh is picked.
            // The selection has to be locked first: `lock()` on an expired
            // weak_ptr returns nullptr, so dereferencing it straight away
            // crashes as soon as no mesh is selected.
            if (const auto selected_object = selected_mesh_object_.lock()) {
                const auto mesh_object = std::dynamic_pointer_cast<MeshObject>(selected_object);
                if (mesh_object == nullptr || !is_mesh_valid(mesh_object->mesh()))
                    selected_mesh_object_.reset();
            }

            // When no mesh object is selected yet, everything below the combo
            // box (parameters and the perform button) is non-interactive.
            ImGui::BeginDisabled(selected_mesh_object_.expired());
            {
                draw_window_contents_details();
            }
            ImGui::EndDisabled();
        }

        std::vector<std::shared_ptr<BaseObject>>& objects_;

        /** The mesh object currently selected in the combo box. */
        std::weak_ptr<BaseObject> selected_mesh_object_;

        /** Width used by every slider/drag of the derived windows: 75% of the
         *  default item width. Refreshed at the beginning of each
         *  draw_window_contents(), so it follows window resizes; it must not
         *  be computed in the constructor (no ImGui frame is active then). */
        float slider_width_ = 0.0f;
    };
}

#endif //GEOLIO_MESH_APPLICATION_H
