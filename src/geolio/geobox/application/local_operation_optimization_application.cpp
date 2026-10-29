//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/23.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "local_operation_optimization_application.h"
#include <algorithm>
#include <utility>

#include "geolio/common/log.h"
#include "geolio/geobox/object/mesh_object.h"
#include "geolio/local_operation_optimization/tri_local_operation_optimization.h"

namespace geolio::geobox
{
    LocalOperationOptimizationApplication::LocalOperationOptimizationApplication(
        const std::vector<std::shared_ptr<BaseObject>>& objects
        ) : BaseApplication("mesh optimization based on local operations"),
            objects_(objects)
    {
        menu_tooltip_ = "Triangular mesh optimization based on local operations.";
    }

    void LocalOperationOptimizationApplication::draw_window_contents(
        ) {
        // Collect the objects whose type is a mesh ("Mesh", see MeshObject).
        std::vector<std::shared_ptr<MeshObject>> mesh_objects;
        for (const auto& object : objects_) {
            const auto mesh_object = std::dynamic_pointer_cast<MeshObject>(object);
            if (mesh_object == nullptr)
                continue;
            if (const GEO::Mesh& mesh = mesh_object->mesh();
                mesh.facets.nb() > 0 && mesh.facets.are_simplices())
                mesh_objects.push_back(mesh_object);
        }

        // When only one triangular mesh object exists, select it
        // automatically (and initialize the target edge length from it),
        // unless a mesh is already selected.
        if (mesh_objects.size() == 1 && selected_mesh_object_.expired()) {
            selected_mesh_object_ = mesh_objects[0];
            init_target_edge_length();
        }

        // Selection combo box: pick one mesh object.
        if (const bool has_mesh_object = !mesh_objects.empty();
            !has_mesh_object)
            ImGui::TextUnformatted("No triangular mesh object available.");
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

                        /* Update target edge length */
                        init_target_edge_length();
                    }

                    if (is_selected)
                        ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();
            }
        }

        // When no mesh object is selected yet, everything below the combo
        // box (parameters and the perform button) is non-interactive.
        ImGui::BeginDisabled(selected_mesh_object_.expired());

        ImGui::Separator();

        // All numeric sliders/drags in this window are drawn at 75% of the
        // default (full available width) item width.
        const float slider_width = ImGui::CalcItemWidth() * 0.75f;

        // Rounds: drag input with a lower limit of 1 and no upper limit
        // (v_max <= v_min means unbounded; the value is clamped to >= 1
        // when the user releases the drag).
        {
            int rounds = static_cast<int>(rounds_nb_);
            ImGui::SetNextItemWidth(slider_width);
            if (ImGui::DragInt("Rounds", &rounds, 0.05f, 1, 0))
                rounds_nb_ = static_cast<GEO::index_t>(std::max(rounds, 1));
        }

        // Target edge length: drag input on the left for manual adjustment,
        // and a clickable text on the right that re-initializes the value
        // from the selected mesh's average edge length.
        {
            constexpr double min_edge_length = 1e-4;
            ImGui::SetNextItemWidth(slider_width);
            if (ImGui::DragScalar(
                "##target_edge_length", ImGuiDataType_Double,
                &target_edge_length_, 0.001f, &min_edge_length, nullptr,
                "%.4f"))
                target_edge_length_ = std::max(target_edge_length_, min_edge_length);

            ImGui::SameLine();
            // The rest of the row is clickable text, vertically centered
            // like the buttons; clicking it re-initializes the value.
            ImGui::PushStyleVar(
                ImGuiStyleVar_SelectableTextAlign, ImVec2(0.0f, 0.5f));
            if (ImGui::Selectable(
                "Target edge length", false, 0,
                ImVec2(0.0f, ImGui::GetFrameHeight())))
                init_target_edge_length();
            ImGui::PopStyleVar();
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(
                    "Click to re-initialize from the selected mesh's average edge length");
        }

        ImGui::Separator();
        ImGui::Checkbox("Fix bdy. elements", &fix_boundary_elements_);
        ImGui::Checkbox("Fix sharp elements", &fix_sharp_elements_);
        if (fix_sharp_elements_) {
            // Sharp angle in degrees, only relevant when sharp elements are
            // fixed; clamped to [0, 180].
            constexpr double min_angle = 0.0;
            constexpr double max_angle = 180.0;
            ImGui::SetNextItemWidth(slider_width);
            ImGui::SliderScalar(
                "Sharp angle (deg)", ImGuiDataType_Double,
                &sharp_angle_, &min_angle, &max_angle, "%.1f");
        }
        if (fix_boundary_elements_ || fix_sharp_elements_) {
            ImGui::Indent();

            ImGui::Checkbox("Allow split fixed edges", &allow_split_fixed_edges_);
            ImGui::Checkbox("Allow collapse fixed edges", &allow_collapse_fixed_edges_);
            ImGui::Checkbox("Allow smooth fixed edges/vertices", &allow_smooth_fixed_edges_vertices_);

            ImGui::Unindent();
        }

        // Full-width button that triggers the optimization.
        ImGui::Separator();
        if (ImGui::Button("Perform", ImVec2(-1.0f, 0.0f))) {
            // std::dynamic_pointer_cast needs a shared_ptr, so lock the
            // weak_ptr selection first.
            const auto selected_object = selected_mesh_object_.lock();
            const auto mesh_object = std::dynamic_pointer_cast<MeshObject>(selected_object);
            if (mesh_object != nullptr) {
                if (auto& mesh = mesh_object->mesh();
                    mesh.vertices.dimension() == 2)
                    perform<2>(mesh);
                else
                    perform<3>(mesh);
            }
        }
        ImGui::EndDisabled();
    }

    void LocalOperationOptimizationApplication::init_target_edge_length(
        ) {
        const auto selected_object = selected_mesh_object_.lock();
        if (selected_object == nullptr) {
            target_edge_length_ = 1;
            return;
        }

        const auto mesh_object = std::dynamic_pointer_cast<MeshObject>(selected_object);
        assert(mesh_object != nullptr);
        if (auto& mesh = mesh_object->mesh();
            mesh.vertices.dimension() == 2
            ) {
            TriLocalOperationOptimization<2> TLOO(mesh);
            target_edge_length_ = TLOO.compute_average_edge_length();
            }
        else {
            TriLocalOperationOptimization<3> TLOO(mesh);
            target_edge_length_ = TLOO.compute_average_edge_length();
        }
    }

    template <GEO::index_t DIM>
    void LocalOperationOptimizationApplication::perform(
        GEO::Mesh& mesh
        ) {
        assert(mesh.facets.nb() > 0);
        assert(mesh.facets.are_simplices());

        TriLocalOperationOptimization<DIM> TLOO(mesh);
        if (fix_boundary_elements_)
            TLOO.fix_boundary_elements();
        if (fix_sharp_elements_)
            TLOO.fix_sharp_elements(sharp_angle_ * M_PI / 180.0);
        TLOO.allow_split_fixed_edges = allow_split_fixed_edges_;
        TLOO.allow_collapse_fixed_edges = allow_collapse_fixed_edges_;
        TLOO.allow_smooth_fixed_edges_vertices = allow_smooth_fixed_edges_vertices_;
        TLOO.optimize(rounds_nb_, target_edge_length_);
    }

    template void LocalOperationOptimizationApplication::perform<2>(GEO::Mesh& mesh);
    template void LocalOperationOptimizationApplication::perform<3>(GEO::Mesh& mesh);
}
