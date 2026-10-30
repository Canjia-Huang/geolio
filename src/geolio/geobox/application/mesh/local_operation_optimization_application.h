//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/23.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_LOCAL_OPERATION_OPTIMIZATION_APPLICATION_H
#define GEOLIO_LOCAL_OPERATION_OPTIMIZATION_APPLICATION_H
#include "mesh_application.h"
#include <geolio/geobox/object/base_object.h>
#include <geogram/mesh/mesh.h>
#include "geolio/local_operation_optimization/tri_local_operation_optimization.h"

namespace geolio::geobox
{
    class LocalOperationOptimizationApplication : public MeshApplication {
    public:
        explicit LocalOperationOptimizationApplication(const std::vector<std::shared_ptr<BaseObject>>& objects)
            : MeshApplication("mesh optimization based on local operations", objects)
        {
            menu_tooltip_ = "Triangular mesh optimization based on local operations.";
        }

    protected:
        bool is_mesh_valid(const GEO::Mesh& mesh) override {
            return mesh.facets.nb() > 0 && mesh.facets.are_simplices();
        }

        void after_selecting_mesh() override {
            init_target_edge_length();
        }

        void draw_window_contents_details() override {
            // Rounds: drag input with a lower limit of 1 and no upper limit
            // (v_max <= v_min means unbounded; the value is clamped to >= 1
            // when the user releases the drag).
            {
                ImGui::SetNextItemWidth(slider_width_);

                int rounds = static_cast<int>(rounds_nb_);
                if (ImGui::DragInt("Rounds", &rounds, 0.05f, 1, 0))
                    rounds_nb_ = static_cast<GEO::index_t>(std::max(rounds, 1));
            }

            // Target edge length: drag input on the left for manual adjustment,
            // and a clickable text on the right that re-initializes the value
            // from the selected mesh's average edge length.
            {
                ImGui::SetNextItemWidth(slider_width_);

                constexpr double min_edge_length = 1e-4;
                if (ImGui::DragScalar(
                    "##target_edge_length", ImGuiDataType_Double,
                    &target_edge_length_, 0.0001f, &min_edge_length, nullptr,
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
                ImGui::SetNextItemWidth(slider_width_);
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
        }

        void init_target_edge_length() {
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
        void perform(GEO::Mesh& mesh) {
            LOG::TRACE("{}({})", application_name_, __FUNCTION__);
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

            LOG::INFO("Done {}({})!", application_name_, __FUNCTION__);
        }

        /* Parameters */
        GEO::index_t rounds_nb_ = 5;
        double target_edge_length_ = 1;
        bool fix_boundary_elements_ = true;
        bool fix_sharp_elements_ = true;
        double sharp_angle_ = 135;
        bool allow_split_fixed_edges_ = true;
        bool allow_collapse_fixed_edges_ = true;
        bool allow_smooth_fixed_edges_vertices_ = true;
    };
}

#endif //GEOLIO_LOCAL_OPERATION_OPTIMIZATION_APPLICATION_H
