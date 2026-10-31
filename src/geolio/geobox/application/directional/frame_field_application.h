//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/31.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_FRAME_FIELD_APPLICATION_H
#define GEOLIO_FRAME_FIELD_APPLICATION_H
#include <geolio/geobox/application/base_application.h>
#include <geolio/geobox/object/mesh_object.h>
#include <geolio/common/log.h>
#include <geolio/directional/frame_field.h>

namespace geolio::geobox
{
    class FrameFieldApplication : public BaseApplication {
    public:
        explicit FrameFieldApplication(ApplicationObjects& objects)
            : BaseApplication("frame field", objects)
        {
            menu_tooltip_ = "Generate frame field.";
        }

    protected:
        void draw_window_contents() override {
            // The mesh picker, offering the meshes this application accepts.
            const bool selection_changed = draw_mesh_selector(
                selected_mesh_object_,
                [](const GEO::Mesh& mesh) {
                    return (mesh.facets.nb() > 0 && mesh.facets.are_simplices()) ||
                           (mesh.cells.nb() > 0 && mesh.cells.are_simplices());
                });

            ImGui::Separator();

            // When no mesh object is selected, everything below the picker
            // (the parameters and the button that runs the operation) is
            // non-interactive.
            ImGui::BeginDisabled(selected_mesh_object_.expired());
            if (const auto mesh_object = selected_mesh_object_.lock();
                mesh_object != nullptr
                ) {
                const auto& mesh = mesh_object->mesh();

                {
                    const bool has_facets = mesh.facets.nb() > 0;
                    if (!has_facets)
                        volumetric_ = true;
                    const bool has_cells = mesh.cells.nb() > 0;
                    if (!has_cells)
                        volumetric_ = false;
                    ImGui::BeginDisabled(!has_cells || !has_facets);
                    {
                        ImGui::Checkbox("Volumetric", &volumetric_);
                    }
                    ImGui::EndDisabled();
                }

                ImGui::Separator();

                {
                    ImGui::Checkbox("Curvature weighted", &curvature_weighted_);
                    if (curvature_weighted_) {
                        ImGui::Indent();
                        ImGui::SetNextItemWidth(slider_width_);
                        ImGui::DragFloat("autoscale", &autoscale_, 0.001, 0, 1e8, "%.3f");
                        ImGui::Unindent();
                    }
                }

                ImGui::Separator();

                {
                    ImGui::SetNextItemWidth(slider_width_);
                    ImGui::SliderFloat("Sharp angle", &sharp_angle_threshold_, 0, 180, "%.1f");
                }

                ImGui::Separator();

                if (ImGui::Button("Perform", ImVec2(-1.0f, 0.0f)))
                    perform(mesh);
                }
            ImGui::EndDisabled();
        }

        void perform(const GEO::Mesh& mesh) const {
            LOG::TRACE("{}({})", application_name_, __FUNCTION__);

            FrameField frame_field;
            if (curvature_weighted_)
                frame_field.create_curvature_directions(mesh, volumetric_, sharp_angle_threshold_, autoscale_);
            else
                frame_field.create_from_surface_mesh(mesh, volumetric_, sharp_angle_threshold_);

            // TODO: add object

            LOG::INFO("Done {}({})!", application_name_, __FUNCTION__);
        }

        bool volumetric_ = false;
        bool curvature_weighted_ = false;
        float autoscale_ = 0.0;
        float sharp_angle_threshold_ = 45;

        /** The mesh object picked with draw_mesh_selector(). */
        std::weak_ptr<MeshObject> selected_mesh_object_;
    };
}

#endif //GEOLIO_FRAME_FIELD_APPLICATION_H
