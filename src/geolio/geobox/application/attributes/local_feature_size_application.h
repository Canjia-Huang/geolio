//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/31.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_LOCAL_FEATURE_SIZE_APPLICATION_H
#define GEOLIO_LOCAL_FEATURE_SIZE_APPLICATION_H
#include <geogram/mesh/mesh_geometry.h>
#include <geolio/geobox/application/base_application.h>
#include <geolio/geobox/object/mesh_object.h>
#include <geolio/common/log.h>

namespace geolio::geobox
{
    class LocalFeatureSizeApplication : public BaseApplication {
    public:
        explicit LocalFeatureSizeApplication(std::vector<std::shared_ptr<BaseObject>>& objects)
            : BaseApplication("local feature size", objects)
        {
            menu_tooltip_ = "GEO::mesh.facets/cells.connect()";
        }

    protected:
        void draw_window_contents() override {
            // The mesh picker, offering the meshes this application accepts.
            const bool selection_changed = draw_mesh_selector(
                selected_mesh_object_,
                [](const GEO::Mesh& mesh) {
                    return mesh.facets.nb() > 0;
                });

            ImGui::Separator();

            // When no mesh object is selected, everything below the picker
            // (the parameters and the button that runs the operation) is
            // non-interactive.
            ImGui::BeginDisabled(selected_mesh_object_.expired());
            if (const auto mesh_object = selected_mesh_object_.lock();
                mesh_object != nullptr
                ) {
                {
                    ImGui::SetNextItemWidth(slider_width_);

                    ImGui::DragFloat(
                        "Gradation", &gradation_,
                        0.001, 0, 1e8,
                        "%.3f");
                    ImGui::SetItemTooltip("The exponent to be applied to the sizing field");
                }

                ImGui::Separator();

                {
                    ImGui::SetNextItemWidth(slider_width_);

                    ImGui::DragInt(
                        "LFS samples nb",
                        &nb_lfs_samples_, 1, 0, 1e8);
                    ImGui::SetItemTooltip("If set to 0, the vertices of the mesh are used, else mesh is resampled (needed if mesh's facets density is highly irregular).");
                }

                ImGui::Separator();

                if (ImGui::Button("Perform", ImVec2(-1.0f, 0.0f))) {
                    perform(mesh_object->mesh());

                    // GEO::compute_sizing_field() stores the sizing field in the
                    // "weight" attribute of the vertices: show it, together with
                    // the vertices it is defined on, so that the result of the
                    // operation is what the user sees.
                    mesh_object->mesh_changed();
                    if (!mesh_object->display_attribute(kWeightAttribute))
                        LOG::WARN("The mesh has no `{}` attribute to display.", kWeightAttribute);
                }
            }
            ImGui::EndDisabled();
        }

        void perform(GEO::Mesh& mesh) const {
            LOG::TRACE("{}({})", application_name_, __FUNCTION__);

            GEO::compute_sizing_field(mesh, gradation_, nb_lfs_samples_);

            LOG::INFO("Done {}({})!", application_name_, __FUNCTION__);
        }

        /** The mesh object picked with draw_mesh_selector(). */
        std::weak_ptr<MeshObject> selected_mesh_object_;

        /** Attribute GEO::compute_sizing_field() stores the sizing field in
         *  (see its documentation: "the sizing field is stored into the vertex
         *  weights"). */
        static constexpr auto kWeightAttribute = "vertices.weight";

        float gradation_ = 1;
        int nb_lfs_samples_ = 0;
    };
}

#endif //GEOLIO_LOCAL_FEATURE_SIZE_APPLICATION_H
