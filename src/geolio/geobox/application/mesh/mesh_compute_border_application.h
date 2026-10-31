//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_MESH_COMPUTE_BORDER_APPLICATION_H
#define GEOLIO_MESH_COMPUTE_BORDER_APPLICATION_H
#include <geolio/geobox/application/base_application.h>
#include <geolio/common/log.h>

namespace geolio::geobox
{
    class MeshComputeBorderApplication : public BaseApplication {
    public:
        explicit MeshComputeBorderApplication(ApplicationObjects& objects)
            : BaseApplication("compute border", objects)
        {
            menu_tooltip_ = "GEO::Mesh.cells.compute_border()";
        }

    protected:
        void draw_window_contents() override {
            // The mesh picker, offering the meshes this application accepts.
            const bool selection_changed = draw_mesh_selector(
                selected_mesh_object_,
                [](const GEO::Mesh& mesh) {
                    return mesh.cells.nb() > 0;
                });

            ImGui::Separator();

            // When no mesh object is selected, everything below the picker
            // (the parameters and the button that runs the operation) is
            // non-interactive.
            ImGui::BeginDisabled(selected_mesh_object_.expired());
            if (const auto mesh_object = selected_mesh_object_.lock();
                mesh_object != nullptr
                ) {
                auto& mesh = mesh_object->mesh();

                if (ImGui::Button("Perform", ImVec2(-1.0f, 0.0f))) {
                    perform(mesh);
                    mesh_object->mesh_changed();
                }
            }
            ImGui::EndDisabled();
        }

        void perform(GEO::Mesh& mesh) const {
            LOG::TRACE("{}({})", application_name_, __FUNCTION__);

            mesh.cells.compute_borders();

            LOG::INFO("Done {}({})!", application_name_, __FUNCTION__);
        }

        /** The mesh object picked with draw_mesh_selector(). */
        std::weak_ptr<MeshObject> selected_mesh_object_;
    };
}

#endif //GEOLIO_MESH_COMPUTE_BORDER_APPLICATION_H
