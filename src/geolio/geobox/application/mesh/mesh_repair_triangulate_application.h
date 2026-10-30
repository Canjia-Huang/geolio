//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/29.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_MESH_REPAIR_TRIANGULATE_APPLICATION_H
#define GEOLIO_MESH_REPAIR_TRIANGULATE_APPLICATION_H
#include <geolio/geobox/application/base_application.h>
#include <geolio/geobox/object/mesh_object.h>
#include <geogram/mesh/mesh_repair.h>

namespace geolio::geobox
{
    class MeshRepairTriangulateApplication : public BaseApplication {
    public:
        explicit MeshRepairTriangulateApplication(std::vector<std::shared_ptr<BaseObject>>& objects)
            : BaseApplication("triangulate", objects)
        {
            menu_tooltip_ = "GEO::mesh_repair(..., GEO::MESH_REPAIR_TRIANGULATE)";
        }

    protected:
        void draw_window_contents() override {
            // The mesh picker, offering the meshes this application accepts.
            const bool selection_changed = draw_mesh_selector(
                [](const GEO::Mesh& mesh) {
                    return mesh.facets.nb() > 0 && !mesh.facets.are_simplices();
                });

            ImGui::Separator();

            // When no mesh object is selected, everything below the picker
            // (the parameters and the button that runs the operation) is
            // non-interactive.
            ImGui::BeginDisabled(selected_mesh_object_.expired());
            if (const auto mesh_object = selected_mesh();
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

            GEO::mesh_repair(mesh, GEO::MESH_REPAIR_TRIANGULATE);

            LOG::INFO("Done {}({})!", application_name_, __FUNCTION__);
        }
    };
}

#endif //GEOLIO_MESH_REPAIR_TRIANGULATE_APPLICATION_H
