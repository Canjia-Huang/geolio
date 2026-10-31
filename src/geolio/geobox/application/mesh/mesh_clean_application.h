//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/31.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_MESH_CLEAN_APPLICATION_H
#define GEOLIO_MESH_CLEAN_APPLICATION_H
#include <geolio/geobox/application/base_application.h>
#include <geolio/geobox/object/mesh_object.h>
#include <geolio/common/log.h>

namespace geolio::geobox
{
    class MeshCleanApplication : public BaseApplication {
    public:
        explicit MeshCleanApplication(ApplicationObjects& objects)
            : BaseApplication("clean", objects)
        {
            menu_tooltip_ = "Clean mesh elements";
        }

    protected:
        void draw_window_contents() override {
            // The mesh picker, offering the meshes this application accepts.
            const bool selection_changed = draw_mesh_selector(
                selected_mesh_object_,
                [](const GEO::Mesh& mesh) {
                    return mesh.vertices.nb() > 0;
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

                if (ImGui::Button("Clean isolated vertices", ImVec2(-1.0f, 0.0f)))
                    mesh.vertices.remove_isolated();

                ImGui::BeginDisabled(mesh.edges.nb() == 0);
                {
                    if (ImGui::Button("Clean edges", ImVec2(-1.0f, 0.0f)))
                        mesh.edges.clear();
                }
                ImGui::EndDisabled();

                ImGui::BeginDisabled(mesh.facets.nb() == 0);
                {
                    if (ImGui::Button("Clean facets", ImVec2(-1.0f, 0.0f)))
                        mesh.facets.clear();
                }
                ImGui::EndDisabled();

                ImGui::BeginDisabled(mesh.cells.nb() == 0);
                {
                    if (ImGui::Button("Clean cells", ImVec2(-1.0f, 0.0f)))
                        mesh.cells.clear();
                }
                ImGui::EndDisabled();
            }
            ImGui::EndDisabled();
        }

        std::weak_ptr<MeshObject> selected_mesh_object_;
    };
}

#endif //GEOLIO_MESH_CLEAN_APPLICATION_H
