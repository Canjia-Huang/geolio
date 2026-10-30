//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_MESH_CONNECT_APPLICATION_H
#define GEOLIO_MESH_CONNECT_APPLICATION_H
#include <geolio/geobox/application/base_application.h>
#include <geolio/geobox/object/mesh_object.h>
#include <geolio/common/log.h>

namespace geolio::geobox
{
    class MeshConnectApplication : public BaseApplication {
    public:
        explicit MeshConnectApplication(std::vector<std::shared_ptr<BaseObject>>& objects)
            : BaseApplication("connect", objects)
        {
            menu_tooltip_ = "GEO::mesh.facets/cells.connect()";
        }

    protected:
        void draw_window_contents() override {
            // The mesh picker, offering the meshes this application accepts.
            const bool selection_changed = draw_mesh_selector(
                [](const GEO::Mesh& mesh) {
                    return mesh.facets.nb() > 0 || mesh.cells.nb() > 0;
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

                const bool has_facets = mesh.facets.nb() > 0;
                const bool has_cells = mesh.cells.nb() > 0;

                ImGui::BeginDisabled(!has_facets);
                {
                    if (ImGui::Button("Connect facets", ImVec2(-1.0f, 0.0f))) {
                        perform_connect_facets(mesh);
                        mesh_object->mesh_changed();
                    }

                    if (ImGui::Button("Disconnect facets", ImVec2(-1.0f, 0.0f))) {
                        perform_disconnect_facets(mesh);
                        mesh_object->mesh_changed();
                    }
                }
                ImGui::EndDisabled();

                ImGui::Separator();

                ImGui::BeginDisabled(!has_cells);
                {
                    if (ImGui::Button("Connect cells", ImVec2(-1.0f, 0.0f))) {
                        perform_connect_cells(mesh);
                        mesh_object->mesh_changed();
                    }

                    if (ImGui::Button("Disconnect cells", ImVec2(-1.0f, 0.0f))) {
                        perform_disconnect_cells(mesh);
                        mesh_object->mesh_changed();
                    }
                }
                ImGui::EndDisabled();
            }
            ImGui::EndDisabled();
        }

        void perform_connect_facets(GEO::Mesh& mesh) const {
            LOG::TRACE("{}({})", application_name_, __FUNCTION__);

            mesh.facets.connect();

            LOG::INFO("Done {}({})!", application_name_, __FUNCTION__);
        }

        void perform_disconnect_facets(GEO::Mesh& mesh) const {
            LOG::TRACE("{}({})", application_name_, __FUNCTION__);

            for (const auto& f : mesh.facets) {
                for (const auto& fc : mesh.facets.corners(f))
                    mesh.facet_corners.set_adjacent_facet(fc, GEO::NO_FACET);
            }

            LOG::INFO("Done {}({})!", application_name_, __FUNCTION__);
        }

        void perform_connect_cells(GEO::Mesh& mesh) const {
            LOG::TRACE("{}({})", application_name_, __FUNCTION__);

            mesh.cells.connect();

            LOG::INFO("Done {}({})!", application_name_, __FUNCTION__);
        }

        void perform_disconnect_cells(GEO::Mesh& mesh) const {
            LOG::TRACE("{}({})", application_name_, __FUNCTION__);

            for (const auto& c : mesh.cells) {
                for (const auto& cf : mesh.cells.facets(c))
                    mesh.cell_facets.set_adjacent_cell(cf, GEO::NO_CELL);
            }

            LOG::INFO("Done {}({})!", application_name_, __FUNCTION__);
        }
    };
}
#endif //GEOLIO_MESH_CONNECT_APPLICATION_H
