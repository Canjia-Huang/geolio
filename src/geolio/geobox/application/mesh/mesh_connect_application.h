//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_MESH_CONNECT_APPLICATION_H
#define GEOLIO_MESH_CONNECT_APPLICATION_H
#include "mesh_application.h"
#include <geolio/geobox/object/mesh_object.h>
#include <geolio/common/log.h>

namespace geolio::geobox
{
    class MeshConnectApplication : public MeshApplication {
    public:
        explicit MeshConnectApplication(std::vector<std::shared_ptr<BaseObject>>& objects)
            : MeshApplication("connect", objects)
        {
            menu_tooltip_ = "GEO::mesh.facets/cells.connect()";
        }

    protected:
        bool is_mesh_valid(const GEO::Mesh& mesh) override {
            return mesh.facets.nb() > 0 || mesh.cells.nb() > 0;
        }

        void draw_window_contents_details() override {
            if (ImGui::Button("Connect facets", ImVec2(-1.0f, 0.0f))) {
                // std::dynamic_pointer_cast needs a shared_ptr, so lock the
                // weak_ptr selection first.
                const auto selected_object = selected_mesh_object_.lock();
                const auto mesh_object = std::dynamic_pointer_cast<MeshObject>(selected_object);
                if (mesh_object != nullptr) {
                    auto& mesh = mesh_object->mesh();
                    perform_connect_facets(mesh);
                }
            }

            if (ImGui::Button("Connect cells", ImVec2(-1.0f, 0.0f))) {
                // std::dynamic_pointer_cast needs a shared_ptr, so lock the
                // weak_ptr selection first.
                const auto selected_object = selected_mesh_object_.lock();
                const auto mesh_object = std::dynamic_pointer_cast<MeshObject>(selected_object);
                if (mesh_object != nullptr) {
                    auto& mesh = mesh_object->mesh();
                    perform_connect_cells(mesh);
                }
            }

            if (ImGui::Button("Disconnect facets", ImVec2(-1.0f, 0.0f))) {
                // std::dynamic_pointer_cast needs a shared_ptr, so lock the
                // weak_ptr selection first.
                const auto selected_object = selected_mesh_object_.lock();
                const auto mesh_object = std::dynamic_pointer_cast<MeshObject>(selected_object);
                if (mesh_object != nullptr) {
                    auto& mesh = mesh_object->mesh();
                    perform_disconnect_facets(mesh);
                }
            }

            if (ImGui::Button("Disconnect cells", ImVec2(-1.0f, 0.0f))) {
                // std::dynamic_pointer_cast needs a shared_ptr, so lock the
                // weak_ptr selection first.
                const auto selected_object = selected_mesh_object_.lock();
                const auto mesh_object = std::dynamic_pointer_cast<MeshObject>(selected_object);
                if (mesh_object != nullptr) {
                    auto& mesh = mesh_object->mesh();
                    perform_disconnect_cells(mesh);
                }
            }
        }

        void perform_connect_facets(GEO::Mesh& mesh) const {
            LOG::TRACE("{}({})", application_name_, __FUNCTION__);

            mesh.facets.connect();

            LOG::INFO("Done {}({})!", application_name_, __FUNCTION__);
        }

        void perform_connect_cells(GEO::Mesh& mesh) const {
            LOG::TRACE("{}({})", application_name_, __FUNCTION__);

            mesh.cells.connect();

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
