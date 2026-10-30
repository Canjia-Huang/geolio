//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/29.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_MESH_REPAIR_TRIANGULATE_APPLICATION_H
#define GEOLIO_MESH_REPAIR_TRIANGULATE_APPLICATION_H
#include "mesh_application.h"
#include <geolio/geobox/object/mesh_object.h>
#include <geogram/mesh/mesh_repair.h>

namespace geolio::geobox
{
    class MeshRepairTriangulateApplication : public MeshApplication {
    public:
        explicit MeshRepairTriangulateApplication(const std::vector<std::shared_ptr<BaseObject>>& objects)
            : MeshApplication("triangulate", objects)
        {
            menu_tooltip_ = "GEO::mesh_repair(GEO::MESH_REPAIR_TRIANGULATE)";
        }

    protected:
        bool is_mesh_valid(const GEO::Mesh& mesh) override {
            return mesh.facets.nb() > 0 && !mesh.facets.are_simplices();
        }

        void draw_window_contents_details() override {
            if (ImGui::Button("Perform", ImVec2(-1.0f, 0.0f))) {
                // std::dynamic_pointer_cast needs a shared_ptr, so lock the
                // weak_ptr selection first.
                const auto selected_object = selected_mesh_object_.lock();
                const auto mesh_object = std::dynamic_pointer_cast<MeshObject>(selected_object);
                if (mesh_object != nullptr) {
                    auto& mesh = mesh_object->mesh();
                    perform(mesh);
                }
            }
        }

        void perform(GEO::Mesh& mesh) const {
            LOG::TRACE("{}", application_name_);

            GEO::mesh_repair(mesh, GEO::MESH_REPAIR_TRIANGULATE);

            LOG::INFO("Done {}!", application_name_);
        }
    };
}

#endif //GEOLIO_MESH_REPAIR_TRIANGULATE_APPLICATION_H
