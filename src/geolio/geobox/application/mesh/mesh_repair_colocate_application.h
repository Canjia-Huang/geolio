//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_MESH_REPAIR_COLOCATE_APPLICATION_H
#define GEOLIO_MESH_REPAIR_COLOCATE_APPLICATION_H
#include "mesh_application.h"
#include <geolio/geobox/object/mesh_object.h>
#include <geogram/points/colocate.h>

namespace geolio::geobox
{
    class MeshRepairColocateApplication : public MeshApplication {
    public:
        explicit MeshRepairColocateApplication(const std::vector<std::shared_ptr<BaseObject>>& objects)
            : MeshApplication("colocate", objects)
        {
            menu_tooltip_ = "GEO::mesh_colocate_vertices_no_check(...)";
        }

    protected:
        bool is_mesh_valid(const GEO::Mesh& mesh) override {
            return mesh.vertices.nb() > 0;
        }

        void draw_window_contents_details() override {
            {
                ImGui::SetNextItemWidth(slider_width_);

                constexpr double min_colocate_epsilon = 0;
                ImGui::DragScalar(
                    "Colocate epsilon", ImGuiDataType_Double,
                    &colocate_epsilon, 0.0001f, &min_colocate_epsilon, nullptr,
                    "%.4f");
            }

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
            LOG::TRACE("{}({})", application_name_, __FUNCTION__);

            GEO::vector<GEO::index_t> old2new;

            GEO::index_t nb_new_vertices = 0;
            if(colocate_epsilon == 0.0)
                nb_new_vertices = GEO::Geom::colocate_by_lexico_sort(
                    mesh.vertices.point_ptr(0),
                    3,
                    mesh.vertices.nb(),
                    old2new,
                    mesh.vertices.dimension());
            else
                nb_new_vertices = GEO::Geom::colocate(
                    mesh.vertices.point_ptr(0),
                    3,
                    mesh.vertices.nb(),
                    old2new,
                    colocate_epsilon,
                    mesh.vertices.dimension());

            if(nb_new_vertices == mesh.vertices.nb()) {
                LOG::DEBUG("No duplicated vertices to remove.");
                return;
            }

            LOG::DEBUG("Removed {} duplicated vertices.", mesh.vertices.nb() - nb_new_vertices);

            // Replace vertex indices for edges
            for(const auto&  e: mesh.edges) {
                mesh.edges.set_vertex(e, 0, old2new[mesh.edges.vertex(e,0)]);
                mesh.edges.set_vertex(e, 1, old2new[mesh.edges.vertex(e,1)]);
            }

            // Replace vertex indices for facets
            for(const auto& c: mesh.facet_corners)
                mesh.facet_corners.set_vertex(c, old2new[mesh.facet_corners.vertex(c)]);

            // Replace vertex indices for cells
            for(const auto& ce: mesh.cells) {
                for(const auto& c: mesh.cells.corners(ce))
                    mesh.cell_corners.set_vertex(c, old2new[mesh.cell_corners.vertex(c)]);
            }

            // Now old2new is "recycled" for marking vertices that
            // need to be removed.
            for(GEO::index_t i = 0; i < old2new.size(); i++) {
                if(old2new[i] == i)
                    old2new[i] = 0;
                else
                    old2new[i] = 1;
            }
            mesh.vertices.delete_elements(old2new);
            for(const auto& fc: mesh.facet_corners)
                mesh.facet_corners.set_adjacent_facet(fc, GEO::NO_FACET);

            LOG::INFO("Done {}({})!", application_name_, __FUNCTION__);
        }

        double colocate_epsilon = 0.0;
    };
}

#endif //GEOLIO_MESH_REPAIR_COLOCATE_APPLICATION_H
