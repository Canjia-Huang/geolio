//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/11/16.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_MOTORCYCLE_COMPLEX_APPLICATION_H
#define GEOLIO_MOTORCYCLE_COMPLEX_APPLICATION_H
#include <geolio/geobox/application/base_application.h>
#include <geolio/common/log.h>
#include <geolio/motorcycle/quad_motorcycle_graph.h>
#include <geolio/geobox/colormap.h>

#include "geolio/motorcycle/hex_motorcycle_complex.h"

namespace geolio::geobox
{
    class MotorcycleComplexApplication : public BaseApplication {
    public:
        explicit MotorcycleComplexApplication(ApplicationObjects& objects)
            : BaseApplication("motorcycle complex", objects)
        {
            menu_tooltip_ = "Compute the motorcycle graph/complex of a quad/hex mesh.";
        }

    protected:
        void draw_window_contents() override {
            auto is_all_quad_mesh = [](const GEO::Mesh& mesh)->bool {
                if (mesh.facets.are_simplices())
                    return false;

                for (const auto& f : mesh.facets) {
                    if (mesh.facets.nb_vertices(f) != 4)
                        return false;
                }
                return true;
            };

            auto is_all_hex_mesh = [](const GEO::Mesh& mesh)->bool {
                if (mesh.cells.are_simplices())
                    return false;

                return std::all_of(
                        mesh.cells.cell_type_ptr(0),
                        mesh.cells.cell_type_ptr(0)+mesh.cells.nb(),
                        [](const auto& type){ return type == GEO::MESH_HEX; });
            };

            // The mesh picker, offering the meshes this application accepts.
            const bool selection_changed = draw_mesh_selector(
                selected_mesh_object_,
                [&](const GEO::Mesh& mesh) {
                    return is_all_quad_mesh(mesh) || is_all_hex_mesh(mesh);
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
                const bool all_quad = is_all_quad_mesh(mesh);
                const bool all_hex = is_all_hex_mesh(mesh);

                {
                    static const char* complex_types[] = { "motorcycle", "base" };
                    ImGui::Combo("complex type", &complex_type_, complex_types, IM_ARRAYSIZE(complex_types));
                }

                ImGui::Checkbox("create coarse mesh", &create_coarse_mesh_);

                ImGui::BeginDisabled(!all_quad);
                {
                    if (ImGui::Button("Compute MotorcycleGraph", ImVec2(-1.0f, 0.0f))) {
                        perform_motorcycle_graph(mesh);
                        mesh_object->mesh_changed();
                        mesh_object->display_attribute("facets."+id_+":block");
                        mesh_object->set_colormap(RANDOM_COLORMAP_NAME);
                    }
                }
                ImGui::EndDisabled();

                ImGui::BeginDisabled(!all_hex);
                {
                    if (ImGui::Button("Compute MotorcycleComplex", ImVec2(-1.0f, 0.0f))) {
                        perform_motorcycle_complex(mesh);
                        mesh_object->mesh_changed();
                        mesh_object->display_attribute("cells."+id_+":block");
                        mesh_object->set_colormap(RANDOM_COLORMAP_NAME);
                    }
                }
                ImGui::EndDisabled();
            }
            ImGui::EndDisabled();
        }

        void perform_motorcycle_graph(const GEO::Mesh& mesh) const {
            LOG::TRACE("{}({})", application_name_, __FUNCTION__);

            QuadMotorCycleGraph MC(mesh);
            if (complex_type_)
                MC.compute(QuadMotorCycleGraph::BASE_COMPLEX);
            else
                MC.compute(QuadMotorCycleGraph::MOTORCYCLE_COMPLEX);

            GEO::Attribute<GEO::index_t> mesh_f_block;
            mesh_f_block.bind(mesh.facets.attributes(), id_+":block");
            MC.label_blocks(mesh_f_block);

            if (create_coarse_mesh_) {
                GEO::Mesh coarse_mesh;
                MC.create_coarse_mesh(coarse_mesh);

                const auto object = std::make_shared<MeshObject>(
                    "coarse mesh", "", coarse_mesh);
                objects_.push_back(object);
            }

            LOG::INFO("Done {}({})!", application_name_, __FUNCTION__);
        }

        void perform_motorcycle_complex(const GEO::Mesh& mesh) const {
            LOG::TRACE("{}({})", application_name_, __FUNCTION__);

            HexMotorCycleComplex MC(mesh);
            if (complex_type_)
                MC.compute(HexMotorCycleComplex::BASE_COMPLEX);
            else
                MC.compute(HexMotorCycleComplex::MOTORCYCLE_COMPLEX);

            GEO::Attribute<GEO::index_t> mesh_c_block;
            mesh_c_block.bind(mesh.cells.attributes(), id_+":block");
            MC.label_blocks(mesh_c_block);

            if (create_coarse_mesh_) {
                GEO::Mesh coarse_mesh;
                MC.create_coarse_mesh(coarse_mesh);

                const auto object = std::make_shared<MeshObject>(
                    "coarse mesh", "", coarse_mesh);
                objects_.push_back(object);
            }

            LOG::INFO("Done {}({})!", application_name_, __FUNCTION__);
        }

        /** The mesh object picked with draw_mesh_selector(). */
        std::weak_ptr<MeshObject> selected_mesh_object_;

        int complex_type_ = 0; // 0: motorcycle, 1: base
        bool create_coarse_mesh_ = false;
    };
}

#endif //GEOLIO_MOTORCYCLE_COMPLEX_APPLICATION_H
