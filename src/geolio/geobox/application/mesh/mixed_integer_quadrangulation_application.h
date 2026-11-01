//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/11/1.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_MIXED_INTEGER_QUADRANGULATION_APPLICATION_H
#define GEOLIO_MIXED_INTEGER_QUADRANGULATION_APPLICATION_H
#include <geolio/geobox/application/base_application.h>
#include <geolio/geobox/object/mesh_object.h>
#include <geolio/common/log.h>
#include <geolio/MIQ/miq_interface.h>

namespace geolio::geobox
{
    class MixedIntegerQuadratulationApplication : public BaseApplication {
    public:
        explicit MixedIntegerQuadratulationApplication(ApplicationObjects& objects)
            : BaseApplication("mixed integer quadrangulation", objects)
        {
            menu_tooltip_ = "`[Bommes2009] Mixed-integer quadrangulation` interface";
        }

    protected:
        void draw_window_contents() override {
            // The mesh picker, offering the meshes this application accepts.
            const bool selection_changed = draw_mesh_selector(
                selected_mesh_object_,
                [](const GEO::Mesh& mesh) {
                    return mesh.facets.nb() > 0 && mesh.facets.are_simplices();
                });

            ImGui::Separator();

            ImGui::BeginDisabled(selected_mesh_object_.expired());
            if (const auto mesh_object = selected_mesh_object_.lock();
                mesh_object != nullptr
                ) {
                {
                    ImGui::Checkbox("Use frame field", &use_precomputed_frame_field_);
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("Use a precomputed frame field, or automatically compute 4-RoSy directional field");
                    if (use_precomputed_frame_field_) {
                        draw_directional_selector(
                            selected_directional_object_,
                            [&mesh_object](const DirectionalObject& directional) {
                                return directional.attached() &&
                                       directional.subelements() == GEO::MESH_FACETS &&
                                       directional.attached_mesh() == &(mesh_object->mesh());
                            });
                    }
                }

                ImGui::Separator();

                {
                    ImGui::SetNextItemWidth(slider_width_);
                    ImGui::DragFloat("Scale", &scale_, 0.001, 0.001, 1e8);
                }

                ImGui::Separator();

                {
                    ImGui::Checkbox("Snap boundary", &snap_boundary_);
                }

                ImGui::Separator();

                const auto& mesh = mesh_object->mesh();
                if (ImGui::Button("Perform", ImVec2(-1.0f, 0.0f))) {
                    perform(mesh);
                    mesh_object->set_painting_mode(PaintingMode::TEXTURE);
                }
            }
            ImGui::EndDisabled();
        }

        void perform(const GEO::Mesh& mesh) const {
            LOG::TRACE("{}({})", application_name_, __FUNCTION__);

            GEO::Attribute<double> mesh_fc_uv;
            mesh_fc_uv.bind_if_is_defined(mesh.facet_corners.attributes(), "tex_coord");
            if (mesh_fc_uv.is_bound() && mesh_fc_uv.dimension() != 2)
                mesh_fc_uv.destroy();
            if (!mesh_fc_uv.is_bound())
                mesh_fc_uv.create_vector_attribute(mesh.facet_corners.attributes(), "tex_coord", 2);
            assert(mesh_fc_uv.is_bound());

            geolio::MIQParameters params;
            if (use_precomputed_frame_field_) {
                const auto directional_object = selected_directional_object_.lock();
                if (directional_object != nullptr) {
                    params.cross = directional_object->frames();
                    params.vectors_nb_per_facet = directional_object->vectors_nb_per_element();
                }
            }
            else
                params.output_cross = true;
            params.scale = scale_;
            params.snap_boundary = snap_boundary_;

            if (mesh.vertices.dimension() == 2)
                miq<2>(mesh, mesh_fc_uv, params);
            else
                miq<3>(mesh, mesh_fc_uv, params);

            /* Create directional object */
            if (params.output_cross) {
                const auto directional_object = std::make_shared<DirectionalObject>(
                    "cross field", "", objects_, 2, params.out_cross);
                objects_.push_back(directional_object);
            }

            LOG::INFO("Done {}({})!", application_name_, __FUNCTION__);
        }

        /** The mesh object picked with draw_mesh_selector(). */
        std::weak_ptr<MeshObject> selected_mesh_object_;

        bool use_precomputed_frame_field_ = false;
        std::weak_ptr<DirectionalObject> selected_directional_object_;

        float scale_ = 30.0;
        bool snap_boundary_ = true;
    };
}

#endif //GEOLIO_MIXED_INTEGER_QUADRANGULATION_APPLICATION_H
