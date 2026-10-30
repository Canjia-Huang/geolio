//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_MESH_RANDOM_SAMPLING_APPLICATION_H
#define GEOLIO_MESH_RANDOM_SAMPLING_APPLICATION_H
#include "mesh_application.h"
#include <geolio/geobox/object/mesh_object.h>
#include <geolio/common/log.h>
#include <geogram/mesh/mesh_sampling.h>

namespace geolio::geobox
{
    class MeshRandomSamplingApplication : public MeshApplication {
    public:
        explicit MeshRandomSamplingApplication(std::vector<std::shared_ptr<BaseObject>>& objects)
            : MeshApplication("random sampling", objects)
        {
            menu_tooltip_ = "Random sampling on mesh.";
        }

    protected:
        bool is_mesh_valid(const GEO::Mesh& mesh) override {
            return mesh.facets.nb() > 0 || mesh.cells.nb() > 0;
        }

        void draw_window_contents_details() override {
            {
                ImGui::SetNextItemWidth(slider_width_);

                ImGui::DragInt(
                    "Samples nb",
                    &samples_nb, 1, 0, 1e8);
            }

            ImGui::Separator();

            {
                // Sampling weight: the sampler reads the selected attribute to
                // make the number of samples of a triangle proportional to its
                // weighted area (see
                // GEO::mesh_generate_random_samples_on_surface(), which takes
                // an Attribute<double> on the vertices). Only the per-vertex
                // attributes holding one double per vertex can be bound to
                // mesh_v_weight_, so those are the only ones offered here.
                std::vector<std::string> attribute_names;
                if (const auto mesh_object = std::dynamic_pointer_cast<MeshObject>(selected_mesh_object_.lock());
                    mesh_object != nullptr)
                    GEO::String::split_string(
                        mesh_object->vertex_scalar_attribute_names(), ';', attribute_names);

                if (attribute_names.empty()) {
                    ImGui::TextUnformatted("No vertex attribute available.");
                }
                else {
                    int selected = 0;
                    for (std::size_t i = 1; i < attribute_names.size(); ++i) {
                        if (attribute_names[i] == vertex_weight_attribute_name_)
                            selected = static_cast<int>(i);
                    }
                    // The selected attribute is gone (another mesh was selected, or
                    // the mesh was reloaded without it): fall back to "none".
                    if (selected == 0)
                        vertex_weight_attribute_name_.clear();

                    std::string items;
                    for (const std::string& name : attribute_names) {
                        items += name;
                        items.push_back('\0');
                    }

                    ImGui::Separator();

                    ImGui::Checkbox("Weighted by vertex attribute", &vertex_weighted_);

                    if (vertex_weighted_) {
                        ImGui::AlignTextToFramePadding();
                        ImGui::Text("Attr.");
                        ImGui::SameLine();
                        if (ImGui::Combo("##weight_attr", &selected, items.c_str())) {
                            vertex_weight_attribute_name_ = selected == 0
                                ? std::string()
                                : attribute_names[static_cast<std::size_t>(selected)];
                        }
                    }
                }
            }

            ImGui::Separator();

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

        void perform(const GEO::Mesh& mesh) {
            LOG::TRACE("{}({})", application_name_, __FUNCTION__);

            const GEO::index_t DIM = mesh.vertices.dimension() > 2 ? 3 : 2;

            // Bind the weight attribute the user selected. Every call rebinds,
            // since the attribute lives in the store of the selected mesh: the
            // previous binding may point to another mesh (another object was
            // selected) or to a store the reload of this one replaced.
            // bind_if_is_compatible() leaves the attribute unbound when the
            // name no longer matches a per-vertex double attribute, in which
            // case the sampling is area-proportional, as GeoGram does.
            if (mesh_v_weight_.is_bound())
                mesh_v_weight_.unbind();
            if (!vertex_weight_attribute_name_.empty())
                mesh_v_weight_.bind_if_is_compatible(
                    mesh.vertices.attributes(), vertex_weight_attribute_name_);

            /* Sampling */
            std::vector<double> sample_points;
            sample_points.resize(samples_nb*DIM);
            if (DIM == 2)
                GEO::mesh_generate_random_samples_on_surface<2>(
                    mesh,
                    sample_points.data(),
                    samples_nb,
                    mesh_v_weight_);
            else
                GEO::mesh_generate_random_samples_on_surface<3>(
                    mesh,
                    sample_points.data(),
                    samples_nb,
                    mesh_v_weight_);

            /* Build samples object */
            GEO::Mesh samples_mesh(DIM);
            samples_mesh.vertices.assign_points(sample_points.data(), DIM, samples_nb);
            objects_.push_back(std::make_shared<MeshObject>("samples", "", samples_mesh));

            LOG::INFO("Done {}({})!", application_name_, __FUNCTION__);
        }

        int samples_nb = 1e3;

        bool vertex_weighted_ = false;
        GEO::Attribute<double> mesh_v_weight_;
        std::string vertex_weight_attribute_name_;
    };
}

#endif //GEOLIO_MESH_RANDOM_SAMPLING_APPLICATION_H
