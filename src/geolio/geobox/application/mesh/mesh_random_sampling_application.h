//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_MESH_RANDOM_SAMPLING_APPLICATION_H
#define GEOLIO_MESH_RANDOM_SAMPLING_APPLICATION_H
#include <geolio/geobox/application/base_application.h>
#include <geolio/geobox/object/mesh_object.h>
#include <geolio/common/log.h>
#include <geogram/mesh/mesh_sampling.h>

namespace geolio::geobox
{
    class MeshRandomSamplingApplication : public BaseApplication {
    public:
        explicit MeshRandomSamplingApplication(ApplicationObjects& objects)
            : BaseApplication("random sampling", objects)
        {
            menu_tooltip_ = "Random sampling on mesh.";
        }

    protected:
        void draw_window_contents() override {
            // The mesh picker, offering the meshes this application accepts.
            const bool selection_changed = draw_mesh_selector(
                selected_mesh_object_,
                [](const GEO::Mesh& mesh) {
                    return (mesh.facets.nb() > 0 && mesh.facets.are_simplices()) ||
                           (mesh.cells.nb() > 0 && mesh.cells.are_simplices());
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

                const bool has_facets = mesh.facets.nb() > 0;
                if (!has_facets)
                    volumetric_ = true;
                const bool has_cells = mesh.cells.nb() > 0;
                if (!has_cells)
                    volumetric_ = false;
                ImGui::BeginDisabled(!has_cells || !has_facets);
                {
                    ImGui::Checkbox("Volumetric", &volumetric_);
                }
                ImGui::EndDisabled();

                ImGui::Separator();

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
                    // mesh_v_weight_, which is what is_weight_attribute() keeps.
                    ImGui::Checkbox("Weighted by vertex attribute", &vertex_weighted_);
                    if (vertex_weighted_)
                        draw_attribute_selector(
                            weight_attribute_,
                            *mesh_object,
                            [](const MeshObject::AttributeInfo& attribute) {
                                return attribute.subelements == GEO::MESH_VERTICES &&
                                       attribute.holds<double>() &&
                                       attribute.dimension == 1;}
                                       );
                }

                ImGui::Separator();

                if (ImGui::Button("Perform", ImVec2(-1.0f, 0.0f)))
                        perform(mesh);
            }
            ImGui::EndDisabled();
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
            GEO::Attribute<double> mesh_v_weight;
            if (vertex_weighted_ && !weight_attribute_.name.empty())
                mesh_v_weight.bind_if_is_compatible(mesh.vertices.attributes(), weight_attribute_.name);

            /* Sampling */
            std::vector<double> sample_points;
            sample_points.resize(samples_nb*DIM);
            if (DIM == 2)
                GEO::mesh_generate_random_samples_on_surface<2>(
                    mesh,
                    sample_points.data(),
                    samples_nb,
                    mesh_v_weight);
            else {
                if (volumetric_) {
                    GEO::mesh_generate_random_samples_in_volume<3>(
                        mesh,
                        sample_points.data(),
                        samples_nb,
                        mesh_v_weight);
                }
                else {
                    GEO::mesh_generate_random_samples_on_surface<3>(
                        mesh,
                        sample_points.data(),
                        samples_nb,
                        mesh_v_weight);
                }
            }


            /* Build samples object */
            GEO::Mesh samples_mesh(DIM);
            samples_mesh.vertices.assign_points(sample_points.data(), DIM, samples_nb);
            objects_.push_back(std::make_shared<MeshObject>("samples", "", samples_mesh));

            LOG::INFO("Done {}({})!", application_name_, __FUNCTION__);
        }

        bool volumetric_ = false;

        int samples_nb = 1e3;

        /** Whether the sampling is weighted by a vertex attribute (see
         *  weight_attribute_ for which one). */
        bool vertex_weighted_ = false;

        /** The mesh object picked with draw_mesh_selector(). */
        std::weak_ptr<MeshObject> selected_mesh_object_;

        /** The vertex attribute weighting the sampling, picked with
         *  draw_attribute_selector(); its default value (GEO::MESH_NONE
         *  sub-elements, empty name) means "not weighted". */
        MeshObject::AttributeInfo weight_attribute_;
    };
}

#endif //GEOLIO_MESH_RANDOM_SAMPLING_APPLICATION_H
