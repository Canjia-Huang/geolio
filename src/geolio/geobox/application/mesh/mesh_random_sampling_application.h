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
            // const auto samples_object = std::make_shared<MeshObject>("samples", )

            LOG::INFO("Done {}({})!", application_name_, __FUNCTION__);
        }

        int samples_nb = 1e3;
        GEO::Attribute<double> mesh_v_weight_;
    };
}

#endif //GEOLIO_MESH_RANDOM_SAMPLING_APPLICATION_H
