//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/31.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_STREAMLINE_TRACING_APPLICATION_H
#define GEOLIO_STREAMLINE_TRACING_APPLICATION_H
#include <geolio/geobox/application/base_application.h>
#include <geolio/geobox/object/directional_object.h>
#include <geolio/common/log.h>
#include <geolio/directional/streamline_tracing.h>

namespace geolio::geobox
{
    class StreamlineTracingApplication : public BaseApplication {
    public:
        explicit StreamlineTracingApplication(ApplicationObjects& objects)
            : BaseApplication("streamline tracing", objects)
        {
            menu_tooltip_ = "Tracing directional field.";
        }

    protected:
        void draw_window_contents() override {
            // The directional picker, offering the meshes this application accepts.
            draw_directional_selector(
                selected_directional_object_,
                [](const DirectionalObject& directional) {
                    return directional.attached();
                });

            draw_mesh_selector(
                selected_samples_object_,
                [](const GEO::Mesh& mesh) {
                    return mesh.vertices.nb() > 0;
                },
                "Samples");

            ImGui::Separator();

            ImGui::BeginDisabled(selected_directional_object_.expired() || selected_samples_object_.expired());
            const auto directional_object = selected_directional_object_.lock();
            const auto samples_object = selected_samples_object_.lock();
            if (directional_object != nullptr &&
                samples_object != nullptr
                ) {
                const auto& samples = samples_object->mesh();

                if (ImGui::Button("Perform", ImVec2(-1.0f, 0.0f)))
                    perform(base_surface, samples);
            }
        }

        void perform(GEO::Mesh& base_surface, const GEO::Mesh& samples) const {
            LOG::TRACE("{}({})", application_name_, __FUNCTION__);

            std::unique_ptr<StreamlineTracer> tracer;
            if (volumetric_)
                tracer = std::make_unique<StreamlineVolumetricTracer>(base_surface);
            else
                tracer = std::make_unique<StreamlineSurficialTracer>(base_surface);
            if () // TODO: if directional object attach to facets
                tracer->set_per_facet_directions(...);
            else if () // TODO: if directional object attach to cells
                tracer->set_per_cell_directions(...);
            else // TODO: if directional object attach to vertices
                tracer->set_per_vertex_directions(...);
            tracer->set_initial_points(samples.vertices.nb(), samples.vertices.point_ptr(0));
            tracer->trace(iterations_nb_);

            /* Build tracing lines object */
            GEO::Mesh mesh_out;
            GEO::Attribute<GEO::index_t> mesh_out_v_tracing_times(mesh_out.vertices.attributes(), "tracing_times");
            GEO::Attribute<GEO::index_t> mesh_out_e_point_idx(mesh_out.edges.attributes(), "point_idx");
            const auto tracing_lines_object = std::make_shared<MeshObject>("tracing lines", "", mesh_out);
            tracing_lines_object->display_attribute("vertices.point_idx");
            objects_.push_back(tracing_lines_object);

            LOG::INFO("Done {}({})!", application_name_, __FUNCTION__);
        }

        std::weak_ptr<DirectionalObject> selected_directional_object_;
        std::weak_ptr<MeshObject> selected_samples_object_;

        bool volumetric_ = false;
        int iterations_nb_ = 100;
        // int smooth_iterations_nb_ = 3;
        // float smooth_damping_ = 0.5;
    };
}

#endif //GEOLIO_STREAMLINE_TRACING_APPLICATION_H
