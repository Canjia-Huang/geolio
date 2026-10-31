//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/31.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_STREAMLINE_TRACING_APPLICATION_H
#define GEOLIO_STREAMLINE_TRACING_APPLICATION_H
#include <memory>
#include <geolio/geobox/application/base_application.h>
#include <geolio/geobox/object/directional_object.h>
#include <geolio/geobox/object/mesh_object.h>
#include <geolio/common/log.h>
#include <geolio/directional/streamline_tracing.h>

namespace geolio::geobox
{
    /**
     * @brief Traces the streamlines of a directional field.
     * @details The streamlines follow the field a DirectionalObject displays,
     *          starting from the vertices of another mesh object (the seeds).
     *          The field has to be attached to a mesh, since its vectors go to
     *          the elements of that mesh -- the one the streamlines are traced
     *          on, so that the tracer can follow them from element to element
     *          (see StreamlineTracer).
     */
    class StreamlineTracingApplication : public BaseApplication {
    public:
        explicit StreamlineTracingApplication(ApplicationObjects& objects)
            : BaseApplication("streamline tracing", objects)
        {
            menu_tooltip_ = "Trace streamlines of a directional field.";
        }

    protected:
        void draw_window_contents() override {
            // The field to trace: only the attached ones, since the streamlines
            // follow the elements of the mesh the field sits on.
            draw_directional_selector(
                selected_directional_object_,
                [](const DirectionalObject& directional) {
                    return directional.attached();
                });

            // The seeds: the vertices of a mesh object.
            draw_mesh_selector(
                selected_samples_object_,
                [](const GEO::Mesh& mesh) {
                    return mesh.vertices.nb() > 0;
                },
                "Samples");

            ImGui::Separator();

            const auto directional_object = selected_directional_object_.lock();
            const auto samples_object = selected_samples_object_.lock();
            const bool ready =
                directional_object != nullptr && samples_object != nullptr;

            // Without a field and its seeds there is nothing to trace: the
            // parameters and the button are non-interactive.
            ImGui::BeginDisabled(!ready);
            {
                ImGui::SetNextItemWidth(slider_width_);
                ImGui::DragInt("iterations", &iterations_nb_, 1.0f, 0, 1000000);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Number of tracing steps of each seed.");

                if (ImGui::Button("Perform", ImVec2(-1.0f, 0.0f)) && ready)
                    perform(*directional_object, samples_object->mesh());
            }
            ImGui::EndDisabled();

        }

        /**
         * @brief Traces the streamlines of @p directional from the vertices of
         *        @p samples, and shows them as a new mesh object.
         * @details The tracer is given the directions of one element at a time,
         *          so the way they are handed over depends on the elements the
         *          field is attached to (see DirectionalObject::subelements()):
         *          per facet for a surface field, per cell for a volumetric
         *          one. The vectors of the field have exactly the layout the
         *          tracer expects -- the N vectors of an element one after the
         *          other, 3 coordinates each (see
         *          DirectionalObject::vectors()) -- so they are handed over as
         *          they are.
         * @param[in] directional Field to trace; it has to be attached.
         * @param[in] samples Mesh whose vertices are the seed points of the
         *            streamlines.
         */
        void perform(
            const DirectionalObject& directional,
            const GEO::Mesh& samples
            ) const {
            LOG::TRACE("{}({})", application_name_, __FUNCTION__);

            const auto mesh_object = directional.mesh_object().lock();
            if (mesh_object == nullptr ||
                directional.attached_mesh() == nullptr) {
                LOG::ERROR(
                    "`{}` is not attached to a mesh: there is nothing to trace "
                    "on.",
                    directional.name());
                return;
            }
            // The tracer binds its directions to the mesh it traces on, so it
            // needs to modify it: the field only holds a weak pointer to the
            // object that owns that mesh.
            GEO::Mesh& base_surface = mesh_object->mesh();

            const GEO::index_t vectors_nb_per_element =
                directional.vectors_nb_per_element();
            const auto subelements = directional.subelements();

            // The tracer stores the directions on the elements of the mesh it
            // traces on; a field attached to the vertices is not something it
            // can follow yet (see
            // StreamlineTracer::set_per_vertex_directions()).
            std::unique_ptr<StreamlineTracer> tracer;
            if (subelements == GEO::MESH_CELLS) {
                if (!base_surface.cells.are_simplices()) {
                    LOG::ERROR(
                        "The cells of `{}` are not all tetrahedra: its "
                        "streamlines cannot be traced.",
                        mesh_object->name());
                    return;
                }
                auto volumetric_tracer = std::make_unique<StreamlineVolumetricTracer>(base_surface);
                volumetric_tracer->set_per_cell_directions(vectors_nb_per_element, directional.vectors().data());
                tracer = std::move(volumetric_tracer);
            }
            else if (subelements == GEO::MESH_FACETS) {
                if (!base_surface.facets.are_simplices()) {
                    LOG::ERROR(
                        "The facets of `{}` are not all triangles: its "
                        "streamlines cannot be traced.",
                        mesh_object->name());
                    return;
                }
                auto surficial_tracer = std::make_unique<StreamlineSurficialTracer>(base_surface);
                surficial_tracer->set_per_facet_directions(vectors_nb_per_element, directional.vectors().data());
                tracer = std::move(surficial_tracer);
            }
            else {
                LOG::ERROR(
                    "The field is attached to the {} of `{}`: the streamlines "
                    "can only follow a field attached to its facets or to its "
                    "cells.",
                    DirectionalObject::subelements_name(subelements),
                    mesh_object->name());
                return;
            }

            tracer->set_initial_points(
                samples.vertices.nb(), samples.vertices.point_ptr(0));
            tracer->trace(static_cast<GEO::index_t>(iterations_nb_));

            // The traced lines: a mesh whose edges are the segments of the
            // streamlines, with the tracing step of their endpoints and the
            // seed point each line belongs to.
            GEO::Mesh mesh_out;
            GEO::Attribute<GEO::index_t> tracing_times(mesh_out.vertices.attributes(), "tracing_times");
            GEO::Attribute<GEO::index_t> point_idx(mesh_out.edges.attributes(), "point_idx");
            tracer->append_tracing_points_to_mesh(mesh_out, &tracing_times, &point_idx);

            const auto tracing_lines_object = std::make_shared<MeshObject>("tracing lines", "", mesh_out);
            tracing_lines_object->display_attribute("vertices.tracing_times");
            objects_.push_back(tracing_lines_object);

            // The tracer binds its directions to the mesh it traces on, and
            // removes them when it is destroyed, which happens here: the
            // object of that mesh is told that it changed, so that what it
            // caches about the attributes of its mesh is refreshed.
            tracer.reset();
            mesh_object->mesh_changed();

            LOG::INFO("Done {}({})!", application_name_, __FUNCTION__);
        }

        /// The field to trace (see draw_directional_selector()).
        std::weak_ptr<DirectionalObject> selected_directional_object_;

        /// The mesh whose vertices are the seeds (see draw_mesh_selector()).
        std::weak_ptr<MeshObject> selected_samples_object_;

        /// Number of tracing steps of each seed.
        int iterations_nb_ = 100;
    };
}

#endif //GEOLIO_STREAMLINE_TRACING_APPLICATION_H
