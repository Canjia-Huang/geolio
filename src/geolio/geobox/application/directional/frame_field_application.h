//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/31.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_FRAME_FIELD_APPLICATION_H
#define GEOLIO_FRAME_FIELD_APPLICATION_H
#include <geolio/geobox/application/base_application.h>
#include <geolio/geobox/object/directional_object.h>
#include <geolio/geobox/object/mesh_object.h>
#include <geolio/common/log.h>
#include <geolio/directional/frame_field.h>

namespace geolio::geobox
{
    class FrameFieldApplication : public BaseApplication {
    public:
        explicit FrameFieldApplication(ApplicationObjects& objects)
            : BaseApplication("frame field", objects)
        {
            menu_tooltip_ = "Generate frame field.";
        }

    protected:
        void draw_window_contents() override {
            // The mesh picker, offering the meshes this application accepts.
            const bool selection_changed = draw_mesh_selector(
                selected_mesh_object_,
                [](const GEO::Mesh& mesh) {
                    return (mesh.facets.nb() > 0 && mesh.facets.are_simplices());
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

                {
                    // A volumetric field is extrapolated to the cells of the
                    // mesh, which GeoGram reads as tetrahedra. A surface field is
                    // computed on the facets of the mesh, which are the border of
                    // a tetrahedral mesh that has none (see perform()).
                    const bool has_tets =
                        mesh.cells.nb() > 0 && mesh.cells.are_simplices();
                    if (!has_tets)
                        volumetric_ = false;
                    ImGui::BeginDisabled(!has_tets);
                    {
                        ImGui::Checkbox("Volumetric", &volumetric_);
                    }
                    ImGui::EndDisabled();
                }

                ImGui::Separator();

                {
                    ImGui::Checkbox("Curvature weighted", &curvature_weighted_);
                    if (curvature_weighted_) {
                        ImGui::Indent();
                        ImGui::SetNextItemWidth(slider_width_);
                        ImGui::DragFloat("autoscale", &autoscale_, 0.001, 0, 1e8, "%.3f");
                        ImGui::Unindent();
                    }
                }

                ImGui::Separator();

                {
                    ImGui::SetNextItemWidth(slider_width_);
                    ImGui::SliderFloat("Sharp angle", &sharp_angle_threshold_, 0, 180, "%.1f");
                }

                ImGui::Separator();

                if (ImGui::Button("Perform", ImVec2(-1.0f, 0.0f)))
                    perform(mesh);
            }
            ImGui::EndDisabled();
        }

        /**
         * @brief Computes the frame field of @p mesh and shows it as an object.
         * @details The field is displayed by a DirectionalObject, which draws its
         *          three vectors per element as edges starting at the center of
         *          that element (see DirectionalObject).
         * @param[in] mesh Mesh to compute the field on.
         * @return The object that displays the field, or nullptr when the field
         *         could not be computed.
         */
        void perform(const GEO::Mesh& mesh) const {
            LOG::TRACE("{}({})", application_name_, __FUNCTION__);

            // GeoGram computes the field from the facets of the mesh, and its
            // solver works on their adjacency. The facets of a tetrahedral mesh
            // whose border was never computed are built here, on a copy: the mesh
            // of the object is const, and a copy is only made when there is
            // something to complete.
            GEO::Mesh completed;
            const GEO::Mesh* surface = &mesh;
            if (mesh.facets.nb() == 0 && mesh.cells.nb() != 0) {
                completed.copy(mesh);
                completed.cells.connect();
                completed.cells.compute_borders();
                completed.facets.connect();
                surface = &completed;
            }

            // Without facets, the linear system of GeoGram's solver is empty and
            // its solver aborts.
            if (surface->facets.nb() == 0) {
                LOG::ERROR(
                    "The frame field is computed from the surface of the mesh, "
                    "but this one has no facet.");
                return;
            }
            if (volumetric_ &&
                (surface->cells.nb() == 0 || !surface->cells.are_simplices())) {
                LOG::ERROR(
                    "A volumetric frame field is extrapolated to the cells of "
                    "the mesh, but they are not all tetrahedra.");
                return;
            }

            FrameField frame_field;
            if (curvature_weighted_)
                frame_field.create_curvature_directions(*surface, volumetric_, sharp_angle_threshold_, autoscale_);
            else
                frame_field.create_from_surface_mesh(*surface, volumetric_, sharp_angle_threshold_);

            /* Build directional object */

            // A frame field holds three orthogonal directions per element.
            constexpr GEO::index_t NB_VECTORS_PER_ELEMENT = 3;
            const std::vector<double> vectors(frame_field.frames().begin(), frame_field.frames().end());

            // One frame per element: anything else would make the object read
            // the vectors of another element, or past the end of the array.
            const GEO::index_t nb_elements =
                volumetric_ ? surface->cells.nb() : surface->facets.nb();
            if (vectors.size() != 3 * NB_VECTORS_PER_ELEMENT * nb_elements) {
                LOG::ERROR(
                    "The computed field holds {} values, expected {} for {} "
                    "elements of {} vectors.",
                    vectors.size(),
                    3 * NB_VECTORS_PER_ELEMENT * nb_elements,
                    nb_elements,
                    NB_VECTORS_PER_ELEMENT);
                return;
            }

            objects_.push_back(std::make_shared<DirectionalObject>(
                "frame field",
                "",
                *surface,
                NB_VECTORS_PER_ELEMENT,
                vectors,
                volumetric_ ? GEO::MESH_CELLS : GEO::MESH_FACETS));

            LOG::INFO("Done {}({})!", application_name_, __FUNCTION__);
        }

        bool volumetric_ = false;
        bool curvature_weighted_ = false;
        float autoscale_ = 0.0;
        float sharp_angle_threshold_ = 45;

        /** The mesh object picked with draw_mesh_selector(). */
        std::weak_ptr<MeshObject> selected_mesh_object_;

        /** The object that displays the field of the last run, so that a new run
         *  replaces it instead of piling the fields up in the scene. */
        std::weak_ptr<DirectionalObject> object_;
    };
}

#endif //GEOLIO_FRAME_FIELD_APPLICATION_H
