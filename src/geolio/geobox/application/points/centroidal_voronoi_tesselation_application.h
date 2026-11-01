//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_CENTROIDAL_VORONOI_TESSELATION_APPLICATION_H
#define GEOLIO_CENTROIDAL_VORONOI_TESSELATION_APPLICATION_H
#include <geogram/voronoi/CVT.h>
#include <geolio/geobox/application/base_application.h>
#include <geolio/common/log.h>

#include "geolio/directional/frame_field.h"
#include "geolio/LpCVT/lp_cvt.h"

namespace geolio::geobox
{
    class CentroidalVoronoiTesselationApplication : public BaseApplication {
    public:
        explicit CentroidalVoronoiTesselationApplication(ApplicationObjects& objects)
            : BaseApplication("centroidal Voronoi tesselation", objects)
        {
            menu_tooltip_ = "GEO::CentroidalVoronoiTesselation";
        }

    protected:
        template <class T, class U>
        static bool same_owner(const std::weak_ptr<T>& a, const std::weak_ptr<U>& b) noexcept {
            return !a.owner_before(b) && !b.owner_before(a);
        }

        void draw_window_contents() override {
            // One picker per mesh this application works on: each keeps its own
            // selection (and its own label, which also names the ImGui item).
            draw_mesh_selector(
                selected_base_surface_object_,
                [](const GEO::Mesh& mesh) {
                    return (mesh.facets.nb() > 0 && mesh.facets.are_simplices()) ||
                           (mesh.cells.nb() > 0 && mesh.cells.are_simplices());
                },
                "Base surface");

            draw_mesh_selector(
                selected_samples_object_,
                [](const GEO::Mesh& mesh) {
                    return mesh.vertices.nb() > 0;
                },
                "Samples");

            ImGui::Separator();

            // When no mesh object is selected, everything below the picker
            // (the parameters and the button that runs the operation) is
            // non-interactive.
            ImGui::BeginDisabled(selected_base_surface_object_.expired() || selected_samples_object_.expired());
            const auto base_surface_object = selected_base_surface_object_.lock();
            const auto samples_object = selected_samples_object_.lock();
            if (base_surface_object != nullptr &&
                samples_object != nullptr
                ) {
                if (same_owner(selected_base_surface_object_, selected_samples_object_)) {
                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.4f, 0.2f, 1.0f));
                    ImGui::TextWrapped("Warning: The configured base surface and samples are the same object; unexpected behavior may occur!");
                    ImGui::PopStyleColor();

                    ImGui::Separator();
                }

                auto& base_surface = base_surface_object->mesh();
                auto& samples = samples_object->mesh();

                {
                    ImGui::Checkbox("LpCVT", &use_LpCVT_);
                    if (use_LpCVT_) {
                        ImGui::Indent();

                        ImGui::SetNextItemWidth(slider_width_);
                        if (ImGui::SliderInt("p", &p_, 4, 16, "%d", ImGuiSliderFlags_AlwaysClamp)) {
                            if (p_ % 2 != 0)
                                --p_;
                            assert(LpCentroidalVoronoiTesselation::is_supported_norm_exponent(p_));
                        }

                        ImGui::Checkbox("Frame field aligned", &frame_field_aligned_);
                        if (frame_field_aligned_) {
                            draw_directional_selector(
                                selected_directional_object_,
                                [this, &base_surface](const DirectionalObject& directional) {
                                    return directional.attached() &&
                                           directional.vectors_nb_per_element() == 3 &&
                                           (volumetric_ ?
                                           (directional.subelements() == GEO::MESH_CELLS &&
                                            directional.nb_elements() == base_surface.cells.nb()) :
                                           (directional.subelements() == GEO::MESH_FACETS &&
                                            directional.nb_elements() == base_surface.facets.nb())
                                            );
                                });

                            ImGui::DragFloat("max aspect", &max_aspect_, 0.001, 1, 1e8, "%.3f");
                        }

                        ImGui::Unindent();
                    }
                }

                ImGui::Separator();

                {
                    const bool has_facets = base_surface.facets.nb() > 0;
                    if (!has_facets)
                        volumetric_ = true;
                    const bool has_cells = base_surface.cells.nb() > 0;
                    if (!has_cells)
                        volumetric_ = false;
                    ImGui::BeginDisabled(!has_cells || !has_facets);
                    {
                        ImGui::Checkbox("Volumetric", &volumetric_);
                    }
                    ImGui::EndDisabled();
                }

                ImGui::Separator();

                ImGui::BeginDisabled(use_LpCVT_);
                {
                    if (use_LpCVT_)
                        Lloyd_iterations_ = false;

                    ImGui::Checkbox("Lloyd iter.", &Lloyd_iterations_);
                    ImGui::SameLine();
                    bool Newton = !Lloyd_iterations_;
                    if (ImGui::Checkbox("Newton iter.", &Newton))
                        Lloyd_iterations_ = !Newton;
                }
                ImGui::EndDisabled();

                {
                    ImGui::SetNextItemWidth(slider_width_);

                    ImGui::DragInt(
                        "Iterations nb",
                        &iterations_nb_, 1, 0, 1e8);
                }

                ImGui::Separator();

                {
                    ImGui::Checkbox("Vertex weighted", &vertex_weighted_);
                    if (vertex_weighted_)
                        draw_attribute_selector(
                            weight_attribute_,
                            *base_surface_object,
                            [](const MeshObject::AttributeInfo& attribute) {
                                return attribute.subelements == GEO::MESH_VERTICES &&
                                       attribute.holds<double>() &&
                                       attribute.dimension == 1;}
                                       );
                }

                ImGui::Separator();

                if (ImGui::Button("Perform", ImVec2(-1.0f, 0.0f)))
                    perform(base_surface, samples);
            }
            ImGui::EndDisabled();
        }

        void perform(
            const GEO::Mesh& base_surface_in,
            GEO::Mesh& samples
            ) const {
            LOG::TRACE("{}({})", application_name_, __FUNCTION__);

            GEO::Mesh base_surface;
            base_surface.copy(base_surface_in, false);

            if (vertex_weighted_ && !weight_attribute_.name.empty()) {
                GEO::Attribute<double> base_surface_in_v_weight;
                if (!weight_attribute_.name.empty())
                    base_surface_in_v_weight.bind_if_is_compatible(base_surface_in.vertices.attributes(), weight_attribute_.name);

                GEO::Attribute<double> base_surface_v_weight(base_surface.vertices.attributes(), "weight");
                base_surface_v_weight.copy(base_surface_in_v_weight);
            }

            const GEO::index_t DIM = base_surface.vertices.dimension();

            std::shared_ptr<GEO::CentroidalVoronoiTesselation> CVT_ptr;
            if (use_LpCVT_) {
                const auto LpCVT_ptr = std::make_shared<LpCentroidalVoronoiTesselation>(&base_surface, p_, volumetric_);
                if (frame_field_aligned_) {
                    const auto directional_object = selected_directional_object_.lock();

                    // The matrices are laid out from the mesh, so the field has to hold
                    // exactly one frame per element of the mode being meshed. The picker
                    // above chooses the field against the facets, so the check is
                    // repeated here rather than assumed: a volumetric run reads the same
                    // frames as if they were one per cell.
                    const GEO::index_t nb_elements = volumetric_
                        ? base_surface.cells.nb()
                        : base_surface.facets.nb();
                    if (directional_object != nullptr &&
                        directional_object->vectors_nb_per_element() == 3 &&
                        directional_object->nb_frames() == nb_elements) {
                        // The field comes from a .fra file rather than from a
                        // FrameField, so it carries no curvature certainty: the
                        // anisotropy is then read from the lengths of the frame
                        // vectors, which is what the file stores.
                        const FrameField frame_field;
                        std::vector<double> matrices;
                        frame_field.compute_anisotropy_matrices(
                            base_surface,
                            volumetric_,
                            directional_object->frames(),
                            max_aspect_,
                            matrices);
                        LpCVT_ptr->set_matrices(matrices);
                    }
                    else {
                        LOG::ERROR(
                            "The frame field does not carry one frame per {} of the "
                            "base surface ({} frames, {} needed): the anisotropic "
                            "metric is left out and the LpCVT runs isotropic.",
                            volumetric_ ? "cell" : "facet",
                            directional_object != nullptr
                                ? directional_object->nb_frames()
                                : GEO::index_t(0),
                            nb_elements);
                    }
                }
                CVT_ptr = LpCVT_ptr;
            }
            else {
                CVT_ptr = std::make_shared<GEO::CentroidalVoronoiTesselation>(&base_surface);
                CVT_ptr->set_volumetric(volumetric_);
            }

            CVT_ptr->set_points(samples.vertices.nb(), samples.vertices.point_ptr(0));
            if (Lloyd_iterations_)
                CVT_ptr->Lloyd_iterations(iterations_nb_);
            else
                CVT_ptr->Newton_iterations(iterations_nb_);

            for (const auto& v : samples.vertices)
                std::ranges::copy_n(CVT_ptr->embedding(v), DIM, samples.vertices.point_ptr(v));

            LOG::INFO("Done {}({})!", application_name_, __FUNCTION__);
        }

        std::weak_ptr<MeshObject> selected_base_surface_object_;
        std::weak_ptr<MeshObject> selected_samples_object_;

        MeshObject::AttributeInfo weight_attribute_;

        bool use_LpCVT_ = false;
        int p_ = 4;
        bool frame_field_aligned_ = false;
        std::weak_ptr<DirectionalObject> selected_directional_object_;
        float max_aspect_ = 3;

        bool volumetric_ = false;
        bool Lloyd_iterations_ = true; // false -> Newton iterations
        int iterations_nb_ = 30;
        bool vertex_weighted_ = false;
    };
}

#endif //GEOLIO_CENTROIDAL_VORONOI_TESSELATION_APPLICATION_H
