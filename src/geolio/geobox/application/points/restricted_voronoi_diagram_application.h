//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/31.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_RESTRICTED_VORONOI_DIAGRAM_APPLICATION_H
#define GEOLIO_RESTRICTED_VORONOI_DIAGRAM_APPLICATION_H
#include <geogram/delaunay/periodic_delaunay_3d.h>
#include <geogram/voronoi/CVT.h>
#include <geolio/geobox/application/base_application.h>
#include <geolio/common/log.h>

namespace geolio::geobox
{
    class RestrictedVoronoiDiagramApplication : public BaseApplication {
    public:
        explicit RestrictedVoronoiDiagramApplication(ApplicationObjects& objects)
            : BaseApplication("restricted Voronoi diagram", objects)
        {
            menu_tooltip_ = "GEO::RestrictedVoronoiDiagram";
        }

    protected:
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
                {
                    ImGui::Checkbox("Voronoi", &voronoi_or_delaunay_);
                    ImGui::SameLine();
                    bool Delaunay = !voronoi_or_delaunay_;
                    if (ImGui::Checkbox("Delaunay", &Delaunay))
                        voronoi_or_delaunay_ = !Delaunay;
                }

                ImGui::Separator();

                auto& base_surface = base_surface_object->mesh();
                const auto& samples = samples_object->mesh();

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

                    if (volumetric_) {
                        ImGui::Indent();

                        ImGui::Checkbox("Cell border only", &cell_border_only_);
                        ImGui::SetItemTooltip("in volumetric mode, computes only the surfacic borders of the volumetric cells (for visualization purpose)");

                        ImGui::Unindent();
                    }
                }

                ImGui::Separator();

                if (ImGui::Button("Perform", ImVec2(-1.0f, 0.0f))) {
                    perform(base_surface, samples);

                    base_surface_object->set_visible(false);
                }
            }
            ImGui::EndDisabled();
        }

        void perform(
            GEO::Mesh& base_surface,
            const GEO::Mesh& samples
            ) const {
            LOG::TRACE("{}({})", application_name_, __FUNCTION__);

            const GEO::SmartPointer delaunay = new GEO::PeriodicDelaunay3d(false);
            const auto& RVD = GEO::RestrictedVoronoiDiagram::create(delaunay, &base_surface);

            RVD->set_volumetric(volumetric_);

            delaunay->set_keeps_infinite(false);
            delaunay->set_vertices(samples.vertices.nb(), samples.vertices.point_ptr(0));
            delaunay->compute();

            if (voronoi_or_delaunay_) {
                GEO::Mesh mesh_out;
                RVD->compute_RVD(mesh_out, 0, cell_border_only_);

                const auto RVD_object = std::make_shared<MeshObject>("RVD", "", mesh_out);
                if (!RVD_object->display_attribute((volumetric_ && !cell_border_only_) ? kCellRegionAttribute : kFacetRegionAttribute))
                    LOG::WARN("The result RVD mesh has no `{}` attribute to display (something may wrong?).",
                        (volumetric_ && !cell_border_only_) ? kCellRegionAttribute : kFacetRegionAttribute);
                objects_.push_back(RVD_object);
            }
            else {
                GEO::Mesh mesh_out;
                RVD->compute_RDT(mesh_out);

                const auto RDT_object = std::make_shared<MeshObject>("RDT", "", mesh_out);
                objects_.push_back(RDT_object);
            }

            LOG::INFO("Done {}({})!", application_name_, __FUNCTION__);
        }

        static constexpr auto kFacetRegionAttribute = "facets.region";
        static constexpr auto kCellRegionAttribute = "cells.region";

        std::weak_ptr<MeshObject> selected_base_surface_object_;
        std::weak_ptr<MeshObject> selected_samples_object_;

        bool voronoi_or_delaunay_ = true;

        bool volumetric_ = false;
        bool cell_border_only_ = false;
    };
}

#endif //GEOLIO_RESTRICTED_VORONOI_DIAGRAM_APPLICATION_H
