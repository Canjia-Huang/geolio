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
            menu_tooltip_ = "GEO::CentroidalVoronoiTesselation";
        }

    protected:
        void draw_window_contents() override {
            // One picker per mesh this application works on: each keeps its own
            // selection (and its own label, which also names the ImGui item).
            draw_mesh_selector(
                base_surface_object_,
                [](const GEO::Mesh& mesh) {
                    return (mesh.facets.nb() > 0 && mesh.facets.are_simplices()) ||
                           (mesh.cells.nb() > 0 && mesh.cells.are_simplices());
                },
                "Base surface");

            draw_mesh_selector(
                samples_object_,
                [](const GEO::Mesh& mesh) {
                    return mesh.vertices.nb() > 0;
                },
                "Samples");

            ImGui::Separator();

            // When no mesh object is selected, everything below the picker
            // (the parameters and the button that runs the operation) is
            // non-interactive.
            ImGui::BeginDisabled(base_surface_object_.expired() || samples_object_.expired());
            const auto base_surface_object = base_surface_object_.lock();
            const auto samples_object = samples_object_.lock();
            if (base_surface_object != nullptr &&
                samples_object != nullptr
                ) {
                auto& base_surface = base_surface_object->mesh();
                const auto& samples = samples_object->mesh();

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

            delaunay->set_keeps_infinite(false);
            delaunay->set_vertices(samples.vertices.nb(), samples.vertices.point_ptr(0));
            delaunay->compute();

            GEO::Mesh mesh_out;
            RVD->compute_RVD(mesh_out, 0);

            const auto RVD_object = std::make_shared<MeshObject>("RVD", "", mesh_out);
            if (!RVD_object->display_attribute(kRegionAttribute))
                LOG::WARN("The result RVD mesh has no `{}` attribute to display (something may wrong?).", kRegionAttribute);
            objects_.push_back(RVD_object);

            LOG::INFO("Done {}({})!", application_name_, __FUNCTION__);
        }

        static constexpr auto kRegionAttribute = "facets.region";

        std::weak_ptr<MeshObject> base_surface_object_;
        std::weak_ptr<MeshObject> samples_object_;
    };
}

#endif //GEOLIO_RESTRICTED_VORONOI_DIAGRAM_APPLICATION_H
