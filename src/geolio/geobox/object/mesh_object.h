//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/19.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_MESH_OBJECT_H
#define GEOLIO_MESH_OBJECT_H
#include "base_object.h"
#include <geogram/mesh/mesh.h>
#include <geogram_gfx/mesh/mesh_gfx.h>
#include "geolio/geobox/colormap.h"

namespace geolio::geobox
{
    /**
     * @brief Mesh-backed object rendered by GeoBox.
     * @details This class wraps a GEO::Mesh and exposes the viewer controls needed to
     *          display vertices, surface facets, edges, and volume cells.
     * @ref <geogram_gfx/gui/simple_mesh_application.cpp> SimpleMeshApplication
     */
    class MeshObject : public BaseObject {
    public:
        /**
         * @brief Constructs a mesh object from a display name and a mesh instance.
         * @param[in] name Name used to identify the object in the UI.
         * @param[in] colormaps Available colormap textures for scalar attribute rendering.
         * @param[in] mesh Source mesh data to render.
         */
        explicit MeshObject(
            const std::string& name,
            const std::vector<ColormapInfo>& colormaps,
            const GEO::Mesh& mesh);

        /**
         * @brief Draws the object-specific property widgets.
         */
        void draw_object_properties() override;

        /**
         * @brief Renders the mesh object in the scene.
         * @param[in] lighting If true, enable lighting for shaded volume or surface rendering.
         */
        void draw_scene(bool lighting) override;

        /**
         * @brief Reloads the mesh object from its source representation.
         */
        void reload();

        /**
         * @brief Returns the bounding box of the mesh in world coordinates.
         * @param[out] xyzmin Minimum coordinates in x, y, z order.
         * @param[out] xyzmax Maximum coordinates in x, y, z order.
         */
        void get_bbox(double* xyzmin, double* xyzmax) const override;

    protected:
        /**
         * @brief Draws the mesh vertices.
         * @ref <geogram_gfx/gui/simple_mesh_application.cpp> draw_points()
         */
        void draw_points();

        /**
         * @brief Draws the mesh surface facets.
         * @ref <geogram_gfx/gui/simple_mesh_application.cpp> draw_surface()
         */
        void draw_surface();

        /**
         * @brief Draws the mesh edges.
         * @ref <geogram_gfx/gui/simple_mesh_application.cpp> draw_edges()
         */
        void draw_edges();

        /**
         * @brief Draws the tetrahedral or volumetric cells.
         * @param[in] lighting If true, use lighting when drawing the volume.
         * @ref <geogram_gfx/gui/simple_mesh_application.cpp> draw_volume()
         */
        void draw_volume(bool lighting);

        /**
         * @brief Autoscales the attribute range for display.
         * @ref <geogram_gfx/gui/simple_mesh_application.cpp> autorange()
         */
        void autorange();

        /**
         * @brief Returns a list of available scalar attribute names.
         * @return Comma-separated attribute names for the mesh.
         * @ref <geogram_gfx/gui/simple_mesh_application.cpp> attribute_names()
         */
        std::string attribute_names() const {
            return mesh_.get_scalar_attributes();
        }

        /**
         * @brief Selects the active attribute used for scalar coloring.
         * @param[in] attribute Name of the mesh attribute to visualize.
         * @ref <geogram_gfx/gui/simple_mesh_application.cpp> set_attribute()
         */
        void set_attribute(const std::string& attribute);

        GEO::Mesh mesh_;
        GEO::MeshGfx mesh_gfx_;

        bool show_vertices_ = false;
        float vertices_size_ = 1.0f;
        GEO::vec4f vertices_color_ = GEO::vec4f(0.0f, 1.0f, 0.0f, 1.0f);
        float vertices_transparency_ = 0.0f;

        bool show_surface_ = true;
        bool show_surface_sides_ = false;
        GEO::vec4f surface_color_ = GEO::vec4f(0.5f, 0.5f, 1.0f, 1.0f);
        GEO::vec4f surface_color_2_ = GEO::vec4f(1.0f, 0.5f, 0.0f, 1.0f);
        float surface_transparency_ = 0.0f;

        bool show_mesh_ = true;
        float mesh_width_ = 0.1f;
        GEO::vec4f mesh_color_ = GEO::vec4f(0.05f, 0.05f, 0.05f, 1.0f);

        bool show_surface_borders_ = true;
        float surface_borders_width_ = 0.3f;
        GEO::vec4f surface_borders_color_ = GEO::vec4f(0.0f, 0.85f, 0.85f, 1.0f);

        bool show_volume_ = true;
        float cells_shrink_ = 0.0f;
        GEO::vec4f volume_color_ = GEO::vec4f(0.9f, 0.9f, 0.9f, 1.0f);
        bool show_colored_cells_ = false;
        bool show_hexes_ = true;
        bool show_connectors_ = true;
        const std::vector<ColormapInfo>& colormaps_;

        bool show_attributes_ = false;
        GEO::index_t current_colormap_index_ = 0;
        std::string attribute_ = "vertices.point[0]";
        std::string attribute_name_ = "point[0]";
        GEO::MeshElementsFlags attribute_subelements_ = GEO::MESH_VERTICES;
        float attribute_min_ = 0;
        float attribute_max_ = 0;
    };
}

#endif //GEOLIO_MESH_OBJECT_H
