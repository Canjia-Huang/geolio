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

        /**
         * @brief Returns the length of the mesh bounding-box diagonal.
         * @details The result is cached in bbox_diag_ and recomputed when the
         *          mesh changes (see reload()); it is used to give the vertex
         *          marker size slider a sensible range and default for models
         *          of any scale.
         */
        float bbox_diagonal() const;

        auto& mesh() { return mesh_; }

    protected:
        /**
         * @brief Draws the mesh vertices as markers with an absolute
         *        (model-space) size.
         * @details Each vertex is drawn as a camera-facing disc of diameter
         *          vertices_size_ (in mesh coordinates), so the on-screen
         *          size follows the camera distance (near large, far small)
         *          instead of staying constant in pixels.
         */
        void draw_points();

        /**
         * @brief Draws the mesh surface facets.
         * @ref <geogram_gfx/gui/simple_mesh_application.cpp> draw_surface()
         */
        void draw_surface();

        /**
         * @brief Draws the explicit 1-D edges of the mesh (mesh.edges).
         * @details Unlike the facet/cell wireframes, which are derived on the
         *          fly from mesh.facets/mesh.cells while drawing, this pass
         *          renders the edges actually stored in mesh.edges as
         *          standalone lines, with their own visibility/color/width
         *          (show_edges_, edges_color_, edges_width_).
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
         * @brief Tests whether the volume must be drawn facet by facet.
         * @details True when a scalar attribute is displayed on the cell facets
         *          (GEO::MESH_CELL_FACETS). Such an attribute is defined per
         *          half-facet, i.e. once per (cell, facet) incidence, so a cell
         *          can no longer be sent to GLUP as a whole-cell primitive
         *          (GLUP_TETRAHEDRA, GLUP_HEXAHEDRA, ...): its facets have to be
         *          emitted one by one, each with the attribute of the cell facet
         *          it comes from.
         * @return true if the attribute display must use the cell-facet path.
         * @ref <geogram_gfx/mesh/mesh_gfx.cpp> MeshGfx::draw_tets() and
         *      MeshGfx::draw_hybrid(), GeoGram PR #386
         */
        bool cell_facets_attribute_active() const;

        /**
         * @brief Returns the shrink to apply to the cell facets.
         * @details GLUP only shrinks whole-cell primitives, so the cell-facet
         *          passes shrink their facets themselves and have to reproduce
         *          the two cases in which GLUP does not shrink at all: no shrink
         *          requested, and cells cut by a slice (see
         *          GLUP::Context::shrink_cells_in_immediate_buffers()).
         * @return The shrink coefficient, between 0.0 (no shrink) and 1.0.
         */
        double cells_shrink_factor() const;

        /**
         * @brief Draws the volume cells as individual facets, each coloured by
         *        the scalar attribute of its cell facet.
         * @details This is the GeoBox implementation of the GeoGram PR #386
         *          drawing paths MeshGfx::draw_tets_immediate_attrib() and
         *          MeshGfx::draw_hybrid_immediate_attrib(), which cannot be
         *          reused here: the helpers they are built on
         *          (MeshGfx::begin_attributes(), MeshGfx::draw_vertex(),
         *          MeshGfx::draw_sequences_if(), MeshGfx::draw_vertex_with_
         *          attribute()...) are protected members of MeshGfx, and
         *          MeshObject owns a MeshGfx instead of deriving from it, so
         *          this class reimplements the same drawing (see begin_
         *          attributes(), draw_volume_cell_facets(), draw_volume_facet_
         *          vertex()).
         * @details GLUP shrinks whole-cell primitives only, so its cells shrink
         *          is disabled for this pass and replicated per facet vertex.
         */
        void draw_volume_cell_facets_attribute();

        /**
         * @brief Draws the facets of the cells in [begin_c, end_c[ that have
         *        \p prim as their GLUP primitive.
         * @details The cells of a volumetric mesh are of mixed types, so their
         *          facets are of mixed size: this pass emits the triangular
         *          facets when \p prim is GLUP_TRIANGLES and the quadrilateral
         *          ones when it is GLUP_QUADS, which is how MeshGfx does it too
         *          (GLUP requires one primitive kind per glupBegin/glupEnd pair).
         * @param[in] begin_c Index of the first cell of the range.
         * @param[in] end_c One position past the last cell of the range.
         * @param[in] prim GLUP_TRIANGLES or GLUP_QUADS.
         * @ref <geogram_gfx/mesh/mesh_gfx.h> MeshGfx::draw_volume_cell_facets(),
         *      GeoGram PR #386
         */
        void draw_volume_cell_facets(
            GEO::index_t begin_c, GEO::index_t end_c, GLUPprimitive prim);

        /**
         * @brief Draws one vertex of one facet of one cell, with the attribute
         *        of that cell facet.
         * @details The attribute is fetched per cell facet (and not per cell),
         *          so that the two facets shared by two adjacent cells can be
         *          displayed with different colors.
         * @param[in] cell Index of the cell the facet belongs to.
         * @param[in] lf Local index of the facet in the cell.
         * @param[in] lv Local index of the vertex in the facet.
         * @param[in] cell_facet Index of the cell facet in mesh.cell_facets,
         *            used as the attribute index.
         * @param[in] centroid Center of \p cell, used to shrink the facet when
         *            cells_shrink_ is set.
         * @ref <geogram_gfx/mesh/mesh_gfx.h>
         *      MeshGfx::draw_volume_facet_vertex(), GeoGram PR #386
         */
        void draw_volume_facet_vertex(
            GEO::index_t cell, GEO::index_t lf, GEO::index_t lv,
            GEO::index_t cell_facet, const double* centroid);

        /**
         * @brief Emits a mesh vertex as the current GLUP vertex.
         * @details Same contract as MeshGfx::draw_vertex(): the vertex is
         *          interpolated between the two positions stored in a 6-D
         *          vertex when animation is active, and 2-D meshes are handled.
         * @param[in] v Index of the vertex to emit.
         * @ref <geogram_gfx/mesh/mesh_gfx.h> MeshGfx::draw_vertex()
         */
        void draw_vertex(GEO::index_t v);

        /**
         * @brief Reads the position of a mesh vertex as three coordinates.
         * @details Missing coordinates of dimension-2 meshes are set to 0, as
         *          in draw_points().
         * @param[in] v Index of the vertex to read.
         * @param[out] p The three coordinates of the vertex.
         */
        void get_vertex_position(GEO::index_t v, double* p) const;

        /**
         * @brief Sets the attribute value of \p element as the texture
         *        coordinate of the next GLUP vertex.
         * @details Same contract as MeshGfx::draw_attribute_as_tex_coord(),
         *          restricted to the scalar (1-D) attributes displayed by
         *          draw_scene().
         * @param[in] element Index of the element the attribute is read from.
         * @ref <geogram_gfx/mesh/mesh_gfx.h>
         *      MeshGfx::draw_attribute_as_tex_coord()
         */
        void draw_attribute_as_tex_coord(GEO::index_t element);

        /**
         * @brief Binds the displayed scalar attribute and sets up the GLUP
         *        texturing used to color the elements with it.
         * @details Same contract as MeshGfx::begin_attributes(), which is a
         *          protected member of MeshGfx and can therefore not be called
         *          from this class; it must be paired with end_attributes().
         * @ref <geogram_gfx/mesh/mesh_gfx.cpp> MeshGfx::begin_attributes()
         */
        void begin_attributes();

        /**
         * @brief Unbinds the scalar attribute and restores the GLUP state
         *        changed by begin_attributes().
         * @ref <geogram_gfx/mesh/mesh_gfx.cpp> MeshGfx::end_attributes()
         */
        void end_attributes();

        /**
         * @brief Autoscales the attribute range for display.
         * @ref <geogram_gfx/gui/simple_mesh_application.cpp> autorange()
         */
        void autorange();

        /**
         * @brief Range of attribute values the random colormap is sampled with.
         * @details The random colormap (RANDOM_COLORMAP_NAME) is meant to give
         *          an attribute with few distinct values -- an index attribute
         *          such as "vertices.v_cell", say -- one color per value. The
         *          range autorange() computes and the min/max fields show
         *          cannot do that when such an attribute holds GEO::NO_INDEX
         *          values: the uint32(-1) sentinel means "no such element"
         *          rather than a value on the attribute scale, but it would
         *          still stretch the range to 4.29e9 and squeeze every real
         *          value, say 0..10, into the first texel of the colormap.
         *          Elements are therefore colored with this range instead when
         *          the random colormap is selected, which gives each integer of
         *          the real range a band of its own and keeps one band free for
         *          GEO::NO_INDEX. The other colormaps keep the plain GeoGram
         *          range, i.e. behave as if this class did not exist.
         */
        struct ColormapRange {
            /** Bounds mapped onto the colormap, [0,1] after the mapping. */
            double min = 0.0;
            double max = 0.0;

            /**
             * Inputs of the computation above: update_colormap_range() returns
             * immediately while they are unchanged, since recomputing them
             * means scanning the whole attribute.
             */
            GEO::index_t colormap_index = 0;
            const void* store = nullptr;
            GEO::index_t element_index = 0;
            GEO::index_t nb_elements = 0;
            float attribute_min = 0.0f;
            float attribute_max = 0.0f;
        };
        mutable ColormapRange colormap_range_;

        /**
         * @brief Tests whether the selected colormap is the random one.
         * @return true if the attribute range must be laid out for one color
         *         per distinct value, see ColormapRange.
         */
        bool random_colormap_active() const;

        /**
         * @brief Lower bound of the attribute range mapped onto the colormap.
         * @return Bound to pass to GEO::glupMapTexCoords1d() (or to
         *         MeshGfx::set_scalar_attribute()) instead of attribute_min_.
         */
        double colormap_range_min() const;

        /**
         * @brief Upper bound of the attribute range mapped onto the colormap.
         * @return Bound to pass to GEO::glupMapTexCoords1d() (or to
         *         MeshGfx::set_scalar_attribute()) instead of attribute_max_.
         */
        double colormap_range_max() const;

        /**
         * @brief Updates colormap_range_ for the current attribute and range.
         * @details Called by colormap_range_min()/max(), so that the colormap
         *          range always matches the attribute the min/max fields
         *          describe, whatever changed them (autorange(), the min/max
         *          fields, another attribute, another colormap, another mesh).
         *          The attribute is only scanned when the random colormap is
         *          selected; autorange() itself keeps the GeoGram behavior of
         *          ranging over every value of the attribute, GEO::NO_INDEX
         *          included.
         */
        void update_colormap_range() const;

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

        /** Cached bounding-box diagonal; < 0 means "not computed yet". */
        mutable float bbox_diag_ = -1.0f;

        bool show_vertices_ = false;
        /**
         * Absolute (model-space) diameter of the vertex markers, expressed
         * in the same units as the mesh coordinates. Because the size is
         * fixed in world space, the on-screen size of a marker grows when
         * the camera gets closer and shrinks when it moves away (near
         * large, far small). The constructor seeds it to a small fraction
         * of the bounding-box diagonal.
         */
        float vertices_size_ = 1.0f;
        GEO::vec4f vertices_color_ = GEO::vec4f(0.0f, 1.0f, 0.0f, 1.0f);
        float vertices_transparency_ = 0.0f;

        // Explicit 1-D edges stored in mesh.edges, drawn as standalone lines
        // (independent from the facet/cell wireframes).
        bool show_edges_ = true;
        float edges_width_ = 0.3f;
        GEO::vec4f edges_color_ = GEO::vec4f(0.00f, 0.55f, 0.05f, 1.0f);

        bool show_surface_ = true;
        bool show_surface_sides_ = false;
        GEO::vec4f surface_color_ = GEO::vec4f(0.5f, 0.5f, 1.0f, 1.0f);
        GEO::vec4f surface_color_2_ = GEO::vec4f(1.0f, 0.5f, 0.0f, 1.0f);
        float surface_transparency_ = 0.0f;

        // Wireframe of the surface facets ("mesh" lines: the edges of
        // mesh.facets, drawn over the surface).
        bool show_surface_mesh_ = true;
        float surface_mesh_width_ = 0.1f;
        GEO::vec4f surface_mesh_color_ = GEO::vec4f(0.05f, 0.05f, 0.05f, 1.0f);

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

        // Wireframe of the volume cells (the edges of mesh.cells, drawn by
        // GLUP while rendering the volume).
        bool show_volume_mesh_ = true;
        float volume_mesh_width_ = 0.1f;
        GEO::vec4f volume_mesh_color_ = GEO::vec4f(0.05f, 0.05f, 0.05f, 1.0f);

        bool show_attributes_ = false;
        GEO::index_t current_colormap_index_ = 0;
        std::string attribute_ = "vertices.point[0]";
        std::string attribute_name_ = "point[0]";
        GEO::MeshElementsFlags attribute_subelements_ = GEO::MESH_VERTICES;
        float attribute_min_ = 0;
        float attribute_max_ = 0;

        /**
         * Accessor to the displayed scalar attribute; bound by
         * begin_attributes() and unbound by end_attributes(), as
         * MeshGfx::scalar_attribute_ is (see MeshGfx::begin_attributes()).
         */
        GEO::ReadOnlyScalarAttributeAdapter scalar_attribute_;
    };
}

#endif //GEOLIO_MESH_OBJECT_H
