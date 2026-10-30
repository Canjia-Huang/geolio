//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/19.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_MESH_OBJECT_H
#define GEOLIO_MESH_OBJECT_H
#include <algorithm>
#include <string>
#include <vector>
#include "base_object.h"
#include <geogram/mesh/mesh.h>
#include <geogram_gfx/mesh/mesh_gfx.h>
#include "geolio/geobox/colormap.h"

namespace geolio::geobox
{
    /**
     * @brief How the elements of a mesh are colored.
     * @details The painting modes of Graphite's MeshGrobShader:
     *          - SOLID_COLOR colors the surface with surface_color_;
     *          - ATTRIBUTE maps a scalar attribute through a colormap;
     *          - TEXTURE samples an image through a 2D texture-coordinate
     *            attribute.
     */
    enum class PaintingMode {
        SOLID_COLOR,
        ATTRIBUTE,
        TEXTURE
    };

    /**
     * @brief Releases the GL textures loaded for the TEXTURE painting mode.
     * @details The textures are shared by all the objects (they are cached by
     *          image file name), so they cannot be owned by a MeshObject. The
     *          application calls this while its GL context is still current.
     */
    void release_texture_cache();

    /**
     * @brief Mesh-backed object rendered by GeoBox.
     * @details This class wraps a GEO::Mesh and exposes the viewer controls needed to
     *          display vertices, surface facets, edges, and volume cells.
     * @details The colormaps a scalar attribute can be displayed with are not
     *          part of the object: they are the constant set of the application
     *          (see ColormapLibrary), read here through colormaps().
     * @ref <geogram_gfx/gui/simple_mesh_application.cpp> SimpleMeshApplication
     */
    class MeshObject : public BaseObject {
    public:
        /**
         * @brief Constructs a mesh object from a display name and a mesh instance.
         * @param[in] name Object name.
         * @param[in] filepath Path to the source asset.
         * @param[in] mesh Source mesh data to render.
         */
        explicit MeshObject(
            std::string name,
            std::string filepath,
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
         * @brief Facet and cell counts broken down by kind, for the Info
         *        panel.
         * @details Facets are classified by arity (triangle, quad, polygon)
         *          and cells by their GeoGram type (tet, hex, prism, pyramid,
         *          connector).
         */
        struct ElementCounts {
            GEO::index_t triangles = 0;
            GEO::index_t quads = 0;
            GEO::index_t polygons = 0;
            /** Indexed by GEO::MeshCellType. */
            GEO::index_t cells[GEO::MESH_NB_CELL_TYPES] = {0, 0, 0, 0, 0};
        };

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
         * @brief Tests whether the surface has border edges to draw.
         * @details Same criterion as MeshGfx::draw_surface_borders(), which
         *          draws the facet edges without an adjacent facet: an edge is
         *          on the border when the facet corner it comes from has
         *          GEO::NO_FACET as its adjacent facet. A closed surface has no
         *          such edge, so its border controls are hidden (see
         *          draw_object_properties()).
         * @return true if at least one facet edge is on the border.
         */
        bool compute_surface_has_borders() const;

        /**
         * @brief Counts the facets and the cells of each kind.
         * @details The result is cached in element_counts_ when the mesh is
         *          set (constructor) or reloaded; it feeds the breakdown rows
         *          of the Info panel (see draw_object_properties()).
         * @return The counts, indexed by facet arity and by cell type.
         */
        ElementCounts count_elements() const;

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
         * @brief Returns the image texture sampled in TEXTURE painting mode.
         * @details The image is loaded on first use and cached by file name, so
         *          that several objects that display the same image share the
         *          same GL texture (see release_texture_cache()).
         * @return The GL texture, or 0 when no image is selected or the image
         *         could not be loaded.
         */
        GLuint texture();

        /**
         * @brief Repeat factor applied to the image in TEXTURE mode.
         * @details tex_repeat_ while tex_repeat_enabled_ is set, and 1
         *          otherwise (the image is then sampled once over the texture
         *          coordinate range).
         */
        GEO::index_t texture_repeat() const {
            return tex_repeat_enabled_ ?
                static_cast<GEO::index_t>(std::max(tex_repeat_, 1)) : 1;
        }

        /**
         * @brief Selects the texture-coordinate attribute used in TEXTURE mode.
         * @details The attribute must be a vector attribute of dimension 2, such
         *          as "facet_corners.tex_coord" (as loaded from an OBJ file).
         * @param[in] attribute Full attribute name, e.g. "facet_corners.tex_coord".
         */
        void set_tex_coords(const std::string& attribute);

        /**
         * @brief Lists the attributes that can be used as texture coordinates.
         * @details Only the 2D vector attributes MeshGfx can feed to a 2D
         *          texture are listed, on the element stores it can texture.
         *          As in Graphite, an attribute stored as a vector of scalars
         *          and one stored as an array of vec2 are both supported.
         * @return Semicolon-separated full attribute names; empty when the
         *         mesh cannot be textured, in which case the TEXTURE painting
         *         mode is not offered by the GUI.
         */
        std::string texture_attribute_names() const;

        /**
         * @brief Returns the number of scalar components of an attribute.
         * @param[in] subelements Store the attribute belongs to.
         * @param[in] name Base name of the attribute.
         * @return 2 for a 2D vector attribute, 3 for a 3D one, 0 when the
         *         attribute does not exist.
         */
        GEO::index_t attribute_dimension(
            GEO::MeshElementsFlags subelements,
            const std::string& name) const;

        /**
         * @brief Tests whether the displayed attribute exists in the mesh.
         * @details The selection made in the GUI names an attribute of the mesh
         *          the object was built from, and the mesh can lose it: reload()
         *          replaces the mesh with the one its file now holds, which may
         *          not have that attribute any more. GeoGram reconciles the same
         *          situation in MeshGfx::set_scalar_attribute(), by falling back
         *          to MESH_NONE; GeoBox has to do it itself, since the cell-facet
         *          pass below draws without MeshGfx (see
         *          cell_facets_attribute_active()).
         * @details The attribute tested is the one that would be sampled, i.e.
         *          the display copy while it is up to date and the displayed
         *          attribute otherwise (see colormap_attribute_name()); the test
         *          is the one MeshGfx::begin_attributes() binds with, so it tells
         *          exactly whether there is a value to feed the colormap with.
         * @return true if the displayed attribute can be sampled.
         */
        bool displayed_attribute_is_defined();

        /**
         * @brief Tests whether the volume must be drawn facet by facet.
         * @details True when a scalar attribute is displayed on the cell facets
         *          (GEO::MESH_CELL_FACETS) and the mesh still carries it (see
         *          displayed_attribute_is_defined()). Such an attribute is
         *          defined per half-facet, i.e. once per (cell, facet)
         *          incidence, so a cell can no longer be sent to GLUP as a
         *          whole-cell primitive (GLUP_TETRAHEDRA, GLUP_HEXAHEDRA, ...):
         *          its facets have to be emitted one by one, each with the
         *          attribute of the cell facet it comes from.
         * @return true if the attribute display must use the cell-facet path.
         * @ref <geogram_gfx/mesh/mesh_gfx.cpp> MeshGfx::draw_tets() and
         *      MeshGfx::draw_hybrid(), GeoGram PR #386
         */
        bool cell_facets_attribute_active();

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
         *          draw_scene(). Emits no texture coordinate when the mesh does
         *          not have the displayed attribute, which the colormap then
         *          samples as the plain element color (see
         *          displayed_attribute_is_defined()).
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
         * @details Same as GeoGram's SimpleMeshApplication::autorange(): the
         *          range is the raw [min, max] of the attribute, GEO::NO_INDEX
         *          included, which is what the min/max fields show. Handling
         *          that sentinel is up to the colormap range: the elements
         *          outside the range the fields describe take the color of the
         *          nearest end of the colormap (see
         *          update_display_attribute()), so the fields do not have to
         *          leave the sentinel out to keep it from squeezing the values
         *          of the scale into a single color.
         * @ref <geogram_gfx/gui/simple_mesh_application.cpp> autorange()
         */
        void autorange();

        /**
         * @brief Lower bound of the range the colormap spans.
         * @details The range is the one the min/max fields read, so the
         *          colormap covers exactly the values the user asked for: the
         *          two bounds are colored with the two ends of the colormap
         *          (see colormap_mapping_min() for which bound gets which end),
         *          and every value outside of the range takes the color of the
         *          nearest end (see update_display_attribute()).
         * @details The two bounds are ordered here: a range filled in the other
         *          way round is read in increasing order. They are the bounds
         *          the displayed values are clamped to; the bounds the colormap
         *          is sampled with are colormap_mapping_min() and
         *          colormap_mapping_max(), which swap these while the colormap
         *          is flipped.
         * @return Lower bound of the displayed range.
         */
        double colormap_range_min() const {
            return static_cast<double>(
                std::min(attribute_min_, attribute_max_));
        }

        /**
         * @brief Upper bound of the range the colormap spans, see
         *        colormap_range_min().
         */
        double colormap_range_max() const {
            return static_cast<double>(
                std::max(attribute_min_, attribute_max_));
        }

        /**
         * @brief Bound the attribute values are mapped onto the colormap from.
         * @details Same as colormap_range_min() while the colormap is not
         *          flipped (flip_colormap_). Flipping swaps the two bounds
         *          instead of modifying the colormap: GEO::glupMapTexCoords1d()
         *          maps a value with (value - min) / (max - min), so feeding it
         *          the range back to front makes the minimum sample the last
         *          color of the colormap and the maximum its first one. The
         *          displayed values themselves keep being clamped to the
         *          ordered range (see update_display_attribute()), so flipping
         *          only mirrors the colors.
         * @return Lower bound of the mapping, which may be greater than the
         *         upper one.
         */
        double colormap_mapping_min() const {
            return flip_colormap_ ? colormap_range_max() : colormap_range_min();
        }

        /**
         * @brief Upper bound of the colormap mapping, see
         *        colormap_mapping_min().
         */
        double colormap_mapping_max() const {
            return flip_colormap_ ? colormap_range_min() : colormap_range_max();
        }

        /**
         * @brief Returns the colormaps the attribute display selects from.
         * @details The set of colormaps is a property of the application, not of
         *          the object being displayed: it does not change while the
         *          objects live, and every object samples the same one. It is
         *          therefore not handed to the constructor, and not stored here;
         *          reading it through this accessor is what keeps the objects
         *          valid across a rebuild of the table (see
         *          ColormapLibrary::initialize()).
         * @return The process-wide colormap table, in display order.
         */
        static const std::vector<ColormapInfo>& colormaps() {
            return ColormapLibrary::instance().colormaps();
        }

        /**
         * @brief Returns the colormap texture the attribute is sampled with.
         * @details The smoothed texture while smooth_colormap_ is set, and its
         *          nearest-filtered variant otherwise (see
         *          ColormapInfo::nearest_texture): GL_LINEAR blends the
         *          neighboring colormap entries into a gradient, while
         *          GL_NEAREST gives each entry a flat color band separated from
         *          the next one by a hard boundary.
         * @param[in] index Index of the colormap in colormaps().
         * @return The texture to bind, or 0 when @p index is out of range.
         */
        GLuint colormap_texture(GEO::index_t index) const {
            const ColormapInfo* colormap =
                ColormapLibrary::instance().colormap(index);
            if (colormap == nullptr)
                return 0;
            if (smooth_colormap_ || colormap->nearest_texture == 0)
                return colormap->texture;
            return colormap->nearest_texture;
        }

        /**
         * @brief Returns the texture of the selected colormap, see
         *        colormap_texture(GEO::index_t).
         */
        GLuint colormap_texture() const {
            return colormap_texture(current_colormap_index_);
        }

        /**
         * @brief Tests whether the selected colormap hides values through its
         *        alpha channel.
         * @details The "transparent" colormap is the only one: its lower half
         *          is fully transparent, so the fragments it colors have to be
         *          discarded instead of drawn black (see draw_scene()).
         */
        bool colormap_is_transparent() const {
            const ColormapInfo* colormap =
                ColormapLibrary::instance().colormap(current_colormap_index_);
            return colormap != nullptr &&
                   colormap->name == TRANSPARENT_COLORMAP_NAME;
        }

        /**
         * @brief Name of the display-only attribute that holds the values the
         *        colormap is sampled with.
         * @details MeshGfx colors the mesh with the values it finds in the
         *          mesh, but the colormap must be sampled with the attribute
         *          values clamped to the displayed range. An attribute holding
         *          0..3 plus GEO::NO_INDEX would otherwise be sampled with
         *          texcoords up to 1.4e9 -- far outside [0,1] -- which the
         *          texture cannot turn into the color of its last texel: a
         *          minified lookup resolves to an averaged mipmap level, and
         *          the interpolation across the facets mixing such values with
         *          ordinary ones saturates. The clamped values are therefore
         *          materialized in this attribute, built on the store of the
         *          displayed one (see update_display_attribute()), which puts
         *          every lookup back inside [0,1]. The name starts with "__" and
         *          is left out of attribute_names(), so it can neither be
         *          selected in the GUI nor be mistaken for an attribute of the
         *          mesh; the displayed attribute itself is never modified.
         */
        static constexpr const char* DISPLAY_ATTRIBUTE_NAME = "__geobox_display";

        /**
         * @brief Inputs the display copy of the displayed attribute was built
         *        from.
         * @details Everything the copy depends on: the attribute it holds the
         *          clamped values of, and the bounds they are clamped to.
         */
        struct DisplayAttribute {
            /** Store of the displayed attribute; nullptr when there is none. */
            const void* store = nullptr;
            /** Name of the displayed attribute (e.g. "v_cell"). */
            std::string name;
            /** Scalar element read from it (e.g. 0 for "point[0]"). */
            GEO::index_t element_index = 0;
            /** Number of elements of its store. */
            GEO::index_t nb_elements = 0;
            /** Bounds of the range its values are clamped to. */
            double min = 0.0;
            double max = 0.0;
            /** True while the copy holds the values the fields above describe. */
            bool up_to_date = false;
        };
        /** State of the display copy of the displayed attribute. */
        DisplayAttribute display_attribute_;

        /**
         * @brief Rebuilds the display copy of the displayed attribute when one
         *        of its inputs changed.
         * @details The copy holds, for every element of the store the displayed
         *          attribute lives in, that attribute value clamped to
         *          [colormap_range_min(), colormap_range_max()], and is stored
         *          in DISPLAY_ATTRIBUTE_NAME. Building it scans the whole
         *          attribute and makes MeshGfx re-upload it, so it is only
         *          rebuilt when DisplayAttribute changed; it is called before
         *          drawing (see colormap_attribute_name()).
         */
        void update_display_attribute();

        /**
         * @brief Name of the attribute the colormap is sampled with.
         * @details DISPLAY_ATTRIBUTE_NAME while the display copy is up to date,
         *          and the name of the displayed attribute otherwise: the
         *          elements are then either colored with the values of that
         *          attribute as they are, or -- the attribute being the one
         *          missing -- not colored at all, as in GeoGram.
         * @return Name to pass to MeshGfx's set_scalar_attribute() or
         *         bind_if_is_defined().
         */
        std::string colormap_attribute_name();

        /**
         * @brief Name of the display-only attribute that holds the texture
         *        coordinates after the U/V scale is applied.
         * @details MeshGfx samples the image with the texture coordinates it
         *          finds in the mesh, so scaling U and V independently cannot
         *          be expressed through its texturing API (which only has a
         *          single repeat factor). The scaled coordinates are therefore
         *          materialized in this attribute, built on the store of the
         *          selected texture-coordinate attribute (see
         *          update_display_tex_coords()), and sampled with a repeat of
         *          1. Like DISPLAY_ATTRIBUTE_NAME, the name starts with "__"
         *          and is hidden from the GUI.
         */
        static constexpr const char* DISPLAY_TEX_COORD_NAME = "__geobox_uv";

        /**
         * @brief Inputs the scaled copy of the texture coordinates was built
         *        from.
         */
        struct DisplayTexCoords {
            /** Store of the source attribute; nullptr when there is none. */
            const void* store = nullptr;
            /** Name of the source attribute (e.g. "tex_coord"). */
            std::string name;
            /** Number of elements of its store. */
            GEO::index_t nb_elements = 0;
            /** Scale applied to U. */
            float scale_u = 1.0f;
            /** Scale applied to V. */
            float scale_v = 1.0f;
            /** True while the copy holds the values the fields above describe. */
            bool up_to_date = false;
        };
        /** State of the scaled copy of the texture coordinates. */
        DisplayTexCoords display_tex_coords_;

        /**
         * @brief Rebuilds the scaled copy of the texture coordinates when one
         *        of its inputs changed.
         * @details The copy holds (u * uv_scale_u_, v * uv_scale_v_) for every
         *          element of the store the selected texture-coordinate
         *          attribute lives in, and is stored in DISPLAY_TEX_COORD_NAME.
         *          It is only rebuilt when DisplayTexCoords changed.
         */
        void update_display_tex_coords();

        /**
         * @brief Name of the texture-coordinate attribute the image is sampled
         *        with.
         * @details DISPLAY_TEX_COORD_NAME while the scaled copy is up to date,
         *          and the name of the selected attribute otherwise (the scale
         *          is then 1, so no copy is needed).
         * @return Name to pass to MeshGfx's set_texturing() or to bind as
         *         "name[0]"/"name[1]".
         */
        std::string tex_coord_attribute_name_for_drawing();

        /**
         * @brief Returns a list of available scalar attribute names.
         * @return Semicolon-separated attribute names for the mesh, the display
         *         copy excluded (see DISPLAY_ATTRIBUTE_NAME).
         * @ref <geogram_gfx/gui/simple_mesh_application.cpp> attribute_names()
         */
        std::string attribute_names() const;

        /**
         * @brief Selects the active attribute used for scalar coloring.
         * @details Same as GeoGram's SimpleMeshApplication::set_attribute(): the
         *          range autoranges only while the min/max fields are still
         *          untouched (both zero), so a range the user set survives
         *          selecting another attribute.
         * @param[in] attribute Name of the mesh attribute to visualize.
         * @ref <geogram_gfx/gui/simple_mesh_application.cpp> set_attribute()
         */
        void set_attribute(const std::string& attribute);

        GEO::Mesh mesh_;
        GEO::MeshGfx mesh_gfx_;

        /** Cached bounding-box diagonal; < 0 means "not computed yet". */
        mutable float bbox_diag_ = -1.0f;

        /**
         * @brief The element breakdown above, counted when the mesh is set
         *        (constructor) or reloaded (see count_elements()).
         * @details Counted once rather than on each GUI frame: the Info panel
         *          would otherwise walk every facet and every cell of the
         *          mesh each time it is drawn, which is wasteful on large
         *          meshes.
         */
        ElementCounts element_counts_;

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

        /**
         * @brief Whether the surface has border edges, i.e. whether
         *        MeshGfx::draw_surface_borders() has anything to draw.
         * @details Cached when the mesh is set (constructor) or reloaded:
         *          scanning every facet corner on each GUI frame would be
         *          wasteful on large meshes. The border controls are hidden
         *          while it is false (see draw_object_properties()).
         */
        bool surface_has_borders_ = false;

        bool show_volume_ = true;
        float cells_shrink_ = 0.0f;
        GEO::vec4f volume_color_ = GEO::vec4f(0.9f, 0.9f, 0.9f, 1.0f);
        bool show_colored_cells_ = false;
        bool show_hexes_ = true;
        bool show_connectors_ = true;

        // Wireframe of the volume cells (the edges of mesh.cells, drawn by
        // GLUP while rendering the volume).
        bool show_volume_mesh_ = true;
        float volume_mesh_width_ = 0.1f;
        GEO::vec4f volume_mesh_color_ = GEO::vec4f(0.05f, 0.05f, 0.05f, 1.0f);

        /**
         * @brief How the mesh elements are colored (see PaintingMode).
         */
        PaintingMode painting_mode_ = PaintingMode::SOLID_COLOR;

        /**
         * @brief Path of the image sampled in TEXTURE mode.
         * @details Empty when no image is selected; the GL texture is built
         *          lazily from it (see texture()).
         */
        std::string texture_filename_;

        /**
         * @brief Repeat count of the image in the unit texture space.
         * @details 1 samples the image once over [0,1] x [0,1]; larger values
         *          repeat it, as Graphite's tex_repeat property. Only applied
         *          while tex_repeat_enabled_ is set (see texture_repeat()).
         */
        int tex_repeat_ = 1;

        /**
         * @brief Whether tex_repeat_ is applied in TEXTURE mode.
         * @details Disabled by default: the image is sampled once over the
         *          texture-coordinate range, as if tex_repeat_ were 1.
         */
        bool tex_repeat_enabled_ = false;

        /**
         * @brief Whether the U/V scale is applied in TEXTURE mode.
         * @details Disabled by default: the parameterization is used as it is,
         *          as if uv_scale_u_ and uv_scale_v_ were 1.
         */
        bool uv_scale_enabled_ = false;

        /**
         * @brief Scale applied to the U coordinate of the texture parameterization.
         * @details The image is sampled with (u * uv_scale_u_, v * uv_scale_v_);
         *          a value larger than 1 stretches the parameterization, i.e.
         *          repeats the image along that axis. Only applied while
         *          uv_scale_enabled_ is set.
         */
        float uv_scale_u_ = 1.0f;

        /**
         * @brief Scale applied to the V coordinate of the texture parameterization.
         * @see uv_scale_u_
         */
        float uv_scale_v_ = 1.0f;

        /**
         * @brief If true, U and V are scaled by the same factor.
         * @details Editing one of the two scales then updates the other one,
         *          so that the image is stretched uniformly.
         */
        bool uniform_uv_scale_ = true;

        /** Full name of the texture-coordinate attribute (e.g. "facet_corners.tex_coord"). */
        std::string tex_coord_full_name_;
        /** Base name of the texture-coordinate attribute (e.g. "tex_coord"). */
        std::string tex_coord_attribute_name_;
        /** Store the texture-coordinate attribute lives in. */
        GEO::MeshElementsFlags tex_coord_subelements_ = GEO::MESH_NONE;

        /**
         * @brief Index of the selected colormap in colormaps().
         */
        GEO::index_t current_colormap_index_ = 0;

        /**
         * @brief Whether the colormap is interpolated between its entries.
         * @details True: the colormap is sampled with GL_LINEAR filtering, so
         *          the colors of the attribute vary smoothly. False: it is
         *          sampled with GL_NEAREST filtering, so every colormap entry
         *          is a flat color band and the boundaries between the bands
         *          are hard (see colormap_texture()).
         */
        bool smooth_colormap_ = true;

        /**
         * @brief Whether the colormap is flipped, i.e. sampled back to front.
         * @details The colormap texture itself is left alone: the bounds it is
         *          sampled with are swapped instead (see
         *          colormap_mapping_min()), which reverses the mapping from
         *          values to colors.
         */
        bool flip_colormap_ = false;

        std::string vertices_point_attribute_name_ = "vertices.point[0]";
        std::string point_attribute_name_ = "point[0]";
        GEO::MeshElementsFlags attribute_subelements_ = GEO::MESH_VERTICES;
        /** Bounds of the displayed attribute range, editable in the GUI. */
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
