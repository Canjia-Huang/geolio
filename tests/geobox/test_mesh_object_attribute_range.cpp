//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/20.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <gtest/gtest.h>
#include <algorithm>
#include <set>
#include <string>
#include <vector>
#include <geogram/mesh/mesh.h>
#include <geolio/geobox/object/mesh_object.h>

namespace geolio::test
{
    /**
     * @brief MeshObject with its attribute range logic made observable.
     * @details The attribute range logic is protected: this subclass forwards
     *          it to the tests and reproduces what GLUP does with the colormap
     *          range, i.e. it maps an attribute value to the colormap texel
     *          that colors it.
     */
    class TestMeshObject : public geobox::MeshObject {
    public:
        using MeshObject::MeshObject;

        void select(const std::string& attribute) { set_attribute(attribute); }

        /// The "autorange" button of the GUI.
        void press_autorange() { autorange(); }

        void select_colormap(const GEO::index_t index) {
            current_colormap_index_ = index;
        }

        /// Range autorange() computed, i.e. what the min/max fields show.
        double field_min() const { return attribute_min_; }

        double field_max() const { return attribute_max_; }

        void set_field_max(const float value) { attribute_max_ = value; }

        void set_field(const float lo, const float hi) {
            attribute_min_ = lo;
            attribute_max_ = hi;
        }

        /// Range actually mapped onto the colormap.
        double colormap_min() const { return colormap_range_min(); }

        double colormap_max() const { return colormap_range_max(); }

        /// The "smooth" and "flip" checkboxes of the GUI.
        void set_smooth(const bool smooth) { smooth_colormap_ = smooth; }

        void set_flip(const bool flip) { flip_colormap_ = flip; }

        /// Bounds the colormap is sampled with, swapped while it is flipped.
        double mapping_min() const { return colormap_mapping_min(); }

        double mapping_max() const { return colormap_mapping_max(); }

        /// Texture the colormap at \p index is sampled with.
        GLuint sampled_texture(const GEO::index_t index) const {
            return colormap_texture(index);
        }

        /// Attributes the TEXTURE mode can sample an image with.
        std::string texture_attributes() const {
            return texture_attribute_names();
        }

        /// Selects \p attribute as the texture-coordinate attribute.
        void select_tex_coords(const std::string& attribute) {
            set_tex_coords(attribute);
        }

        /// Store of the selected texture-coordinate attribute.
        GEO::MeshElementsFlags tex_coords_subelements() const {
            return tex_coord_subelements_;
        }

        /// Enables and sets the U/V scale applied to the texture coordinates.
        void set_uv_scale(const float u, const float v) {
            uv_scale_enabled_ = true;
            uv_scale_u_ = u;
            uv_scale_v_ = v;
        }

        /// Enables or disables the U/V scale.
        void enable_uv_scale(const bool enabled) {
            uv_scale_enabled_ = enabled;
        }

        /// Sets the texture repeat factor and whether it is applied.
        void set_tex_repeat(const int repeat, const bool enabled) {
            tex_repeat_ = repeat;
            tex_repeat_enabled_ = enabled;
        }

        /// Repeat factor actually passed to the texture sampling.
        GEO::index_t repeat() const { return texture_repeat(); }

        /// Name of the texture-coordinate attribute the image is sampled with.
        std::string drawing_tex_coord_name() {
            return tex_coord_attribute_name_for_drawing();
        }

        /// Component \p component of the scaled texture coordinates of
        /// \p element.
        double scaled_tex_coord(
            const GEO::index_t component,
            const GEO::index_t element
            ) {
            const GEO::MeshSubElementsStore& subelements =
                mesh().get_subelements_by_type(tex_coord_subelements_);
            GEO::ReadOnlyScalarAttributeAdapter attribute(
                subelements.attributes(),
                std::string(DISPLAY_TEX_COORD_NAME) + "[" +
                    std::to_string(component) + "]");
            return attribute[element];
        }

        /// Name of the attribute holding the scaled texture coordinates.
        static std::string display_tex_coord_name() {
            return DISPLAY_TEX_COORD_NAME;
        }

        /// Name of the attribute the colormap is sampled with.
        std::string sampled_attribute() { return colormap_attribute_name(); }

        /// Value of \p element in the attribute the colormap is sampled with,
        /// i.e. the value the GPU is given.
        double sampled_value(const GEO::index_t element) {
            const GEO::MeshSubElementsStore& subelements =
                mesh().get_subelements_by_type(attribute_subelements_);
            GEO::ReadOnlyScalarAttributeAdapter attribute(
                subelements.attributes(), colormap_attribute_name());
            return attribute[element];
        }

        /// Value \p element holds in the mesh attribute itself, which the
        /// display copy must leave untouched.
        double source_value(const GEO::index_t element) {
            const GEO::MeshSubElementsStore& subelements =
                mesh().get_subelements_by_type(attribute_subelements_);
            GEO::ReadOnlyScalarAttributeAdapter attribute(
                subelements.attributes(), point_attribute_name_);
            return attribute[element];
        }

        /// Names of the attributes offered by the attribute popup.
        std::string offered_attributes() const { return attribute_names(); }

        /// Name of the attribute that carries the values the colormap samples.
        static const char* display_attribute() {
            return DISPLAY_ATTRIBUTE_NAME;
        }

        /**
         * @brief Texel of the 256-texel colormap that \p value is colored with.
         * @details The colormap spans exactly [colormap_min(), colormap_max()]
         *          and the values outside of it are clamped to it, so two
         *          values are displayed with the same color exactly when they
         *          fall into the same texel.
         */
        int texel(const double value) const {
            const double low = colormap_range_min();
            const double high = colormap_range_max();
            if (!(high > low)) {
                return 0;
            }
            double t = (std::clamp(value, low, high) - low) / (high - low);
            return std::min(static_cast<int>(t * 256.0), 255);
        }

        /// Texel element \p element is colored with, from the values the
        /// colormap is actually sampled with.
        int element_texel(const GEO::index_t element) {
            return texel(sampled_value(element));
        }
    };

    namespace
    {
        /// Indices used to select a colormap in the tests below.
        constexpr GEO::index_t VIRIDIS = 0;
        constexpr GEO::index_t RANDOM = 1;

        /**
         * @brief Colormaps the tests select from: an ordinary one and the
         *        random one, as GeoBox builds them.
         * @details Only the names matter here: the textures are bound while
         *          drawing only. The list is static because MeshObject keeps a
         *          reference to it.
         */
        const std::vector<geobox::ColormapInfo>& test_colormaps() {
            static const std::vector<geobox::ColormapInfo> colormaps = [] {
                std::vector<geobox::ColormapInfo> result(2);
                result[VIRIDIS].name = "viridis";
                result[RANDOM].name = geobox::RANDOM_COLORMAP_NAME;
                return result;
            }();
            return colormaps;
        }

        /**
         * @brief Colormaps with distinct smoothed and nearest-filtered
         *        textures.
         * @details The ids stand for the two textures of a colormap in the
         *          application; colormap_texture() only picks one of them, so
         *          no OpenGL call is made here.
         */
        const std::vector<geobox::ColormapInfo>& filtered_test_colormaps() {
            static const std::vector<geobox::ColormapInfo> colormaps = [] {
                std::vector<geobox::ColormapInfo> result(1);
                result[0].name = "viridis";
                result[0].texture = 1;
                result[0].nearest_texture = 2;
                return result;
            }();
            return colormaps;
        }

        /**
         * @brief Fills \p mesh with the scenario of the tests below.
         * @details Twelve vertices at the origin, with an index attribute
         *          "v_cell" holding the cell indices 0..10 and GEO::NO_INDEX
         *          for the vertex that belongs to no cell.
         * @param[out] mesh Mesh to fill.
         */
        void make_cell_index_mesh(GEO::Mesh& mesh) {
            mesh.vertices.set_dimension(3);
            mesh.vertices.create_vertices(12);
            for (GEO::index_t v = 0; v < 12; ++v)
                mesh.vertices.point(v) = GEO::vec3(0.0, 0.0, 0.0);

            GEO::Attribute<GEO::index_t> v_cell(
                mesh.vertices.attributes(), "v_cell"
            );
            for (GEO::index_t v = 0; v <= 10; ++v)
                v_cell[v] = v;
            v_cell[11] = GEO::NO_INDEX;
        }

        /**
         * @brief Fills \p mesh with the reported scenario: five vertices, with
         *        the index attribute "v_cell" holding 0..3 and GEO::NO_INDEX.
         * @param[out] mesh Mesh to fill.
         */
        void make_zero_to_three_mesh(GEO::Mesh& mesh) {
            mesh.vertices.set_dimension(3);
            mesh.vertices.create_vertices(5);
            for (GEO::index_t v = 0; v < 5; ++v)
                mesh.vertices.point(v) = GEO::vec3(0.0, 0.0, 0.0);

            GEO::Attribute<GEO::index_t> v_cell(
                mesh.vertices.attributes(), "v_cell"
            );
            for (GEO::index_t v = 0; v <= 3; ++v)
                v_cell[v] = v;
            v_cell[4] = GEO::NO_INDEX;
        }

        /// Index of the vertex that holds GEO::NO_INDEX in the scenarios.
        constexpr GEO::index_t NO_INDEX_VERTEX = 4;
    }

    /**
     * @brief The colormap spans exactly the range the min/max fields read.
     */
    TEST(MeshObjectAttributeRangeTest, ColormapSpansTheMinMaxFields) {
        GEO::Mesh mesh;
        make_zero_to_three_mesh(mesh);

        TestMeshObject object("test", test_colormaps(), mesh);
        object.select("vertices.v_cell");
        object.select_colormap(VIRIDIS);
        object.set_field(0.0f, 3.0f);

        EXPECT_DOUBLE_EQ(object.colormap_min(), 0.0);
        EXPECT_DOUBLE_EQ(object.colormap_max(), 3.0);
        // The two bounds of the range get the two ends of the colormap: no band
        // is reserved above the max for GEO::NO_INDEX, as the values above the
        // max are colored like the max (see
        // ValuesOutsideTheRangeTakeTheEndColors).
        EXPECT_EQ(object.texel(0.0), 0);
        EXPECT_EQ(object.texel(3.0), 255);
    }

    /**
     * @brief A range filled in the other way round is read in increasing order.
     * @details The min/max fields describe which values to display, not which
     *          way round to sample the colormap: reversing the colors is what
     *          the "flip" checkbox is for (see
     *          FlipReversesTheColormapMapping). Reading the fields in
     *          increasing order also keeps the clamping consistent.
     */
    TEST(MeshObjectAttributeRangeTest, BoundsAreOrdered) {
        GEO::Mesh mesh;
        make_zero_to_three_mesh(mesh);

        TestMeshObject object("test", test_colormaps(), mesh);
        object.select("vertices.v_cell");
        object.set_field(3.0f, 0.0f);

        EXPECT_DOUBLE_EQ(object.colormap_min(), 0.0);
        EXPECT_DOUBLE_EQ(object.colormap_max(), 3.0);
    }

    /**
     * @brief The "flip" checkbox swaps the ends of the colormap.
     * @details It swaps the bounds the colormap is sampled with, which maps the
     *          displayed range back to front: the minimum then takes the last
     *          color of the colormap and the maximum its first one. The
     *          displayed values keep being clamped to the ordered range, so the
     *          source attribute and the display copy are unaffected.
     */
    TEST(MeshObjectAttributeRangeTest, FlipReversesTheColormapMapping) {
        GEO::Mesh mesh;
        make_zero_to_three_mesh(mesh);

        TestMeshObject object("test", test_colormaps(), mesh);
        object.select("vertices.v_cell");
        object.select_colormap(VIRIDIS);
        object.set_field(0.0f, 3.0f);

        // Unflipped, the range is sampled front to back.
        EXPECT_DOUBLE_EQ(object.mapping_min(), 0.0);
        EXPECT_DOUBLE_EQ(object.mapping_max(), 3.0);

        // Flipped, the same range is sampled back to front.
        object.set_flip(true);
        EXPECT_DOUBLE_EQ(object.mapping_min(), 3.0);
        EXPECT_DOUBLE_EQ(object.mapping_max(), 0.0);

        // The values themselves are still clamped to the ordered range, so
        // flipping mirrors the colors without changing what is displayed.
        EXPECT_DOUBLE_EQ(object.sampled_value(NO_INDEX_VERTEX), 3.0);
        EXPECT_DOUBLE_EQ(object.source_value(0), 0.0);

        // Unchecking the box restores the original mapping.
        object.set_flip(false);
        EXPECT_DOUBLE_EQ(object.mapping_min(), 0.0);
        EXPECT_DOUBLE_EQ(object.mapping_max(), 3.0);
    }

    /**
     * @brief The "smooth" checkbox selects the colormap texture.
     * @details Smooth sampling interpolates between the colormap entries;
     *          hard boundaries need the nearest-filtered texture instead.
     */
    TEST(MeshObjectAttributeRangeTest, SmoothToggleSelectsTheColormapTexture) {
        GEO::Mesh mesh;
        make_zero_to_three_mesh(mesh);

        TestMeshObject object("test", filtered_test_colormaps(), mesh);
        object.select("vertices.v_cell");

        // Smooth by default: the interpolating texture is sampled...
        EXPECT_EQ(object.sampled_texture(0), 1u);
        // ... and the nearest-filtered one when the hard boundaries are asked
        // for.
        object.set_smooth(false);
        EXPECT_EQ(object.sampled_texture(0), 2u);
        object.set_smooth(true);
        EXPECT_EQ(object.sampled_texture(0), 1u);
    }

    /**
     * @brief The TEXTURE mode only offers 2D texture-coordinate attributes.
     * @details The image is sampled with a 2D vector attribute, the class of
     *          attribute an OBJ file stores on facet corners. Scalar and 3D
     *          attributes cannot be used as texture coordinates.
     */
    TEST(MeshObjectAttributeRangeTest, TextureAttributesAreListed) {
        GEO::Mesh mesh;
        mesh.vertices.set_dimension(3);
        mesh.vertices.create_vertices(3);
        mesh.vertices.point(0) = GEO::vec3(0.0, 0.0, 0.0);
        mesh.vertices.point(1) = GEO::vec3(1.0, 0.0, 0.0);
        mesh.vertices.point(2) = GEO::vec3(0.0, 1.0, 0.0);
        mesh.facets.create_triangle(0, 1, 2);

        GEO::Attribute<double> colors;
        colors.create_vector_attribute(
            mesh.vertices.attributes(), "color", 3);
        GEO::Attribute<double> tex_coords;
        tex_coords.create_vector_attribute(
            mesh.facet_corners.attributes(), "tex_coord", 2);

        TestMeshObject object("test", test_colormaps(), mesh);

        // The 2D texture coordinates are offered...
        const std::string attributes = object.texture_attributes();
        EXPECT_NE(attributes.find("facet_corners.tex_coord"),
                  std::string::npos);
        // ... but the 3D color and point attributes are not.
        EXPECT_EQ(attributes.find("vertices.color"), std::string::npos);
        EXPECT_EQ(attributes.find("vertices.point"), std::string::npos);

        object.select_tex_coords("facet_corners.tex_coord");
        EXPECT_EQ(object.tex_coords_subelements(), GEO::MESH_FACET_CORNERS);
    }

    /**
     * @brief The U/V scale stretches the texture coordinates independently.
     * @details MeshGfx only has a single repeat factor, so the scale is applied
     *          by materializing the scaled coordinates in a display attribute
     *          (see update_display_tex_coords()), which is hidden from the
     *          attribute chooser.
     */
    TEST(MeshObjectAttributeRangeTest, UvScaleStretchesTextureCoordinates) {
        GEO::Mesh mesh;
        mesh.vertices.set_dimension(3);
        mesh.vertices.create_vertices(3);
        mesh.vertices.point(0) = GEO::vec3(0.0, 0.0, 0.0);
        mesh.vertices.point(1) = GEO::vec3(1.0, 0.0, 0.0);
        mesh.vertices.point(2) = GEO::vec3(0.0, 1.0, 0.0);
        mesh.facets.create_triangle(0, 1, 2);

        GEO::Attribute<double> tex_coords;
        tex_coords.create_vector_attribute(
            mesh.facet_corners.attributes(), "tex_coord", 2);
        for (GEO::index_t c = 0; c < mesh.facet_corners.nb(); ++c) {
            tex_coords[2 * c] = 0.1 * double(c + 1);
            tex_coords[2 * c + 1] = 0.2 * double(c + 1);
        }

        TestMeshObject object("test", test_colormaps(), mesh);
        object.select_tex_coords("facet_corners.tex_coord");

        // Disabled by default: the selected attribute is sampled directly.
        EXPECT_EQ(object.drawing_tex_coord_name(), "tex_coord");

        object.set_uv_scale(2.0f, 3.0f);
        EXPECT_EQ(object.drawing_tex_coord_name(),
                  TestMeshObject::display_tex_coord_name());
        for (GEO::index_t c = 0; c < mesh.facet_corners.nb(); ++c) {
            EXPECT_NEAR(object.scaled_tex_coord(0, c),
                        0.1 * double(c + 1) * 2.0, 1.0e-6);
            EXPECT_NEAR(object.scaled_tex_coord(1, c),
                        0.2 * double(c + 1) * 3.0, 1.0e-6);
        }

        // The display copy must not be offered as a texture-coordinate
        // attribute, nor as a scalar attribute in ATTRIBUTE mode (its
        // components are listed by get_scalar_attributes()).
        EXPECT_EQ(object.texture_attributes().find(
                      TestMeshObject::display_tex_coord_name()),
                  std::string::npos);
        EXPECT_EQ(object.offered_attributes().find(
                      TestMeshObject::display_tex_coord_name()),
                  std::string::npos);

        // Without scaling, the selected attribute is sampled directly.
        object.set_uv_scale(1.0f, 1.0f);
        EXPECT_EQ(object.drawing_tex_coord_name(), "tex_coord");

        // Non-uniform scale, the other way round.
        object.set_uv_scale(2.0f, 0.5f);
        EXPECT_EQ(object.drawing_tex_coord_name(),
                  TestMeshObject::display_tex_coord_name());
        EXPECT_NEAR(object.scaled_tex_coord(0, 0), 0.2, 1.0e-6);
        EXPECT_NEAR(object.scaled_tex_coord(1, 0), 0.1, 1.0e-6);

        // Disabling the feature goes back to the unscaled coordinates, and
        // re-enabling it rebuilds the copy with the same scales.
        object.enable_uv_scale(false);
        EXPECT_EQ(object.drawing_tex_coord_name(), "tex_coord");
        object.enable_uv_scale(true);
        EXPECT_EQ(object.drawing_tex_coord_name(),
                  TestMeshObject::display_tex_coord_name());
        EXPECT_NEAR(object.scaled_tex_coord(0, 0), 0.2, 1.0e-6);
    }

    /**
     * @brief The repeat factor is only applied when its checkbox is set.
     * @details While it is disabled the image is sampled once, as if
     *          tex_repeat were 1.
     */
    TEST(MeshObjectAttributeRangeTest, TextureRepeatIsOnlyAppliedWhenEnabled) {
        GEO::Mesh mesh;
        make_zero_to_three_mesh(mesh);

        TestMeshObject object("test", test_colormaps(), mesh);

        object.set_tex_repeat(4, false);
        EXPECT_EQ(object.repeat(), 1u);

        object.set_tex_repeat(4, true);
        EXPECT_EQ(object.repeat(), 4u);

        object.set_tex_repeat(0, true);
        EXPECT_EQ(object.repeat(), 1u);
    }

    /**
     * @brief A mesh without texture coordinates cannot be textured.
     * @details The GUI hides the TEXTURE painting mode then (see
     *          draw_object_properties()).
     */
    TEST(MeshObjectAttributeRangeTest, MeshWithoutTextureCoordinatesIsNotTexturable) {
        GEO::Mesh mesh;
        make_zero_to_three_mesh(mesh);

        TestMeshObject object("test", test_colormaps(), mesh);
        EXPECT_TRUE(object.texture_attributes().empty());
    }

    /**
     * @brief Values outside the range take the color of the nearest end.
     * @details The reported scenario: values 0..3 plus GEO::NO_INDEX. The
     *          sentinel is not a value of the attribute scale, so once the
     *          range is narrowed to the scale it must not stretch the colormap:
     *          it is colored like the max bound, and a value below the min
     *          bound like the min bound.
     */
    TEST(MeshObjectAttributeRangeTest, ValuesOutsideTheRangeTakeTheEndColors) {
        GEO::Mesh mesh;
        make_zero_to_three_mesh(mesh);

        TestMeshObject object("test", test_colormaps(), mesh);
        object.select("vertices.v_cell");
        object.select_colormap(VIRIDIS);
        object.set_field(0.0f, 3.0f);

        // The sentinel is colored like the max bound...
        EXPECT_EQ(object.texel(static_cast<double>(GEO::NO_INDEX)),
                  object.texel(3.0));
        // ... and any value below the min bound like the min bound.
        EXPECT_EQ(object.texel(-1000.0), object.texel(0.0));

        // What the colormap samples is the clamped value, and the mesh
        // attribute keeps the values it was loaded with.
        EXPECT_DOUBLE_EQ(
            object.sampled_value(NO_INDEX_VERTEX), 3.0);
        EXPECT_DOUBLE_EQ(
            object.source_value(NO_INDEX_VERTEX),
            static_cast<double>(GEO::NO_INDEX));
    }

    /**
     * @brief The values of the range are mapped linearly onto the colormap.
     */
    TEST(MeshObjectAttributeRangeTest, ValuesInsideTheRangeAreMappedLinearly) {
        GEO::Mesh mesh;
        make_cell_index_mesh(mesh);

        TestMeshObject object("test", test_colormaps(), mesh);
        object.select("vertices.v_cell");
        object.select_colormap(VIRIDIS);
        object.set_field(0.0f, 4.0f);

        // [0,4] -> [0,1] -> texel: the values of the range are spread over the
        // whole colormap, in order.
        for (GEO::index_t v = 0; v <= 3; ++v) {
            EXPECT_EQ(object.element_texel(v), static_cast<int>(v) * 64)
                << "value " << v;
        }
    }

    /**
     * @brief autorange() keeps ranging over every value, sentinel included.
     * @details GeoBox leaves autorange() to GeoGram: it reports the raw
     *          [min, max] of the attribute, which is what the min/max fields
     *          show. Such a range squeezes the values of the scale into the
     *          first texel; narrowing the fields is what spreads them over the
     *          colormap (see ColormapSpansTheMinMaxFields).
     */
    TEST(MeshObjectAttributeRangeTest, AutorangeStillCoversNoIndex) {
        GEO::Mesh mesh;
        make_cell_index_mesh(mesh);

        TestMeshObject object("test", test_colormaps(), mesh);
        object.select("vertices.v_cell");

        EXPECT_DOUBLE_EQ(object.field_min(), 0.0);
        EXPECT_GE(object.field_max(), 1.0e9);
    }

    /**
     * @brief The random colormap follows the range like every other one.
     * @details It is meant to give an attribute with few distinct values one
     *          color each, which the user gets by setting the min/max fields to
     *          the values of the scale: no band is reserved for GEO::NO_INDEX,
     *          which then shares the color of the max bound (see
     *          ValuesOutsideTheRangeTakeTheEndColors).
     */
    TEST(MeshObjectAttributeRangeTest, RandomColormapFollowsTheRange) {
        GEO::Mesh mesh;
        make_cell_index_mesh(mesh);

        TestMeshObject object("test", test_colormaps(), mesh);
        object.select("vertices.v_cell");
        object.select_colormap(RANDOM);
        object.set_field(0.0f, 10.0f);

        // 0..10, no two of them in the same texel: one random color each.
        std::set<int> texels;
        for (GEO::index_t v = 0; v <= 10; ++v)
            texels.insert(object.element_texel(v));
        EXPECT_EQ(texels.size(), 11u);

        // The sentinel is out of the scale: it is colored like the value that
        // closes the range.
        EXPECT_EQ(object.element_texel(11), object.element_texel(10));
    }

    /**
     * @brief The display copy follows every change of the range.
     * @details It is only rebuilt when one of its inputs changed, so the tests
     *          pin down that the range fields and the displayed attribute are
     *          part of those inputs: a stale copy would color the elements with
     *          the values of a range the fields no longer read.
     */
    TEST(MeshObjectAttributeRangeTest, DisplayCopyFollowsTheRange) {
        GEO::Mesh mesh;
        make_zero_to_three_mesh(mesh);

        TestMeshObject object("test", test_colormaps(), mesh);
        object.select("vertices.v_cell");
        object.select_colormap(VIRIDIS);
        object.set_field(0.0f, 3.0f);
        EXPECT_DOUBLE_EQ(object.sampled_value(NO_INDEX_VERTEX), 3.0);
        EXPECT_EQ(object.element_texel(NO_INDEX_VERTEX), 255);

        // Widening the max field: the sentinel is clamped to the new bound.
        object.set_field_max(10.0f);
        EXPECT_DOUBLE_EQ(object.colormap_max(), 10.0);
        EXPECT_DOUBLE_EQ(object.sampled_value(NO_INDEX_VERTEX), 10.0);

        // Narrowing the range again: the values inside of it are untouched and
        // the sentinel goes back to the last texel.
        object.set_field(-5.0f, 4.0f);
        EXPECT_DOUBLE_EQ(object.sampled_value(0), 0.0);
        EXPECT_EQ(object.element_texel(NO_INDEX_VERTEX), 255);

        // The copy is rebuilt for another attribute too: it belongs to the
        // store of the attribute it is built from, not to that attribute.
        object.select("vertices.point[0]");
        EXPECT_EQ(object.sampled_attribute(),
                  TestMeshObject::display_attribute());
        EXPECT_DOUBLE_EQ(object.sampled_value(0), 0.0);
    }

    /**
     * @brief The display copy is a display artefact, not an attribute.
     * @details The clamped values the colormap samples are stored in an
     *          attribute of the mesh (MeshGfx samples what the mesh holds), so
     *          the tests pin down that it is invisible to the rest of the
     *          application and that it does not alter the source attribute.
     */
    TEST(MeshObjectAttributeRangeTest, DisplayCopyIsHiddenFromTheAttributeList) {
        GEO::Mesh mesh;
        make_zero_to_three_mesh(mesh);

        TestMeshObject object("test", test_colormaps(), mesh);
        object.select("vertices.v_cell");
        object.set_field(0.0f, 3.0f);

        // The colormap samples the display copy...
        EXPECT_EQ(object.sampled_attribute(),
                  TestMeshObject::display_attribute());
        // ... which lives in the mesh the object displays (a copy of the one
        // handed to the constructor) and is not offered as an attribute.
        EXPECT_TRUE(
            object.mesh().vertices.attributes().is_defined(
                TestMeshObject::display_attribute()));
        // ... which is not offered as an attribute to display, and leaves the
        // displayed attribute itself alone.
        EXPECT_EQ(object.offered_attributes().find(
                      TestMeshObject::display_attribute()),
                  std::string::npos);
        EXPECT_NE(object.offered_attributes().find("vertices.v_cell"),
                  std::string::npos);
        for (GEO::index_t v = 0; v <= 3; ++v)
            EXPECT_DOUBLE_EQ(object.source_value(v),
                             static_cast<double>(v));
        EXPECT_DOUBLE_EQ(object.source_value(NO_INDEX_VERTEX),
                         static_cast<double>(GEO::NO_INDEX));
    }

    /**
     * @brief Selecting an attribute never discards a range the user set.
     * @details Same as GeoGram: set_attribute() autoranges only while the
     *          min/max fields are still untouched (both zero). The range of the
     *          fields describes the values the user wants to look at, so
     *          replacing it with the range of the newly selected attribute
     *          would color the elements with a range the fields no longer read.
     */
    TEST(MeshObjectAttributeRangeTest, UserRangeSurvivesSelectingAnAttribute) {
        GEO::Mesh mesh;
        make_cell_index_mesh(mesh);

        TestMeshObject object("test", test_colormaps(), mesh);
        object.select("vertices.v_cell");
        object.select_colormap(VIRIDIS);
        object.set_field(0.0f, 4.0f);

        // Selecting the attribute again (what the attribute popup does when the
        // user clicks its name) keeps the range the user set...
        object.select("vertices.v_cell");
        EXPECT_DOUBLE_EQ(object.field_min(), 0.0);
        EXPECT_DOUBLE_EQ(object.field_max(), 4.0);
        // ... and so does selecting another attribute.
        object.select("vertices.point[0]");
        EXPECT_DOUBLE_EQ(object.field_min(), 0.0);
        EXPECT_DOUBLE_EQ(object.field_max(), 4.0);

        // The "autorange" button gives the raw range back, GEO::NO_INDEX
        // included, as GeoGram's does.
        object.select("vertices.v_cell");
        object.press_autorange();
        EXPECT_DOUBLE_EQ(object.field_min(), 0.0);
        EXPECT_GE(object.field_max(), 1.0e9);
    }

    /**
     * @brief Without a sentinel, values are mapped linearly, as before.
     */
    TEST(MeshObjectAttributeRangeTest, ContinuousAttributeIsMappedLinearly) {
        GEO::Mesh mesh;
        mesh.vertices.set_dimension(3);
        mesh.vertices.create_vertices(11);
        GEO::Attribute<double> quality(
            mesh.vertices.attributes(), "quality"
        );
        for (GEO::index_t v = 0; v <= 10; ++v) {
            mesh.vertices.point(v) = GEO::vec3(0.0, 0.0, 0.0);
            quality[v] = static_cast<double>(v);
        }

        TestMeshObject object("test", test_colormaps(), mesh);
        object.select("vertices.quality");
        object.select_colormap(RANDOM);

        EXPECT_DOUBLE_EQ(object.colormap_min(), 0.0);
        EXPECT_DOUBLE_EQ(object.colormap_max(), 10.0);
        // The extremes still reach the first and last texel of the colormap,
        // and nothing is clamped: the copy holds the values themselves.
        EXPECT_EQ(object.texel(0.0), 0);
        EXPECT_EQ(object.texel(10.0), 255);
        for (GEO::index_t v = 0; v <= 10; ++v)
            EXPECT_DOUBLE_EQ(
                object.sampled_value(v), object.source_value(v));
    }

    /**
     * @brief 4294967295 is an ordinary value of a non index attribute.
     * @details Only an uint32 attribute can hold the GEO::NO_INDEX sentinel;
     *          a float64 attribute that happens to contain the same number is
     *          a plain continuous attribute and is mapped like any other value.
     */
    TEST(MeshObjectAttributeRangeTest, SentinelIsNotSpecialForFloatAttributes) {
        GEO::Mesh mesh;
        mesh.vertices.set_dimension(3);
        mesh.vertices.create_vertices(2);
        for (GEO::index_t v = 0; v < 2; ++v)
            mesh.vertices.point(v) = GEO::vec3(0.0, 0.0, 0.0);
        GEO::Attribute<double> quality(
            mesh.vertices.attributes(), "quality"
        );
        quality[0] = 0.0;
        quality[1] = static_cast<double>(GEO::NO_INDEX);

        TestMeshObject object("test", test_colormaps(), mesh);
        object.select("vertices.quality");
        object.select_colormap(RANDOM);

        // The huge value is part of the displayed range...
        EXPECT_DOUBLE_EQ(object.colormap_min(), 0.0);
        EXPECT_GT(object.colormap_max(), 1.0e9);
        // ... so it is not clamped: 0 keeps the first texel and the huge value
        // the last one.
        EXPECT_EQ(object.texel(0.0), 0);
        EXPECT_EQ(object.element_texel(1), 255);
    }
}
