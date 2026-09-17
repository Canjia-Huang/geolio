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

        void select_colormap(const GEO::index_t index) {
            current_colormap_index_ = index;
        }

        /// Range autorange() computed, i.e. what the min/max fields show.
        double field_min() const { return attribute_min_; }

        double field_max() const { return attribute_max_; }

        void set_field_max(const float value) { attribute_max_ = value; }

        /// Range actually mapped onto the colormap, used to sample it.
        double colormap_min() const { return colormap_range_min(); }

        double colormap_max() const { return colormap_range_max(); }

        /**
         * @brief Texel of the 256-texel colormap that \p value is colored with.
         * @details The texture coordinate computed by GLUP's texture matrix is
         *          clamped (GL_CLAMP_TO_EDGE) and sampled with GL_NEAREST, so
         *          two values are displayed with the same color exactly when
         *          they fall into the same texel.
         */
        int texel(const double value) const {
            const double low = colormap_range_min();
            const double high = colormap_range_max();
            double t = (value - low) / (high - low);
            t = std::min(std::max(t, 0.0), 1.0);
            return std::min(static_cast<int>(t * 256.0), 255);
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

        /// Number of values the scenario attributes hold: 0..10 + the sentinel.
        constexpr GEO::index_t NB_VALUES = 12;
    }

    /**
     * @brief autorange() keeps ranging over every value, sentinel included.
     * @details GeoBox leaves autorange() to GeoGram: it reports the raw
     *          [min, max] of the attribute, which is what the min/max fields
     *          show. Handling GEO::NO_INDEX is up to the colormap range.
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
     * @brief With the random colormap, every value gets its own texel.
     * @details The random colormap shows one random color per distinct value
     *          as long as the values of the range and GEO::NO_INDEX fall into
     *          different texels; that is what the range laid out by
     *          update_colormap_range() is for.
     */
    TEST(MeshObjectAttributeRangeTest, RandomColormapGivesEveryValueATexel) {
        GEO::Mesh mesh;
        make_cell_index_mesh(mesh);

        TestMeshObject object("test", test_colormaps(), mesh);
        object.select("vertices.v_cell");
        object.select_colormap(RANDOM);

        std::set<int> texels;
        for (GEO::index_t v = 0; v <= 10; ++v)
            texels.insert(object.texel(static_cast<double>(v)));
        texels.insert(object.texel(static_cast<double>(GEO::NO_INDEX)));

        // 11 indices + the sentinel, no two of them in the same texel: one
        // random color each.
        EXPECT_EQ(texels.size(), NB_VALUES);
        EXPECT_NE(
            object.texel(10.0),
            object.texel(static_cast<double>(GEO::NO_INDEX))
        );
    }

    /**
     * @brief The other colormaps keep the GeoGram range, sentinel included.
     * @details They are sampled with [attribute_min_, attribute_max_] mapped
     *          linearly, exactly as before the random colormap existed: the
     *          sentinel is part of the range and only the values at the ends of
     *          the colormap stand out.
     */
    TEST(MeshObjectAttributeRangeTest, OtherColormapsKeepTheGeoGramRange) {
        GEO::Mesh mesh;
        make_cell_index_mesh(mesh);

        TestMeshObject object("test", test_colormaps(), mesh);
        object.select("vertices.v_cell");
        object.select_colormap(RANDOM);
        // Lay the random range out first, so that the switch below has to
        // recompute it.
        EXPECT_EQ(object.texel(0.0), 10);

        object.select_colormap(VIRIDIS);
        EXPECT_DOUBLE_EQ(object.colormap_min(), object.field_min());
        EXPECT_DOUBLE_EQ(object.colormap_max(), object.field_max());

        // The whole 0..10 range is squeezed into the first texel, the sentinel
        // lands on the last one.
        for (GEO::index_t v = 0; v <= 10; ++v)
            EXPECT_EQ(object.texel(static_cast<double>(v)), 0);
        EXPECT_EQ(object.texel(static_cast<double>(GEO::NO_INDEX)), 255);
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
        // The extremes still reach the first and last texel of the colormap.
        EXPECT_EQ(object.texel(0.0), 0);
        EXPECT_EQ(object.texel(10.0), 255);
    }

    /**
     * @brief 4294967295 is an ordinary value of a non index attribute.
     * @details Only an uint32 attribute can hold the GEO::NO_INDEX sentinel;
     *          a float64 attribute that happens to contain the same number is
     *          a plain continuous attribute and keeps the linear mapping, even
     *          with the random colormap.
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

        // The huge value is still part of the displayed range...
        EXPECT_DOUBLE_EQ(object.colormap_min(), 0.0);
        EXPECT_GT(object.colormap_max(), 1.0e9);
        // ... and no band is reserved for it, so 0 keeps the first texel.
        EXPECT_EQ(object.texel(0.0), 0);
    }

    /**
     * @brief Narrowing the max field keeps the sentinel in a band of its own.
     * @details The reserved band is what a value above the max field falls
     *          back onto: such a value is out of the displayed range just like
     *          the sentinel is out of the attribute scale.
     */
    TEST(MeshObjectAttributeRangeTest, ValuesAboveTheMaxFieldShareTheReservedBand) {
        GEO::Mesh mesh;
        make_cell_index_mesh(mesh);

        TestMeshObject object("test", test_colormaps(), mesh);
        object.select("vertices.v_cell");
        object.select_colormap(RANDOM);
        // The user narrows the displayed range to 0..8.
        object.set_field_max(8.0f);

        std::set<int> texels;
        for (GEO::index_t v = 0; v <= 9; ++v)
            texels.insert(object.texel(static_cast<double>(v)));
        texels.insert(object.texel(static_cast<double>(GEO::NO_INDEX)));

        // 0..9 have a texel each, and the sentinel keeps the reserved one.
        EXPECT_EQ(texels.size(), 11u);
        EXPECT_EQ(object.texel(10.0), object.texel(double(GEO::NO_INDEX)));
        EXPECT_NE(
            object.texel(9.0),
            object.texel(static_cast<double>(GEO::NO_INDEX))
        );
    }
}
