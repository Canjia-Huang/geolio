//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/31.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <geogram/basic/attributes.h>
#include <geogram/mesh/mesh.h>
#include <geolio/geobox/object/mesh_object.h>

namespace geolio::test
{
    namespace
    {
        /// A single triangle, i.e. a surface an attribute can be displayed on.
        void make_triangle_mesh(GEO::Mesh& mesh) {
            mesh.vertices.set_dimension(3);
            mesh.vertices.create_vertices(3);
            mesh.vertices.point(0) = GEO::vec3(0.0, 0.0, 0.0);
            mesh.vertices.point(1) = GEO::vec3(1.0, 0.0, 0.0);
            mesh.vertices.point(2) = GEO::vec3(0.0, 1.0, 0.0);
            mesh.facets.create_triangle(0, 1, 2);
        }

        /// The sizing field GEO::compute_sizing_field() stores on the vertices.
        void create_weight_attribute(GEO::Mesh& mesh) {
            GEO::Attribute<double> weight(mesh.vertices.attributes(), "weight");
            weight[0] = 0.5;
            weight[1] = 1.0;
            weight[2] = 2.0;
        }
    }

    /**
     * @brief An application can ask an object to display an attribute, without
     *        going through the properties panel.
     * @details The reported need: after computing a sizing field, an application
     *          wants the mesh to show that attribute (the object's painting mode
     *          switched to ATTRIBUTE) instead of the solid color.
     */
    TEST(MeshObjectDisplayTest, DisplaysAnAttributeTheMeshHas) {
        GEO::Mesh mesh;
        make_triangle_mesh(mesh);
        create_weight_attribute(mesh);

        geobox::MeshObject object("mesh", "", mesh);
        // Like GeoGram's mesh applications, the object starts on the first
        // coordinate, painted as a solid color.
        EXPECT_EQ(object.painting_mode(), geobox::PaintingMode::SOLID_COLOR);
        EXPECT_EQ(object.displayed_attribute(), "vertices.point[0]");

        EXPECT_TRUE(object.display_attribute("vertices.weight"));
        EXPECT_EQ(object.painting_mode(), geobox::PaintingMode::ATTRIBUTE);
        EXPECT_EQ(object.displayed_attribute(), "vertices.weight");
    }

    /**
     * @brief An attribute the mesh does not have is refused, and what the object
     *        displays stays as it is.
     * @details Asking for a missing attribute must not leave the colormap
     *          sampling nothing, which would paint the mesh with the value of
     *          the unbound attribute.
     */
    TEST(MeshObjectDisplayTest, RefusesAnAttributeTheMeshDoesNotHave) {
        GEO::Mesh mesh;
        make_triangle_mesh(mesh);
        create_weight_attribute(mesh);

        geobox::MeshObject object("mesh", "", mesh);
        ASSERT_TRUE(object.display_attribute("vertices.weight"));

        EXPECT_FALSE(object.display_attribute("vertices.nothing"));
        EXPECT_EQ(object.displayed_attribute(), "vertices.weight");

        // An attribute that exists, but on another sub-element, is refused too.
        EXPECT_FALSE(object.display_attribute("facets.weight"));
        EXPECT_EQ(object.displayed_attribute(), "vertices.weight");

        // A name that is not in the "subelements.name" form as well.
        EXPECT_FALSE(object.display_attribute("weight"));
        EXPECT_FALSE(object.display_attribute(""));
        EXPECT_EQ(object.displayed_attribute(), "vertices.weight");
    }

    /**
     * @brief The painting mode is under the control of the application too, as
     *        is the visibility of the vertex markers.
     */
    TEST(MeshObjectDisplayTest, ControlsPaintingModeAndVertices) {
        GEO::Mesh mesh;
        make_triangle_mesh(mesh);

        geobox::MeshObject object("mesh", "", mesh);
        EXPECT_FALSE(object.vertices_visible());

        object.set_vertices_visible(true);
        EXPECT_TRUE(object.vertices_visible());
        object.set_vertices_visible(false);
        EXPECT_FALSE(object.vertices_visible());

        object.set_painting_mode(geobox::PaintingMode::TEXTURE);
        EXPECT_EQ(object.painting_mode(), geobox::PaintingMode::TEXTURE);
        object.set_painting_mode(geobox::PaintingMode::SOLID_COLOR);
        EXPECT_EQ(object.painting_mode(), geobox::PaintingMode::SOLID_COLOR);
    }
}
