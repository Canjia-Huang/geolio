//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/11/13.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <gtest/gtest.h>
#include <geogram/mesh/mesh.h>
#include <geolio/geobox/object/mesh_object.h>

namespace geolio::test
{
    namespace
    {
        /// A single triangle, i.e. a surface to shade.
        void make_triangle_mesh(GEO::Mesh& mesh) {
            mesh.vertices.set_dimension(3);
            mesh.vertices.create_vertices(3);
            mesh.vertices.point(0) = GEO::vec3(0.0, 0.0, 0.0);
            mesh.vertices.point(1) = GEO::vec3(1.0, 0.0, 0.0);
            mesh.vertices.point(2) = GEO::vec3(0.0, 1.0, 0.0);
            mesh.facets.create_triangle(0, 1, 2);
        }
    }

    /**
     * @brief The shading controls ported from Graphite start on the values the
     *        object was drawn with before they existed.
     * @details No culling, as Graphite's MeshGrobShader starts; a specular
     *          factor of 4, which is the 0.4 the surface pass used to
     *          hard-code; and TRANSP_BLEND with an amount of 0, i.e. an opaque
     *          surface that turns transparent when the amount is raised, which
     *          is what the old `trsp` control did.
     */
    TEST(MeshObjectShadingTest, DefaultsKeepThePreviousLook) {
        GEO::Mesh mesh;
        make_triangle_mesh(mesh);

        const geobox::MeshObject object("mesh", "", mesh);
        EXPECT_EQ(object.culling_mode(), geobox::CullingMode::NO_CULL);
        EXPECT_EQ(object.specular(), 4);
        EXPECT_EQ(
            object.transparency_mode(),
            geobox::TransparencyMode::TRANSP_BLEND);
        EXPECT_FLOAT_EQ(object.surface_transparency(), 0.0f);
        EXPECT_FLOAT_EQ(object.surface_alpha(), 1.0f);
    }

    /**
     * @brief TRANSP_OPAQUE draws the surface opaque whatever amount of
     *        transparency it is set to, the two other modes composite that
     *        amount as the alpha of the surface.
     */
    TEST(MeshObjectShadingTest, TransparencyModeDecidesTheSurfaceAlpha) {
        GEO::Mesh mesh;
        make_triangle_mesh(mesh);

        geobox::MeshObject object("mesh", "", mesh);
        object.set_surface_transparency(0.75f);
        EXPECT_FLOAT_EQ(object.surface_alpha(), 0.25f);
        EXPECT_FLOAT_EQ(object.applied_transparency(0.75f), 0.75f);

        object.set_transparency_mode(geobox::TransparencyMode::TRANSP_ACCUM);
        EXPECT_FLOAT_EQ(object.surface_alpha(), 0.25f);
        EXPECT_FLOAT_EQ(object.applied_transparency(0.75f), 0.75f);

        object.set_transparency_mode(geobox::TransparencyMode::TRANSP_OPAQUE);
        EXPECT_FLOAT_EQ(object.surface_alpha(), 1.0f);
        // Under TRANSP_OPAQUE nothing is transparent, whatever the amount a
        // `trsp.` control is set to: the controls are locked, and the amount is
        // kept only to come back with another transparency mode.
        EXPECT_FLOAT_EQ(object.applied_transparency(0.75f), 0.0f);
        EXPECT_FLOAT_EQ(object.surface_transparency(), 0.75f);
    }

    /**
     * @brief The shading properties are what the scene panel edits, so what
     *        they are set to must be what the drawing passes read back.
     */
    TEST(MeshObjectShadingTest, ShadingPropertiesRoundTrip) {
        GEO::Mesh mesh;
        make_triangle_mesh(mesh);

        geobox::MeshObject object("mesh", "", mesh);

        object.set_culling_mode(geobox::CullingMode::CULL_BACK);
        EXPECT_EQ(object.culling_mode(), geobox::CullingMode::CULL_BACK);

        object.set_specular(10);
        EXPECT_EQ(object.specular(), 10);
        // Graphite's `specular` has no upper bound (a value above 10 only
        // saturates the highlight), and 0 is the floor: Lambert shading.
        object.set_specular(25);
        EXPECT_EQ(object.specular(), 25);
        object.set_specular(-3);
        EXPECT_EQ(object.specular(), 0);

        object.set_transparency_mode(geobox::TransparencyMode::TRANSP_ACCUM);
        EXPECT_EQ(
            object.transparency_mode(),
            geobox::TransparencyMode::TRANSP_ACCUM);
    }

    /**
     * @brief Resetting the light of the scene gives the object its default
     *        specular factor back.
     * @details The "reset" button of the light controls resets the light of the
     *          scene and, through the objects, what they bring to it.
     */
    TEST(MeshObjectShadingTest, ResetGivesTheDefaultSpecularBack) {
        GEO::Mesh mesh;
        make_triangle_mesh(mesh);

        geobox::MeshObject object("mesh", "", mesh);
        object.set_specular(25);
        ASSERT_EQ(object.specular(), 25);

        object.reset_lighting_properties();
        EXPECT_EQ(object.specular(), geobox::MeshObject::DEFAULT_SPECULAR);
        EXPECT_EQ(object.specular(), 4);
    }

    /**
     * @brief The amount of transparency is clamped to 0..1, which is also the
     *        range the slider offers.
     */
    TEST(MeshObjectShadingTest, TransparencyAmountIsClamped) {
        GEO::Mesh mesh;
        make_triangle_mesh(mesh);

        geobox::MeshObject object("mesh", "", mesh);

        object.set_surface_transparency(2.0f);
        EXPECT_FLOAT_EQ(object.surface_transparency(), 1.0f);
        EXPECT_FLOAT_EQ(object.surface_alpha(), 0.0f);

        object.set_surface_transparency(-1.0f);
        EXPECT_FLOAT_EQ(object.surface_transparency(), 0.0f);
        EXPECT_FLOAT_EQ(object.surface_alpha(), 1.0f);
    }
}
