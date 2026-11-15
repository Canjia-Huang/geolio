//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/11/13.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <gtest/gtest.h>
#include <cmath>
#include <string>
#include <geolio/geobox/light_shader.h>
#include <geolio/geobox/scene_light.h>

namespace geolio::test
{
    namespace
    {
        void expect_direction(
            const GEO::vec3& actual,
            const GEO::vec3& expected,
            const double tolerance = 1e-9
            ) {
            EXPECT_NEAR(actual.x, expected.x, tolerance);
            EXPECT_NEAR(actual.y, expected.y, tolerance);
            EXPECT_NEAR(actual.z, expected.z, tolerance);
        }
    }

    /**
     * @brief The light of the scene is the (1,1,1) direction until it is
     *        rotated: that is the direction the application starts it on, and
     *        the one the "reset light" button puts it back on.
     */
    TEST(SceneLightTest, StartsOnTheDiagonal) {
        GEO::mat4 identity;
        identity.load_identity();

        expect_direction(
            geobox::scene_light_direction(identity),
            GEO::normalize(GEO::vec3(1.0, 1.0, 1.0)));

        const GEO::vec2 angles = geobox::scene_light_angles(
            geobox::scene_light_reference_direction());
        EXPECT_NEAR(angles.x, 45.0, 1e-9);
        EXPECT_NEAR(angles.y, 35.264389682754654, 1e-9);
    }

    /**
     * @brief The rotation computed for a direction points the light along it.
     * @details The application applies the rotation of the light to the
     *          reference direction as a row vector (see
     *          SimpleApplication::draw_scene_begin()), so the rotation is stored
     *          transposed: this is what the test checks, whatever side of the
     *          matrix the direction ends up on.
     */
    TEST(SceneLightTest, RotationPointsTheLightAlongTheDirection) {
        const GEO::vec3 directions[] = {
            GEO::vec3(1.0, 0.0, 0.0),
            GEO::vec3(0.0, 1.0, 0.0),
            GEO::vec3(0.0, 0.0, 1.0),
            GEO::vec3(-1.0, 0.0, 0.0),
            GEO::vec3(0.0, -1.0, 0.0),
            GEO::vec3(0.0, 0.0, -1.0),
            GEO::vec3(1.0, 1.0, -1.0),
            GEO::vec3(-2.0, 3.0, 0.5)
        };
        for (const GEO::vec3& direction : directions) {
            const GEO::vec3 expected = GEO::normalize(direction);
            expect_direction(
                geobox::scene_light_direction(
                    geobox::scene_light_rotation(direction)),
                expected,
                1e-9);
        }
    }

    /**
     * @brief The direction the light is reset to is the one the rotation that
     *        points it there is the identity for.
     */
    TEST(SceneLightTest, ReferenceDirectionIsNotRotated) {
        GEO::mat4 rotation = geobox::scene_light_rotation(
            geobox::scene_light_reference_direction());
        for (GEO::index_t i = 0; i < 4; ++i) {
            for (GEO::index_t j = 0; j < 4; ++j) {
                EXPECT_NEAR(rotation(i, j), i == j ? 1.0 : 0.0, 1e-9);
            }
        }
    }

    /**
     * @brief The opposite of the reference direction is reached by a half turn:
     *        the axis of the rotation is then undefined, which must not give a
     *        light that points anywhere else.
     */
    TEST(SceneLightTest, OppositeDirectionIsAHalfTurn) {
        const GEO::vec3 opposite = -geobox::scene_light_reference_direction();
        expect_direction(
            geobox::scene_light_direction(geobox::scene_light_rotation(opposite)),
            opposite,
            1e-9);
    }

    /**
     * @brief The azimuth and the elevation the viewer panel edits describe the
     *        direction of the light, both ways.
     */
    TEST(SceneLightTest, AnglesAndDirectionAgree) {
        const GEO::vec2 angles[] = {
            GEO::vec2(0.0, 0.0),
            GEO::vec2(90.0, 0.0),
            GEO::vec2(-180.0, 0.0),
            GEO::vec2(45.0, 35.0),
            GEO::vec2(-135.0, -60.0)
        };
        for (const GEO::vec2& expected : angles) {
            const GEO::vec3 direction =
                geobox::scene_light_direction_from_angles(expected);
            const GEO::vec2 actual = geobox::scene_light_angles(direction);
            EXPECT_NEAR(actual.x, expected.x, 1e-9);
            EXPECT_NEAR(actual.y, expected.y, 1e-9);
        }

        // Azimuth 0, elevation 0: the light comes from the X axis of the view.
        expect_direction(
            geobox::scene_light_direction_from_angles(GEO::vec2(0.0, 0.0)),
            GEO::vec3(1.0, 0.0, 0.0));
        // Elevation 90: it comes from above, whatever the azimuth.
        expect_direction(
            geobox::scene_light_direction_from_angles(GEO::vec2(120.0, 90.0)),
            GEO::vec3(0.0, 0.0, 1.0));
    }

    /**
     * @brief The light of the scene starts on the constants of GLUP's shader, so
     *        that a scene drawn with it looks the way it looked before the light
     *        could be set.
     */
    TEST(SceneLightTest, StartsOnTheConstantsOfGLUP) {
        geobox::SceneLight& light = geobox::scene_light();
        light = geobox::SceneLight();

        EXPECT_FLOAT_EQ(light.color.x, 1.0f);
        EXPECT_FLOAT_EQ(light.color.y, 1.0f);
        EXPECT_FLOAT_EQ(light.color.z, 1.0f);
        EXPECT_FLOAT_EQ(light.intensity, 1.0f);
        EXPECT_FLOAT_EQ(light.ambient, 0.2f);
        EXPECT_FLOAT_EQ(light.specular_exponent, 30.0f);
    }

    /**
     * @brief The program GeoBox shades the surfaces with is complete for
     *        glupCompileProgram().
     * @details It has to say which primitive it draws and which shader stages it
     *          has, to import GCUP's own vertex shader (which feeds the fragment
     *          shader) and the declaration of GLUP's uniform state (which GLUP
     *          uploads to it), and to declare the parameters of the light of the
     *          scene the fragment shader reads. A regression on any of these would
     *          only show as a program that does not compile at run time.
     */
    TEST(SceneLightTest, SurfaceProgramIsComplete) {
        const std::string source = geobox::LightShader::program_source();

        EXPECT_NE(source.find("//primitive GLUP_TRIANGLES"), std::string::npos);
        EXPECT_NE(source.find("//stage GL_VERTEX_SHADER"), std::string::npos);
        EXPECT_NE(source.find("//stage GL_FRAGMENT_SHADER"), std::string::npos);
        EXPECT_NE(
            source.find("//import <GLUPES/vertex_shader.h>"),
            std::string::npos);
        EXPECT_NE(
            source.find(
                "//import <GLUP/current_profile/fragment_shader_preamble.h>"),
            std::string::npos);
        EXPECT_NE(
            source.find("//import <GLUPES/fragment_shader_state.h>"),
            std::string::npos);

        EXPECT_NE(source.find("geolio_light_color"), std::string::npos);
        EXPECT_NE(source.find("geolio_light_intensity"), std::string::npos);
        EXPECT_NE(source.find("geolio_light_ambient"), std::string::npos);
        EXPECT_NE(source.find("geolio_specular_exponent"), std::string::npos);

        // The shading of GLUP's own fragment shader is reused, only the lighting
        // is GeoBox's.
        EXPECT_NE(source.find("glup_alpha_discard()"), std::string::npos);
        EXPECT_NE(source.find("void main()"), std::string::npos);


        // The cells are drawn with the program of their primitive: GLUP derives
        // the wireframe of a cell from the primitive drawn, so each cell type has
        // its own program.
        const struct {
            GLUPprimitive primitive;
            const char* name;
        } cell_programs[] = {
            {GLUP_TETRAHEDRA, "GLUP_TETRAHEDRA"},
            {GLUP_HEXAHEDRA, "GLUP_HEXAHEDRA"},
            {GLUP_PRISMS, "GLUP_PRISMS"},
            {GLUP_PYRAMIDS, "GLUP_PYRAMIDS"},
            {GLUP_CONNECTORS, "GLUP_CONNECTORS"}
        };
        for (const auto& cell_program : cell_programs) {
            const std::string cell_source = geobox::LightShader::program_source(
                cell_program.primitive);
            EXPECT_NE(
                cell_source.find(
                    std::string("//primitive ") + cell_program.name + "\n"),
                std::string::npos);
            EXPECT_NE(
                cell_source.find("//stage GL_FRAGMENT_SHADER"),
                std::string::npos);
        }
    }

    /**
     * @brief The colormap rescale maps the range of the attribute onto [0,1], so
     *        that the colormap spans that range instead of the raw values.
     * @details The coefficients are the ones of the texture matrix GLUP builds for
     *          the same range (see glupMapTexCoords1d()), which the vertex shader of
     *          the ES2 profile does not apply to a client program (see
     *          TexCoordMapping).
     */
    TEST(TexCoordMappingTest, RescaleMapsTheRangeOntoTheColormap) {
        geobox::TexCoordMapping mapping;
        EXPECT_FALSE(mapping.rescale);

        // Nothing is rescaled: the texture coordinates are used as they are.
        EXPECT_DOUBLE_EQ(geobox::tex_coord_rescale(mapping)[0], 1.0);
        EXPECT_DOUBLE_EQ(geobox::tex_coord_rescale(mapping)[1], 0.0);

        // A range of [-1,1]: the values are halved and shifted by a half, so that
        // -1 gets the first color of the colormap and 1 its last one.
        mapping.rescale = true;
        mapping.min = -1.0;
        mapping.max = 1.0;
        EXPECT_DOUBLE_EQ(geobox::tex_coord_rescale(mapping)[0], 0.5);
        EXPECT_DOUBLE_EQ(geobox::tex_coord_rescale(mapping)[1], 0.5);

        // A range of [0,10]: the values are divided by ten, and not shifted.
        mapping.min = 0.0;
        mapping.max = 10.0;
        EXPECT_DOUBLE_EQ(geobox::tex_coord_rescale(mapping)[0], 0.1);
        EXPECT_DOUBLE_EQ(geobox::tex_coord_rescale(mapping)[1], 0.0);

        // A range that spans nothing cannot be mapped onto the colormap: the
        // coordinates are left as they are.
        mapping.min = 3.0;
        mapping.max = 3.0;
        EXPECT_DOUBLE_EQ(geobox::tex_coord_rescale(mapping)[0], 1.0);
        EXPECT_DOUBLE_EQ(geobox::tex_coord_rescale(mapping)[1], 0.0);
    }
}
