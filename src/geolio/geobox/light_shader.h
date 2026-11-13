//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/11/13.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_LIGHT_SHADER_H
#define GEOLIO_LIGHT_SHADER_H

#include <array>

#include <geogram_gfx/GLUP/GLUP.h>
#include <geogram_gfx/basic/GL.h>

#include "geolio/geobox/scene_light.h"

namespace geolio::geobox
{
    /**
     * @brief The GLUP programs GeoBox shades the geometry with.
     * @details GLUP shades with a single directional light whose color, intensity,
     *          ambient level and specular exponent are constants of its fragment
     *          shader (see glup_lighting() in
     *          geogram_gfx/GLUP/shaders/GLUP/fragment_shader_utils.h). GLUP lets a
     *          client program take over, though: glupUseProgram() makes it draw
     *          with a program of ours, glupCompileShader() compiles a stage of it
     *          with the preamble, the primitive, the toggles and the imports of
     *          GLUP's own shaders, and the program is linked with the vertex
     *          attributes of GLUP's own programs (see compile()).
     *          The programs below are GLUP's own shaders with their
     *          glup_lighting() replaced by one that reads the parameters of the
     *          light of the scene (see SceneLight), which is what makes the light
     *          color, intensity, ambient level and highlight exponent
     *          controllable. Nothing else of GLUP's shading changes: texturing,
     *          vertex colors, clipping, the wireframe of the facets and of the
     *          cells, and alpha discard are the ones of its shaders.
     *          There is one program per primitive, because GLUP compiles the
     *          wireframe of the cells into its shaders from the primitive drawn
     *          (the mesh texture coordinates of a tetrahedron and of a hexahedron
     *          are not the same): a program compiled for a primitive can only draw
     *          that primitive, and GLUP has a single program bound for a whole
     *          draw call, which is why the cells are drawn one type at a time (see
     *          MeshObject::draw_volume()).
     *          The programs are those of GLUP's ES2 profile, the profile geogram
     *          uses on Apple platforms (see glupCreateContext()); supported()
     *          reports whether it is the profile in use.
     */
    class LightShader {
    public:
        LightShader() = default;
        LightShader(const LightShader&) = delete;
        LightShader& operator=(const LightShader&) = delete;

        /**
         * @brief Whether GLUP runs the profile these programs are written for.
         * @details The sources are the ones of GLUP's ES2 profile. On the profiles
         *          that draw quads and cells with geometry shaders (GLSL 150, ...)
         *          GLUP keeps its own programs, and the light keeps the parameters
         *          of its shaders.
         * @return true when GLUP draws with the ES2 profile.
         */
        [[nodiscard]] static bool supported();

        /**
         * @brief Makes GLUP draw with the program of a primitive.
         * @details Compiles the program the first time that primitive is drawn.
         *          Everything drawn until end() is drawn with it.
         * @param[in] light Parameters of the light to shade with.
         * @param[in] primitive Primitive that is about to be drawn; the program of
         *            GLUP_TRIANGLES, the default, draws the surfaces.
         * @return true when the program is in use; false when it is not available
         *         (see supported()), in which case GLUP keeps its own program and
         *         nothing was changed.
         */
        bool begin(
            const SceneLight& light,
            GLUPprimitive primitive = GLUP_TRIANGLES
            );

        /**
         * @brief Gives GLUP its own program back (see begin()).
         */
        void end();

        /**
         * @brief Releases the programs.
         * @details Called while the GL context is still current (see
         *          GeoBoxApplication::GL_terminate()); the programs are not
         *          released by the destructor, which runs after the context is
         *          gone.
         */
        void release();

        /**
         * @brief The source of the program of a primitive.
         * @details Exposed for the tests, which check that it still declares what
         *          GLUP needs to upload its state to it.
         * @param[in] primitive Primitive that the program draws.
         * @return The source: the primitive, the shader stages and the imports of
         *         GLUP's own shaders.
         */
        [[nodiscard]] static const char* program_source(
            GLUPprimitive primitive = GLUP_TRIANGLES
            );

    private:
        /**
         * @brief One program, and the light it was last shaded with.
         * @details The uniform values of a program are its own state: they are set
         *          on it once, and only again when the light changes.
         */
        struct Program {
            /** The GLUP program, 0 when it could not be compiled. */
            GLuint id = 0;
            /** Whether compiler() already tried to build it. */
            bool tried = false;
            /** Locations of the uniforms the program adds to GLUP's state. */
            GLint light_color_location = -1;
            GLint light_intensity_location = -1;
            GLint light_ambient_location = -1;
            GLint specular_exponent_location = -1;
            /** The light the uniform values of the program are set to. */
            SceneLight uploaded_light;
            /** Whether uploaded_light describes what the program holds. */
            bool uploaded = false;
        };

        /**
         * @brief The program of a primitive, compiled on first use.
         * @param[in] primitive Primitive to draw with it.
         * @return The program; its id is 0 when it is not available.
         */
        Program& program(GLUPprimitive primitive);

        /**
         * @brief Compiles the program of a primitive.
         * @return true when the program is ready to draw with.
         */
        bool compile(Program& program, GLUPprimitive primitive);

        /**
         * @brief Sets the uniforms of a program to a light.
         */
        void upload(Program& program, const SceneLight& light);

        /** The programs, indexed by GLUP primitive. */
        std::array<Program, GLUP_NB_PRIMITIVES> programs_;
    };

    /**
     * @brief The programs GeoBox shades the geometry with.
     * @details There is one set of programs, as there is one light: a program is a
     *          setting of GLUP, not of an object (see glupUseProgram()).
     * @return The programs.
     */
    [[nodiscard]] LightShader& light_shader();
}

#endif //GEOLIO_LIGHT_SHADER_H
