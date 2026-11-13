//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/11/13.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "light_shader.h"

#include <cstring>
#include <map>
#include <string>

#include <geogram_gfx/GLUP/GLUP.h>
#include <geogram_gfx/basic/GLSL.h>

#include "geolio/common/log.h"

namespace geolio::geobox
{
    namespace
    {
        /**
         * @brief The vertex shader of the programs: GLUP's own one.
         * @details It feeds the view space position, the normals, the colors, the
         *          texture coordinates and the vertex ids the fragment shader and
         *          the wireframe of the cells read, so none of that belongs to
         *          GeoBox; only the shading of the fragments does.
         */
        const char* const VERTEX_SHADER =
            "//import <GLUPES/vertex_shader.h>\n";

        /**
         * @brief The indices GLUP binds its vertex attributes to.
         * @details They are the values of GLUP's GLUPattribute enum (see
         *          GLUP_context.h), which GLUP binds for its own programs. A client
         *          program has to bind them too, and the last one -- the vertex id,
         *          which the vertex shader reads to build the texture coordinates of
         *          the wireframe of the cells -- is left out by
         *          glupCompileProgram(), which is why the programs are linked here.
         */
        constexpr GLint ATTRIBUTE_VERTEX = 0;
        constexpr GLint ATTRIBUTE_COLOR = 1;
        constexpr GLint ATTRIBUTE_TEX_COORD = 2;
        constexpr GLint ATTRIBUTE_NORMAL = 3;
        constexpr GLint ATTRIBUTE_VERTEX_ID = 4;

        /**
         * @brief The name of a primitive, as GLUP spells it in its sources.
         * @param[in] primitive Primitive to name.
         * @return The name GLUP's own sources give it.
         */
        const char* glup_primitive_name(GLUPprimitive primitive) {
            switch (primitive) {
            case GLUP_POINTS: return "GLUP_POINTS";
            case GLUP_LINES: return "GLUP_LINES";
            case GLUP_TRIANGLES: return "GLUP_TRIANGLES";
            case GLUP_QUADS: return "GLUP_QUADS";
            case GLUP_TETRAHEDRA: return "GLUP_TETRAHEDRA";
            case GLUP_HEXAHEDRA: return "GLUP_HEXAHEDRA";
            case GLUP_PRISMS: return "GLUP_PRISMS";
            case GLUP_PYRAMIDS: return "GLUP_PYRAMIDS";
            case GLUP_CONNECTORS: return "GLUP_CONNECTORS";
            case GLUP_SPHERES: return "GLUP_SPHERES";
            default: return "GLUP_TRIANGLES";
            }
        }

        const char* const FRAGMENT_SHADER = R"GLSL(
//import <GLUP/current_profile/fragment_shader_preamble.h>
//import <GLUPES/fragment_shader_state.h>
//import <GLUP/stdglup.h>
//import <GLUP/current_profile/toggles.h>
//import <GLUP/current_profile/primitive.h>
//import <GLUPES/fragment_shader_utils.h>

// The light of the scene (see SceneLight). GLUP's own lighting has them as
// constants: the ambient level 0.2, a white light of intensity 1 and a specular
// highlight raised to the power 30.
uniform vec3 geolio_light_color;
uniform float geolio_light_intensity;
uniform float geolio_light_ambient;
uniform float geolio_specular_exponent;

glup_in vec3 vertex_view_space;
glup_in float clip_dist;
glup_in vec4 color;
glup_in vec4 tex_coord;
glup_in vec4 mesh_tex_coord;
glup_flat glup_in glup_id primitive_id;

#if GLUP_PRIMITIVE_DIMENSION==2
glup_in vec3 normal;
#endif

vec4 geolio_lighting(in vec4 color_in, in vec3 normal_in) {
    // GLUP's shading model (see glup_lighting() of
    // GLUPES/fragment_shader_utils.h), with the light of the scene: the diffuse
    // term is scaled by the intensity of the light and tinted by its color, and
    // the specular term by both; the ambient level is added to every fragment.
    vec3 ambient = vec3(geolio_light_ambient);
    float diff = dot(normal_in, GLUP.light_vector);
    if(diff <= 0.0) {
        return vec4(ambient, color_in.a);
    }
    vec3 result = diff * color_in.rgb;
    if(GLUP.specular > 0.0) {
        float spec = dot(normal_in, GLUP.light_half_vector);
        if(spec > 0.0) {
            result += GLUP.specular * pow(spec, geolio_specular_exponent);
        }
    }
    return vec4(
        geolio_light_intensity * geolio_light_color * result + ambient,
        color_in.a
    );
}

vec4 geolio_shading(
    in vec4 color_in,
    in vec4 tex_coord_in,
    in vec3 normal_in,
    in highp int primitive_id_in,
    in vec4 mesh_tex_coord_in
) {
    if(glupIsEnabled(GLUP_PICKING)) {
        return glup_picking(primitive_id_in);
    }
    vec4 result;
    if(glupIsEnabled(GLUP_VERTEX_COLORS)) {
        result = color_in;
    } else {
        result = gl_FrontFacing ? GLUP.front_color : GLUP.back_color;
    }
    if(glupIsEnabled(GLUP_TEXTURING) && !glupIsEnabled(GLUP_NORMAL_MAPPING)) {
        result = glup_texturing(result, tex_coord_in);
    }
    if(glupIsEnabled(GLUP_LIGHTING)) {
        result = geolio_lighting(result, normal_in);
    }
    if(
        glupIsEnabled(GLUP_DRAW_MESH) && (
            (glup_primitive_dimension == 2) ||
            !glupIsEnabled(GLUP_CLIPPING) ||
            GLUP.clipping_mode != GLUP_CLIP_SLICE_CELLS
        )
    ) {
        result = glup_draw_mesh(result, mesh_tex_coord_in);
    }
    return result;
}

void main() {

    if(glupIsEnabled(GLUP_CLIPPING)) {
        if(glup_primitive_dimension == 2) {
            if(clip_dist < 0.0) {
                discard;
            }
        } else if(glup_primitive_dimension == 3) {
            if(
                clip_dist < 0.0 &&
                GLUP.clipping_mode == GLUP_CLIP_STANDARD
            ) {
                discard;
            }
        }
    }

    vec3 N;
    if(glupIsEnabled(GLUP_LIGHTING)) {
        if(
            glupIsEnabled(GLUP_TEXTURING) &&
            glupIsEnabled(GLUP_NORMAL_MAPPING)
        ) {
            N = glup_texturing(vec4(1.0,1.0,1.0,1.0), tex_coord).xyz;
            N = N-vec3(0.5,0.5,0.5);
            N = normalize(GLUP.normal_matrix*N);
            if(!gl_FrontFacing) {
                N = -N;
            }
        } else {
#if GLUP_PRIMITIVE_DIMENSION==2
            if(glupIsEnabled(GLUP_VERTEX_NORMALS)) {
                N = normalize(normal);
                if(!gl_FrontFacing) {
                    N = -N;
                }
            } else
#endif
            {
                // Note: we still compute view space vertex
                // in vertex shader, because we seemingly do
                // not have sufficient precision to do the same
                // trick as in GLUP_GLSL (i.e. going back to
                // view space from clip space using projection
                // matrix).
                vec3 U = dFdx(vertex_view_space);
                vec3 V = dFdy(vertex_view_space);
                N = normalize(cross(U,V));
            }
        }
    }

    glup_FragColor = geolio_shading(
        color, tex_coord, N, int(primitive_id), mesh_tex_coord
    );
    glup_alpha_discard();
}
)GLSL";

        /**
         * @brief Tests whether two lights shade the same way.
         */
        bool same_light(const SceneLight& a, const SceneLight& b) {
            return
                a.color.x == b.color.x &&
                a.color.y == b.color.y &&
                a.color.z == b.color.z &&
                a.intensity == b.intensity &&
                a.ambient == b.ambient &&
                a.specular_exponent == b.specular_exponent;
        }

        /**
         * @brief The source of the program of a primitive, as
         *        glupCompileProgram() takes it.
         * @details GLUP's own vertex shader is imported (it feeds the view space
         *          position, the normals, the colors and the texture coordinates
         *          the fragment shader reads), so that only the shading of the
         *          fragments belongs to GeoBox. The primitive is the one that is
         *          going to be drawn: GLUP derives the wireframe of the cells from
         *          it, and compiles the same shader for triangles, quads and
         *          polygons, so the program of GLUP_TRIANGLES draws all the
         *          surfaces.
         */
        const std::string& program_source_string(GLUPprimitive primitive) {
            static std::map<GLUPprimitive, std::string> sources;
            std::string& source = sources[primitive];
            if (source.empty()) {
                source =
                    std::string("//primitive ") +
                    glup_primitive_name(primitive) + "\n"
                    "//stage GL_VERTEX_SHADER\n" + VERTEX_SHADER +
                    "//stage GL_FRAGMENT_SHADER\n" + FRAGMENT_SHADER;
            }
            return source;
        }
    }

    bool LightShader::supported(
        ) {
        // glupCompileProgram() takes the shaders of the current profile: the one
        // GeoBox's own programs are written for. On the profiles that draw quads
        // and cells with geometry shaders, GLUP keeps its own programs, and the
        // light keeps the parameters of its shaders.
        return std::strcmp(glupCurrentProfileName(), "GLUPES2") == 0;
    }

    LightShader::Program& LightShader::program(
        GLUPprimitive primitive
        ) {
        // Primitives are the values of an enum that starts at 0, one after the
        // other (see GLUPprimitive): the array is indexed with them.
        return programs_[static_cast<std::size_t>(primitive)];
    }

    bool LightShader::begin(
        const SceneLight& light,
        GLUPprimitive primitive
        ) {
        if (!supported())
            return false;

        Program& shader = program(primitive);
        if (!shader.tried && !compile(shader, primitive))
            return false;

        if (!shader.uploaded || !same_light(shader.uploaded_light, light))
            upload(shader, light);

        glupUseProgram(shader.id);
        return true;
    }

    void LightShader::end(
        ) {
        glupUseProgram(0);
    }

    void LightShader::release(
        ) {
        for (Program& shader : programs_) {
            if (shader.id != 0)
                glDeleteProgram(shader.id);
            shader = Program();
        }
    }

    const char* LightShader::program_source(
        GLUPprimitive primitive
        ) {
        return program_source_string(primitive).c_str();
    }

    bool LightShader::compile(
        Program& program,
        GLUPprimitive primitive
        ) {
        program.tried = true;

        // The program is built here rather than with glupCompileProgram(): that
        // one binds the vertex attributes of GLUP but not the vertex id, which the
        // vertex shader reads to build the texture coordinates of the wireframe of
        // the cells (see GLUPES/vertex_shader.h). Without it, the wireframe covers
        // the whole cell. glupCompileShader() is what compiles the stages of GLUP's
        // own programs: it prepends the preamble of the profile, resolves the
        // //import directives and defines the primitive and the toggles.
        const GLuint vertex_shader = glupCompileShader(
            GL_VERTEX_SHADER, primitive, VERTEX_SHADER);
        const GLuint fragment_shader = glupCompileShader(
            GL_FRAGMENT_SHADER, primitive, FRAGMENT_SHADER);
        if (vertex_shader == 0 || fragment_shader == 0) {
            if (vertex_shader != 0)
                glDeleteShader(vertex_shader);
            if (fragment_shader != 0)
                glDeleteShader(fragment_shader);
            LOG::WARN(
                "Could not compile the program that shades with the light of "
                "the scene: the geometry keeps the light of GLUP.");
            return false;
        }

        const GLuint id = glCreateProgram();
        glAttachShader(id, vertex_shader);
        glAttachShader(id, fragment_shader);
        glBindAttribLocation(id, ATTRIBUTE_VERTEX, "vertex_in");
        glBindAttribLocation(id, ATTRIBUTE_COLOR, "color_in");
        glBindAttribLocation(id, ATTRIBUTE_TEX_COORD, "tex_coord_in");
        glBindAttribLocation(id, ATTRIBUTE_NORMAL, "normal_in");
        glBindAttribLocation(id, ATTRIBUTE_VERTEX_ID, "vertex_id_in");
        GEO::GLSL::link_program(id);
        glDeleteShader(vertex_shader);
        glDeleteShader(fragment_shader);
        program.id = id;

        glUseProgram(program.id);
        // GLUP's own programs have these bound when they are created (see
        // Context::set_primitive_info()); a client program has to set them
        // itself, else the colormap and the image textures would both be read
        // from the same texture unit.
        glUniform1i(
            glGetUniformLocation(program.id, "texture1Dsampler"),
            GLUP_TEXTURE_1D_UNIT);
        glUniform1i(
            glGetUniformLocation(program.id, "texture2Dsampler"),
            GLUP_TEXTURE_2D_UNIT);
        glUniform1i(
            glGetUniformLocation(program.id, "texture3Dsampler"),
            GLUP_TEXTURE_3D_UNIT);
        glUseProgram(0);

        program.light_color_location =
            glGetUniformLocation(program.id, "geolio_light_color");
        program.light_intensity_location =
            glGetUniformLocation(program.id, "geolio_light_intensity");
        program.light_ambient_location =
            glGetUniformLocation(program.id, "geolio_light_ambient");
        program.specular_exponent_location =
            glGetUniformLocation(program.id, "geolio_specular_exponent");
        program.uploaded = false;

        return true;
    }

    void LightShader::upload(
        Program& program,
        const SceneLight& light
        ) {
        // The uniform values of a program are set while it is bound; GLUP binds
        // the program it draws with itself, on every draw (see
        // Context::use_program()), so binding it here is harmless.
        glUseProgram(program.id);
        glUniform3f(
            program.light_color_location,
            light.color.x, light.color.y, light.color.z);
        glUniform1f(program.light_intensity_location, light.intensity);
        glUniform1f(program.light_ambient_location, light.ambient);
        glUniform1f(
            program.specular_exponent_location,
            std::max(light.specular_exponent, 1.0f));
        glUseProgram(0);
        program.uploaded_light = light;
        program.uploaded = true;
    }

    LightShader& light_shader(
        ) {
        static LightShader shader;
        return shader;
    }
}
