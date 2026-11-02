//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/11/2.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "surface_borders.h"

#include <vector>
#include <geogram_gfx/basic/GL.h>
#include <geogram_gfx/basic/GLSL.h>
#include <geogram_gfx/GLUP/GLUP.h>
#include <geogram_gfx/GLUP/GLUP_private.h>

namespace geolio::geobox
{
    namespace
    {
        /**
         * @brief Attribute the program of the pass reads the first end of the
         *        edge from.
         * @details The pass uses a vertex array of its own, so the locations are
         *          bound to the program rather than read back from it. The
         *          second end follows at the next location.
         */
        constexpr GLuint EDGE_START_ATTRIBUTE = 0;
        constexpr GLuint EDGE_END_ATTRIBUTE = 1;

        /**
         * @brief Number of coordinates of a position in the edge buffer.
         */
        constexpr GLint POSITION_DIMENSION = 3;

        /**
         * @brief Number of vertices of the strip one border edge is drawn with.
         * @details The two triangles of the quad around the edge, as a triangle
         *          strip (see the vertex program).
         */
        constexpr GLsizei STRIP_VERTEX_COUNT = 4;

        /**
         * @brief Vertex program of the border lines.
         * @details One instance per border edge: the four vertices of the strip
         *          are the four corners of the quad that widens the edge, which
         *          the program computes from the two ends it is given.
         *
         *          The construction is the one of GLUP's thick lines (see its
         *          thick_lines_geometry_shader.h): the two ends are taken to
         *          normalized device coordinates, where the segment is offset of
         *          half its width on both sides and extended of a quarter at
         *          both ends, which rounds the joints between consecutive edges,
         *          then pulled slightly toward the viewer so that the lines do
         *          not fight with the surface they lie on. The job is done here
         *          rather than in a geometry program because the GL
         *          implementation of Apple platforms charges about 0.15 ms per
         *          draw call for one, whatever the number of edges.
         *
         *          A vertex behind the eye has a negative w, and the division
         *          that brings the segment to device coordinates would then
         *          mirror it: the segment is clipped against the near plane
         *          first, so that such an edge is shortened instead of drawn
         *          across the screen.
         */
        constexpr const char* VERTEX_SHADER_SOURCE = R"GLSL(#version 150
in vec3 edge_start;
in vec3 edge_end;

uniform mat4 modelviewprojection;
uniform mat4 modelview;
uniform vec4 clip_plane;
uniform vec2 viewport;
uniform float width;
uniform float depth_bias;

out float clip_distance;

void main() {
    vec4 c1 = modelviewprojection * vec4(edge_start, 1.0);
    vec4 c2 = modelviewprojection * vec4(edge_end, 1.0);

    // The clip plane of GLUP lives in the space of the modelview matrix: a
    // vertex is clipped where this distance is negative.
    float d1 = dot(modelview * vec4(edge_start, 1.0), clip_plane);
    float d2 = dot(modelview * vec4(edge_end, 1.0), clip_plane);

    const float NEAR_EPSILON = 1e-6;
    if(c1.w < NEAR_EPSILON && c2.w < NEAR_EPSILON) {
        // Both ends behind the eye: a degenerate quad outside of the clip
        // volume, which draws nothing.
        gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
        clip_distance = 1.0;
        return;
    }
    if(c1.w < NEAR_EPSILON) {
        float t = (NEAR_EPSILON - c1.w) / (c2.w - c1.w);
        c1 = mix(c1, c2, t);
        d1 = mix(d1, d2, t);
    } else if(c2.w < NEAR_EPSILON) {
        float t = (NEAR_EPSILON - c2.w) / (c1.w - c2.w);
        c2 = mix(c2, c1, t);
        d2 = mix(d2, d1, t);
    }

    vec2 p1 = c1.xy / c1.w;
    vec2 p2 = c2.xy / c2.w;

    // Half of the width, in device coordinates (see GLUP's thick lines).
    float R = width / (0.5 * (viewport.x + viewport.y));
    vec2 segment = p2 - p1;
    float segment_length = length(segment);
    vec2 U = R * ((segment_length > 1e-9) ? segment / segment_length : vec2(1.0, 0.0));
    vec2 V = vec2(U.y, -U.x);

    // Corners 0 and 1 sit at the first end of the edge, 2 and 3 at the second,
    // which is the order of the triangle strip.
    int corner = gl_VertexID % 4;
    vec4 end = (corner < 2) ? c1 : c2;
    vec2 offset = ((corner < 2) ? -U : U) + (((corner % 2) == 0) ? -V : V);

    gl_Position = vec4(
        end.xy / end.w + offset,
        end.z / end.w - depth_bias * R,
        1.0
    );
    clip_distance = (corner < 2) ? d1 : d2;
}
)GLSL";

        /**
         * @brief Fragment program of the border lines.
         * @details Flat color, clipped where GLUP would clip it (see the vertex
         *          program) and blended as the pass that called it asked.
         */
        constexpr const char* FRAGMENT_SHADER_SOURCE = R"GLSL(#version 150
in float clip_distance;

uniform vec4 color;
uniform bool clip_enabled;

out vec4 frag_color;

void main() {
    if(clip_enabled && clip_distance < 0.0) {
        discard;
    }
    frag_color = color;
}
)GLSL";

        /**
         * @brief Deletes an OpenGL buffer object, if it was created.
         */
        void delete_buffer(GLuint& buffer) {
            if (buffer != 0) {
                glDeleteBuffers(1, &buffer);
                buffer = 0;
            }
        }

        /**
         * @brief Writes the product of two 4x4 matrices in column-major order.
         * @details The matrices are the ones GLUP holds, which are in the
         *          layout OpenGL expects.
         * @param[out] result Matrix that receives the product.
         * @param[in] a Left matrix.
         * @param[in] b Right matrix.
         */
        void multiply_matrices(
            GLfloat* result, const GLdouble* a, const GLdouble* b) {
            for (GLint column = 0; column < 4; ++column) {
                for (GLint row = 0; row < 4; ++row) {
                    GLdouble sum = 0.0;
                    for (GLint k = 0; k < 4; ++k)
                        sum += a[k * 4 + row] * b[column * 4 + k];
                    result[column * 4 + row] = GLfloat(sum);
                }
            }
        }

        /**
         * @brief Sends a uniform, unless the compiler optimized it out.
         */
        void set_uniform1f(const GLint location, const GLfloat x) {
            if (location >= 0)
                glUniform1f(location, x);
        }

        void set_uniform2f(const GLint location, const GLfloat x, const GLfloat y) {
            if (location >= 0)
                glUniform2f(location, x, y);
        }

        void set_uniform1i(const GLint location, const GLint x) {
            if (location >= 0)
                glUniform1i(location, x);
        }

        void set_uniform4fv(const GLint location, const GLfloat* xyzw) {
            if (location >= 0)
                glUniform4fv(location, 1, xyzw);
        }

        void set_uniform_matrix(const GLint location, const GLfloat* matrix) {
            if (location >= 0)
                glUniformMatrix4fv(location, 1, GL_FALSE, matrix);
        }
    }

    SurfaceBorders::~SurfaceBorders() {
        release_gl_resources();
    }

    void SurfaceBorders::release_gl_resources() {
        // The objects of an application outlive its OpenGL context (see
        // GeoBoxApplication::GL_terminate()): the ids the context created are
        // meaningless once it is gone, and deleting them would reach a driver
        // that has no context current.
        if (glupCurrentContext() != nullptr) {
            if (vertex_array_ != 0)
                glupDeleteVertexArrays(1, &vertex_array_);
            delete_buffer(edge_buffer_);
            if (program_ != 0)
                glDeleteProgram(program_);
        }
        vertex_array_ = 0;
        edge_buffer_ = 0;
        program_ = 0;
        instance_count_ = 0;
        buffers_dirty_ = true;
    }

    void SurfaceBorders::invalidate() {
        edges_.clear();
        edges_dirty_ = true;
        buffers_dirty_ = true;
    }

    void SurfaceBorders::update(
        const GEO::Mesh& mesh
        ) {
        if (!edges_dirty_)
            return;
        edges_dirty_ = false;
        edges_.clear();

        // Same criterion as MeshGfx::draw_surface_borders(): the edge that goes
        // from a facet corner to the next corner around the facet has no
        // adjacent facet.
        for (const GEO::index_t f : mesh.facets) {
            for (const GEO::index_t c : mesh.facets.corners(f)) {
                if (mesh.facet_corners.adjacent_facet(c) != GEO::NO_FACET)
                    continue;
                edges_.push_back(mesh.facet_corners.vertex(c));
                edges_.push_back(mesh.facet_corners.vertex(
                    mesh.facets.next_corner_around_facet(f, c)));
            }
        }

        buffers_dirty_ = true;
    }

    bool SurfaceBorders::upload(
        const GEO::Mesh& mesh
        ) {
        // No mesh change is pending any more: whatever the outcome below, the
        // buffer is as up to date as it can be.
        buffers_dirty_ = false;

        // One instance per edge, holding both of its ends: an edge of the
        // border shares its vertices with the edges around it, but each edge is
        // widened on its own, so its ends are not gathered.
        std::vector<GLfloat> ends;
        ends.reserve(POSITION_DIMENSION * edges_.size());

        // The z coordinate of a 2-D mesh does not exist in the vertex store
        // (see MeshObject::get_vertex_position()).
        const GEO::coord_index_t dimension =
            mesh.vertices.dimension() < POSITION_DIMENSION
                ? mesh.vertices.dimension()
                : GEO::coord_index_t(POSITION_DIMENSION);

        for (GEO::index_t e = 0; e < edges_.size(); e += 2) {
            const GEO::index_t v1 = edges_[e];
            const GEO::index_t v2 = edges_[e + 1];
            // A mesh that was modified without being reported (see
            // MeshObject::mesh_changed()) can have edges that reference
            // vertices it does not have: they are left out rather than read
            // outside of the vertex store.
            if (v1 >= mesh.vertices.nb() || v2 >= mesh.vertices.nb())
                continue;
            for (const GEO::index_t v : {v1, v2}) {
                const double* p = mesh.vertices.point_ptr(v);
                for (GEO::coord_index_t c = 0; c < POSITION_DIMENSION; ++c)
                    ends.push_back(c < dimension ? GLfloat(p[c]) : 0.0f);
            }
        }

        if (vertex_array_ == 0)
            glupGenVertexArrays(1, &vertex_array_);

        if (vertex_array_ == 0) {
            // Without an OpenGL context the objects are created as id 0: there
            // is nothing to draw them with, and no point in retrying on the
            // next frame either (see draw()).
            buffers_unavailable_ = true;
            return false;
        }

        glupBindVertexArray(vertex_array_);

        // The vertex array records the buffer and the layout of the attributes
        // while it is bound, which is why they are set up with it bound.
        GEO::update_buffer_object(
            edge_buffer_, GL_ARRAY_BUFFER,
            ends.size() * sizeof(GLfloat), ends.data());

        const GLsizei stride =
            2 * POSITION_DIMENSION * GLsizei(sizeof(GLfloat));
        glEnableVertexAttribArray(EDGE_START_ATTRIBUTE);
        glVertexAttribPointer(
            EDGE_START_ATTRIBUTE,
            POSITION_DIMENSION,
            GL_FLOAT,
            GL_FALSE,
            stride,
            nullptr);
        glEnableVertexAttribArray(EDGE_END_ATTRIBUTE);
        glVertexAttribPointer(
            EDGE_END_ATTRIBUTE,
            POSITION_DIMENSION,
            GL_FLOAT,
            GL_FALSE,
            stride,
            reinterpret_cast<const GLvoid*>(
                POSITION_DIMENSION * sizeof(GLfloat)));

        // Both attributes describe one edge, not one vertex: they advance once
        // per instance.
        if (glVertexAttribDivisor != nullptr) {
            glVertexAttribDivisor(EDGE_START_ATTRIBUTE, 1);
            glVertexAttribDivisor(EDGE_END_ATTRIBUTE, 1);
        }

        glupBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        if (edge_buffer_ == 0) {
            buffers_unavailable_ = true;
            return false;
        }

        instance_count_ = GLsizei(ends.size() / (2 * POSITION_DIMENSION));
        return true;
    }

    bool SurfaceBorders::build_program(
        ) {
        if (program_ != 0)
            return true;
        if (program_unavailable_)
            return false;

        GLuint program = 0;
        try {
            const char* vertex_source[] = {VERTEX_SHADER_SOURCE};
            const char* fragment_source[] = {FRAGMENT_SHADER_SOURCE};
            const GLuint vertex_shader = GEO::GLSL::compile_shader(
                GL_VERTEX_SHADER, vertex_source, 1);
            const GLuint fragment_shader = GEO::GLSL::compile_shader(
                GL_FRAGMENT_SHADER, fragment_source, 1);
            program = GEO::GLSL::create_program_from_shaders_no_link(
                vertex_shader, fragment_shader, 0);
            glBindAttribLocation(program, EDGE_START_ATTRIBUTE, "edge_start");
            glBindAttribLocation(program, EDGE_END_ATTRIBUTE, "edge_end");
            GEO::GLSL::link_program(program);
        }
        catch (const GEO::GLSL::GLSLCompileError&) {
            // A GLSL implementation that cannot compile the program (a driver
            // too old for GLSL 1.50, or an OpenGL ES one): the borders keep
            // being drawn by the immediate-mode pass (see draw_immediate()).
            if (program != 0)
                glDeleteProgram(program);
            program_unavailable_ = true;
            return false;
        }

        program_ = program;
        uniforms_.modelviewprojection = glGetUniformLocation(
            program_, "modelviewprojection");
        uniforms_.modelview = glGetUniformLocation(program_, "modelview");
        uniforms_.viewport = glGetUniformLocation(program_, "viewport");
        uniforms_.width = glGetUniformLocation(program_, "width");
        uniforms_.depth_bias = glGetUniformLocation(program_, "depth_bias");
        uniforms_.color = glGetUniformLocation(program_, "color");
        uniforms_.clip_plane = glGetUniformLocation(program_, "clip_plane");
        uniforms_.clip_enabled = glGetUniformLocation(program_, "clip_enabled");
        return true;
    }

    bool SurfaceBorders::draw_from_program(
        const GEO::vec4f& color,
        const float width
        ) {
        if (instance_count_ == 0 || !build_program())
            return false;
        // Drawing one instance per edge needs the instanced draw call and the
        // attribute divisors, which a GL context older than 3.1 does not have.
        if (glDrawArraysInstanced == nullptr || glVertexAttribDivisor == nullptr)
            return false;
        // GLUP's own pass gives the fragments the identifier of the primitive
        // they come from, which is how an application picks the elements of a
        // mesh (see MeshGfx::set_picking_mode()). This program does not, so a
        // picking frame keeps drawing the borders with it.
        if (glupIsEnabled(GLUP_PICKING) != 0)
            return false;

        // The program of the pass needs the matrices, the viewport and the
        // clip plane of GLUP: they are the ones the passes before it drew the
        // surface with, so the borders land exactly where it is.
        GLdouble projection[16];
        GLdouble modelview[16];
        glupGetMatrixdv(GLUP_PROJECTION_MATRIX, projection);
        glupGetMatrixdv(GLUP_MODELVIEW_MATRIX, modelview);
        GLfloat modelviewprojection[16];
        multiply_matrices(modelviewprojection, projection, modelview);
        GLfloat modelview_f[16];
        for (int i = 0; i < 16; ++i)
            modelview_f[i] = GLfloat(modelview[i]);

        GLint viewport[4] = {0, 0, 1, 1};
        glGetIntegerv(GL_VIEWPORT, viewport);

        const bool clip_enabled = glupIsEnabled(GLUP_CLIPPING) != 0;
        GLdouble clip_plane[4] = {0.0, 0.0, 0.0, 0.0};
        if (clip_enabled)
            glupGetClipPlane(clip_plane);
        const GLfloat clip_plane_f[4] = {
            GLfloat(clip_plane[0]), GLfloat(clip_plane[1]),
            GLfloat(clip_plane[2]), GLfloat(clip_plane[3])
        };

        // The pass borrows the program binding, and GLUP remembers the one it
        // bound: it is put back as it was found, so that the passes that follow
        // keep drawing with what they asked for (see MeshObject::draw_scene()).
        GLint previous_program = 0;
        glGetIntegerv(GL_CURRENT_PROGRAM, &previous_program);

        glUseProgram(program_);
        set_uniform_matrix(uniforms_.modelviewprojection, modelviewprojection);
        set_uniform_matrix(uniforms_.modelview, modelview_f);
        set_uniform2f(
            uniforms_.viewport, GLfloat(viewport[2]), GLfloat(viewport[3]));
        set_uniform1f(uniforms_.width, width);
        // Same depth bias as the thick lines of GLUP: a fraction of the line
        // width, in device coordinates, so that the borders stay in front of the
        // surface they are drawn on.
        set_uniform1f(uniforms_.depth_bias, GLfloat(0.2 * modelview[15]));
        set_uniform4fv(uniforms_.color, color.data());
        set_uniform4fv(uniforms_.clip_plane, clip_plane_f);
        set_uniform1i(uniforms_.clip_enabled, clip_enabled ? 1 : 0);

        glupBindVertexArray(vertex_array_);
        glDrawArraysInstanced(
            GL_TRIANGLE_STRIP, 0, STRIP_VERTEX_COUNT, instance_count_);
        glupBindVertexArray(0);
        glUseProgram(GLuint(previous_program));
        return true;
    }

    void SurfaceBorders::draw_immediate(
        const GEO::Mesh& mesh,
        const GEO::vec4f& color,
        const float width
        ) const {
        // The pass of GeoGram (see MeshGfx::draw_surface_borders()). The color
        // is set on both the front and the mesh colors of GLUP: a line drawn
        // with the thick-line primitive takes the mesh color, and the one drawn
        // thin takes the front color.
        const bool lighting = glupIsEnabled(GLUP_LIGHTING) != 0;
        const bool draw_mesh = glupIsEnabled(GLUP_DRAW_MESH) != 0;
        GLfloat previous_color[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        GLfloat previous_mesh_color[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        glupGetColor4fv(GLUP_FRONT_COLOR, previous_color);
        glupGetColor4fv(GLUP_MESH_COLOR, previous_mesh_color);
        const GLUPint previous_width = glupGetMeshWidth();

        glupDisable(GLUP_LIGHTING);
        glupDisable(GLUP_DRAW_MESH);
        glupSetColor4fv(GLUP_FRONT_COLOR, color.data());
        glupSetColor4fv(GLUP_MESH_COLOR, color.data());
        // The width is in tenths of a pixel, as MeshGfx takes it.
        const GLUPint mesh_width = GLUPint(width * 10.0f);
        glupSetMeshWidth(mesh_width);

        // GLUP draws a line of the mesh width with its thick-line primitive (see
        // convert_primitive() in GLUP.cpp), a conversion that glupBegin()
        // applies and that the private functions used here for speed do not:
        // the primitive is chosen here instead.
        const bool single_precision = mesh.vertices.single_precision();
        const bool has_depth = mesh.vertices.dimension() >= 3;
        glupPrivateBegin(
            mesh_width > 1 ? GLUP_RESERVED_PRIMITIVE_1 : GLUP_LINES);
        for (GEO::index_t e = 0; e < edges_.size(); e += 2) {
            const GEO::index_t v1 = edges_[e];
            const GEO::index_t v2 = edges_[e + 1];
            if (v1 >= mesh.vertices.nb() || v2 >= mesh.vertices.nb())
                continue;
            for (const GEO::index_t v : {v1, v2}) {
                // A 2-D mesh has no z coordinate in its vertex store (see
                // MeshObject::get_vertex_position()).
                if (single_precision) {
                    const float* p =
                        mesh.vertices.single_precision_point_ptr(v);
                    if (has_depth)
                        glupPrivateVertex3fv(p);
                    else
                        glupPrivateVertex2fv(p);
                }
                else {
                    const double* p = mesh.vertices.point_ptr(v);
                    if (has_depth)
                        glupPrivateVertex3dv(p);
                    else
                        glupPrivateVertex2dv(p);
                }
            }
        }
        glupPrivateEnd();

        glupSetColor4fv(GLUP_FRONT_COLOR, previous_color);
        glupSetColor4fv(GLUP_MESH_COLOR, previous_mesh_color);
        glupSetMeshWidth(previous_width);
        if (draw_mesh)
            glupEnable(GLUP_DRAW_MESH);
        if (lighting)
            glupEnable(GLUP_LIGHTING);
    }

    void SurfaceBorders::draw(
        const GEO::Mesh& mesh,
        const GEO::vec4f& color,
        const float width
        ) {
        update(mesh);
        if (edges_.empty())
            return;

        if (buffers_dirty_ && !buffers_unavailable_)
            upload(mesh);

        // The width is in tenths of a pixel, as MeshGfx takes it.
        if (draw_from_program(color, width * 10.0f))
            return;
        draw_immediate(mesh, color, width);
    }
}
