//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/11/2.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_SURFACE_BORDERS_H
#define GEOLIO_SURFACE_BORDERS_H

#include <geogram/mesh/mesh.h>
#include <geogram/basic/geometry.h>
#include <geogram_gfx/basic/GL.h>

namespace geolio::geobox
{
    /**
     * @brief The border edges of a mesh surface, enumerated when the mesh
     *        changes and drawn as thick lines from vertex buffers.
     * @details A border edge is a facet edge without an adjacent facet, i.e.
     *          an edge of the boundary of the surface. GeoGram draws them with
     *          MeshGfx::draw_surface_borders(), which looks for them on *every*
     *          frame: it walks all the facet corners of the mesh and emits two
     *          immediate-mode vertices per border edge. Both halves of that
     *          cost grow with the mesh and are paid again on each frame.
     *
     *          This class keeps the result instead. The edges are enumerated
     *          by update(), which does nothing until invalidate() says the mesh
     *          changed; the positions are uploaded to vertex buffer objects the
     *          first time draw() needs them; and each frame after that binds
     *          them and issues one draw call.
     *
     *          The lines are widened by a small program of its own rather than
     *          by GLUP (see the shaders in surface_borders.cpp), for two
     *          reasons. GLUP widens its lines by generating the quads on the
     *          CPU, which costs about 50 ns per border edge per frame: a
     *          surface with a million border edges then takes 50 ms per frame.
     *          And the GLUP profiles used on Apple platforms and on the drivers
     *          that are not NVIDIA draw *everything* in immediate mode, so they
     *          cannot draw a vertex buffer at all. Drawing from the buffer
     *          needs no more than the matrices and the clip plane of GLUP,
     *          which it exposes (see glupGetMatrixdv() and glupGetClipPlane());
     *          the fallback of draw() keeps GLUP's own pass for the contexts
     *          where the program cannot be built.
     *
     *          Each border edge is one instance of a four-vertex triangle strip,
     *          which its vertex program widens to a quad in device coordinates:
     *          instancing rather than a geometry program, since a geometry
     *          program costs about 0.15 ms per draw call on the GL
     *          implementation of Apple platforms, whatever the number of edges.
     *
     *          The edges are drawn where the vertices of the mesh are: a mesh
     *          whose vertices carry their destination as well (see
     *          MeshGfx::get_animate()) has its borders drawn on the source
     *          positions, not on the morphed ones.
     *
     *          The GL objects are created lazily, so an instance can be
     *          invalidated and updated without an OpenGL context (as the unit
     *          tests do).
     */
    class SurfaceBorders {
    public:
        SurfaceBorders() = default;

        /**
         * @brief Releases the buffers and the program the drawing created.
         * @details Called by the destructor. The objects of an application
         *          outlive its OpenGL context (see
         *          GeoBoxApplication::GL_terminate()), so nothing is deleted
         *          once the context is gone: what it created died with it.
         */
        ~SurfaceBorders();

        SurfaceBorders(const SurfaceBorders&) = delete;
        SurfaceBorders& operator=(const SurfaceBorders&) = delete;

        /**
         * @brief Forgets the edges and the buffers: the mesh changed.
         * @details The next update() enumerates the edges again, and the next
         *          draw() uploads them again.
         */
        void invalidate();

        /**
         * @brief Number of border edges found by the last update().
         * @return The count, 0 when the mesh has no border edge or before
         *         update() ran.
         */
        [[nodiscard]] GEO::index_t edge_count() const {
            return GEO::index_t(edges_.size() / 2);
        }

        /**
         * @brief Enumerates the border edges of \p mesh, unless they are
         *        cached already.
         * @details Walks every facet corner, which is the cost this class
         *          exists to pay only when the mesh changes. The criterion is
         *          the one MeshGfx::draw_surface_borders() uses: the edge that
         *          leads from a facet corner to the next corner around its
         *          facet is on the border when that corner has no adjacent
         *          facet. The edges are stored as the ids of their two
         *          vertices in \p mesh, so that the positions are read from the
         *          mesh itself when they are uploaded.
         * @param[in] mesh Mesh to enumerate the border edges of.
         */
        void update(const GEO::Mesh& mesh);

        /**
         * @brief Draws the border edges of \p mesh.
         * @details Enumerates the edges if needed (see update()), fills the
         *          buffers the first time they are drawn, then draws them with
         *          one draw call (see draw_from_program()). The lighting, the
         *          color and the mesh width of GLUP are left untouched: the
         *          program of the pass takes the color and the width from its
         *          own uniforms.
         * @param[in] mesh Mesh the edges were enumerated from; its vertices
         *            give the positions of the edge ends.
         * @param[in] color Color of the lines.
         * @param[in] width Width of the lines, in tenths of pixels, i.e. the
         *            unit of MeshGfx::set_mesh_width() (the GUI shows it divided
         *            by ten).
         */
        void draw(const GEO::Mesh& mesh, const GEO::vec4f& color, float width);

    private:
        /**
         * @brief Draws the cached edges from the vertex buffer.
         * @details Uploads the buffer first if the mesh changed, and builds the
         *          program of the pass the first time it is needed.
         * @param[in] color Color of the lines.
         * @param[in] width Width of the lines, in pixels.
         * @return false when the pass cannot draw from the buffer (the program
         *         or the buffer could not be created, or the GL context cannot
         *         draw instances), in which case draw() falls back to
         *         draw_immediate().
         */
        bool draw_from_program(const GEO::vec4f& color, float width);

        /**
         * @brief Draws the cached edges with the immediate-mode pass of GLUP.
         * @details The fallback of draw_from_program(), for the contexts where
         *          the program of the pass cannot be built (no OpenGL context,
         *          or a GLSL implementation without geometry shaders). It is
         *          the pass GeoGram draws the borders with: as slow as it is,
         *          it still uses the cached edges, so the mesh is not walked
         *          again on every frame.
         * @param[in] mesh Mesh the vertices of the cached edges belong to.
         * @param[in] color Color of the lines.
         * @param[in] width Width of the lines, in tenths of pixels.
         */
        void draw_immediate(
            const GEO::Mesh& mesh, const GEO::vec4f& color, float width) const;

        /**
         * @brief Uploads the cached border edges to the vertex buffer.
         * @details One pair of positions per edge, which the drawing walks as
         *          instances (see draw_from_program()). The positions are
         *          dropped once uploaded, the GPU copy being what the frames
         *          read.
         * @param[in] mesh Mesh the vertices of the cached edges belong to.
         * @return false when the GL objects could not be created, in which case
         *         the class stops trying (see buffers_unavailable_).
         */
        bool upload(const GEO::Mesh& mesh);

        /**
         * @brief Builds the program that draws the lines, if not built yet.
         * @return false when the program could not be built, in which case the
         *         class stops trying (see program_unavailable_).
         */
        bool build_program();

        /**
         * @brief Deletes the GL objects, if they were created.
         */
        void release_gl_resources();

        /**
         * @brief Mesh vertex ids of the border edges, two per edge.
         * @details Only the ids are kept: the positions are read from the mesh
         *          when the buffer is filled, which keeps the cache small and
         *          free of the precision of the vertices.
         */
        GEO::vector<GEO::index_t> edges_;

        /** Whether the mesh changed since edges_ was enumerated. */
        bool edges_dirty_ = true;

        /** Whether the buffers have to be filled from edges_ again. */
        bool buffers_dirty_ = true;

        /**
         * @brief Whether the buffers could not be created, so that draw() does
         *        not retry on every frame.
         */
        bool buffers_unavailable_ = false;

        /** Vertex array object of the border edge lines. */
        GLuint vertex_array_ = 0;

        /** The two ends of each border edge, one instance per edge. */
        GLuint edge_buffer_ = 0;

        /** Number of edges to draw, i.e. number of instances. */
        GLsizei instance_count_ = 0;

        /** Program that widens and colors the lines (see build_program()). */
        GLuint program_ = 0;

        /**
         * @brief Whether the program could not be built, so that draw() does
         *        not retry on every frame.
         */
        bool program_unavailable_ = false;

        /**
         * @brief Locations of the uniforms of the program.
         * @details Looked up when the program is built; -1 when the compiler
         *          optimized a uniform out, which the setters ignore.
         */
        struct UniformLocations {
            GLint modelviewprojection = -1;
            GLint modelview = -1;
            GLint viewport = -1;
            GLint width = -1;
            GLint depth_bias = -1;
            GLint color = -1;
            GLint clip_plane = -1;
            GLint clip_enabled = -1;
        };
        UniformLocations uniforms_;
    };
}

#endif //GEOLIO_SURFACE_BORDERS_H
