//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/11/2.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <gtest/gtest.h>
#include <geogram/mesh/mesh.h>
#include <geolio/geobox/object/surface_borders.h>

namespace geolio::test
{
    namespace
    {
        /**
         * @brief Builds an open quad grid, nb x nb facets.
         * @details The facets are connected, so the only border edges are the
         *          ones of the four sides of the grid: nb on each side.
         * @param[out] mesh Mesh to fill.
         * @param[in] nb Number of facets per side.
         */
        void make_open_grid(GEO::Mesh& mesh, const GEO::index_t nb) {
            mesh.clear();
            mesh.vertices.set_dimension(3);
            mesh.vertices.create_vertices((nb + 1) * (nb + 1));
            for (GEO::index_t j = 0; j <= nb; ++j) {
                for (GEO::index_t i = 0; i <= nb; ++i) {
                    mesh.vertices.point(j * (nb + 1) + i) = GEO::vec3(
                        double(i), double(j), double(i % 2));
                }
            }
            mesh.facets.create_quads(nb * nb);
            for (GEO::index_t j = 0; j < nb; ++j) {
                for (GEO::index_t i = 0; i < nb; ++i) {
                    const GEO::index_t f = j * nb + i;
                    const GEO::index_t v = j * (nb + 1) + i;
                    mesh.facets.set_vertex(f, 0, v);
                    mesh.facets.set_vertex(f, 1, v + 1);
                    mesh.facets.set_vertex(f, 2, v + nb + 2);
                    mesh.facets.set_vertex(f, 3, v + nb + 1);
                }
            }
            mesh.facets.connect();
        }

        /**
         * @brief Builds the boundary of a tetrahedron.
         * @details Every edge is shared by two facets: the surface has no
         *          border edge at all.
         * @param[out] mesh Mesh to fill.
         */
        void make_closed_surface(GEO::Mesh& mesh) {
            mesh.clear();
            mesh.vertices.set_dimension(3);
            mesh.vertices.create_vertices(4);
            mesh.vertices.point(0) = GEO::vec3(0.0, 0.0, 0.0);
            mesh.vertices.point(1) = GEO::vec3(1.0, 0.0, 0.0);
            mesh.vertices.point(2) = GEO::vec3(0.0, 1.0, 0.0);
            mesh.vertices.point(3) = GEO::vec3(0.0, 0.0, 1.0);
            mesh.facets.create_triangle(0, 2, 1);
            mesh.facets.create_triangle(0, 1, 3);
            mesh.facets.create_triangle(0, 3, 2);
            mesh.facets.create_triangle(1, 2, 3);
            mesh.facets.connect();
        }

        /**
         * @brief Detaches every facet from its neighbours.
         * @details The operation "disconnect facets" of the applications: every
         *          facet corner loses its adjacent facet, so every facet edge
         *          becomes a border edge.
         * @param[in,out] mesh Mesh to detach the facets of.
         */
        void detach_facets(GEO::Mesh& mesh) {
            for (const GEO::index_t c : mesh.facet_corners)
                mesh.facet_corners.set_adjacent_facet(c, GEO::NO_FACET);
        }
    }

    /**
     * @brief The border edges of a connected open surface are the edges of its
     *        boundary.
     */
    TEST(SurfaceBorders, CountsTheEdgesOfTheBoundary) {
        GEO::Mesh grid;
        make_open_grid(grid, 4);

        geobox::SurfaceBorders borders;
        borders.update(grid);

        // Four sides of four edges each, and none of the inner edges.
        EXPECT_EQ(borders.edge_count(), 16u);
    }

    /**
     * @brief A closed surface has no border edge to draw.
     */
    TEST(SurfaceBorders, AClosedSurfaceHasNoBorderEdge) {
        GEO::Mesh surface;
        make_closed_surface(surface);

        geobox::SurfaceBorders borders;
        borders.update(surface);

        EXPECT_EQ(borders.edge_count(), 0u);
    }

    /**
     * @brief Detaching the facets of a closed surface puts every facet edge on
     *        the border.
     */
    TEST(SurfaceBorders, DetachedFacetsHaveAllTheirEdgesOnTheBorder) {
        GEO::Mesh surface;
        make_closed_surface(surface);
        detach_facets(surface);

        geobox::SurfaceBorders borders;
        borders.update(surface);

        // Every one of the three edges of each of the four triangles, each
        // reported by the only facet that has it.
        EXPECT_EQ(borders.edge_count(), 3 * surface.facets.nb());
    }

    /**
     * @brief The edges are kept as long as the mesh is not reported to have
     *        changed.
     * @details This is what makes the pass cheap on large meshes: update() does
     *          no work between two changes of the mesh, however many frames are
     *          drawn, and a change is only taken into account once invalidate()
     *          says so (see MeshObject::mesh_changed()).
     */
    TEST(SurfaceBorders, KeepsTheEdgesUntilItIsInvalidated) {
        GEO::Mesh grid;
        make_open_grid(grid, 3);

        geobox::SurfaceBorders borders;
        borders.update(grid);
        const GEO::index_t open_count = borders.edge_count();
        ASSERT_EQ(open_count, 12u);

        // The mesh changes, but the object is not told: the cache is the one of
        // the previous mesh.
        detach_facets(grid);
        borders.update(grid);
        EXPECT_EQ(borders.edge_count(), open_count);

        // Told about the change, the next update() sees the new edges.
        borders.invalidate();
        borders.update(grid);
        EXPECT_EQ(borders.edge_count(), 4 * grid.facets.nb());
    }

    /**
     * @brief A mesh without facets has no border edge.
     */
    TEST(SurfaceBorders, AMeshWithoutFacetsHasNoBorderEdge) {
        GEO::Mesh points;
        points.vertices.create_vertices(3);

        geobox::SurfaceBorders borders;
        borders.update(points);

        EXPECT_EQ(borders.edge_count(), 0u);
    }
}
