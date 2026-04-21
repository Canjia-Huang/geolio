//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/4/21.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOGRAMMESHUTILS_TEST_TRI_OPERATIONS_H
#define GEOGRAMMESHUTILS_TEST_TRI_OPERATIONS_H

#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_io.h>
#include <gtest/gtest.h>
#include "utils.h"

namespace GEO::MeshUtils::Test
{
    /**
     * Build the reference triangular test mesh used by the triangle operator tests.
     *
     * The mesh is a 4x4 vertex grid with two elevated interior vertices and a
     * 3x3 set of triangle pairs, then connected to populate adjacency.
     *
     * @param[in,out] M The mesh to clear and rebuild.
     */
    inline void build_tri_mesh(
        GEO::Mesh& M
        ) {
        M.clear();

        GEO::index_t new_v = M.vertices.create_vertices(4*4);
        for (GEO::index_t i = 0; i < 4; ++i) {
            for (GEO::index_t j = 0; j < 4; ++j)
                M.vertices.point(new_v++) = GEO::vec3(j, i, 0);
        }
        M.vertices.point(5).z += 0.5;
        M.vertices.point(10).z += 0.5;

        GEO::index_t new_f = M.facets.create_triangles(3*3*2);
        for (GEO::index_t i = 0; i < 3; ++i) {
            for (GEO::index_t j = 0; j < 3; ++j) {
                const GEO::index_t v0 = 4*i+j;
                const GEO::index_t v1 = v0+1;
                const GEO::index_t v2 = v1+4;
                const GEO::index_t v3 = v0+4;
                M.facets.set_vertex(new_f, 0, v0);
                M.facets.set_vertex(new_f, 1, v1);
                M.facets.set_vertex(new_f, 2, v3);
                M.facets.set_vertex(new_f + 1, 0, v2);
                M.facets.set_vertex(new_f + 1, 1, v3);
                M.facets.set_vertex(new_f + 1, 2, v1);
                new_f += 2;
            }
        }

        M.facets.connect();
    }

    enum TRIANGLE_MESH_TEST_PARAMS_TYPE {
         INTERIOR_VERTICES_F_LV,
         BORDER_VERTICES_F_LV,
         INTERIOR_EDGES_F_LV,
         BORDER_EDGES_F_LV
     };

    /**
     * Collect test (facet, local-vertex) pairs from the reference triangle mesh.
     *
     * The selected category determines whether interior/border vertices or
     * interior/border edges are returned as (facet, local-vertex) pairs.
     *
     * @param[in] type The category of test parameters to generate.
     * @return A vector of (facet, local-vertex) pairs matching @p type.
     */
    const auto TRIANGLE_MESH_GET_TEST_PARAMS = [](
        const TRIANGLE_MESH_TEST_PARAMS_TYPE type
        ) {
         GEO::initialize(GEO::GEOGRAM_INSTALL_ALL);

         GEO::Mesh M;
         build_tri_mesh(M);

        /* Find all border vertices */
        std::vector<bool> M_v_border(M.vertices.nb(), false);
        for (const auto& f : M.facets) {
            for (GEO::index_t lv = 0; lv < 3; ++lv) {
                if (M.facets.adjacent(f, lv) == GEO::NO_FACET) {
                    M_v_border[M.facets.vertex(f, lv)] = true;
                    M_v_border[M.facets.vertex(f, (lv+1)%3)] = true;
                }
            }
        }

        std::vector<std::pair<GEO::index_t, GEO::index_t>> output;
        switch (type) {
            case INTERIOR_VERTICES_F_LV:
                for (const auto& f : M.facets) {
                    for (GEO::index_t lv = 0; lv < 3; ++lv) {
                        if (!M_v_border[M.facets.vertex(f, lv)])
                            output.emplace_back(f, lv);
                    }
                }
                break;
            case BORDER_VERTICES_F_LV:
                for (const auto& f : M.facets) {
                    for (GEO::index_t lv = 0; lv < 3; ++lv) {
                        if (M_v_border[M.facets.vertex(f, lv)])
                            output.emplace_back(f, lv);
                    }
                }
                break;
            case INTERIOR_EDGES_F_LV:
                for (const auto& f : M.facets) {
                    for (GEO::index_t lv = 0; lv < 3; ++lv) {
                        if (M.facets.adjacent(f, lv) != GEO::NO_FACET)
                            output.emplace_back(f, lv);
                    }
                }
                break;
            case BORDER_EDGES_F_LV:
                for (const auto& f : M.facets) {
                    for (GEO::index_t lv = 0; lv < 3; ++lv) {
                        if (M.facets.adjacent(f, lv) == GEO::NO_FACET)
                            output.emplace_back(f, lv);
                    }
                }
                break;
            default:
                throw std::runtime_error("Invalid type!");
        }

        return output;
     };

    class TriangleOperationsTest : public ::testing::TestWithParam<std::pair<GEO::index_t, GEO::index_t>> {
        /**
         * Build the shared reference mesh before each test case runs.
         */
        void SetUp() override {
            build_tri_mesh(M);
            M_c_affected.bind(M.cells.attributes(), "affected");
        }

    protected:
        /**
         * Verify that reconnecting the mesh preserves the current adjacency layout.
         */
        void check_connections() {
            std::vector<GEO::index_t> current_connections(3*M.facets.nb(), GEO::NO_FACET);
            for (const auto& f : M.facets) {
                for (GEO::index_t lv = 0; lv < 3; ++lv)
                    current_connections[3*f+lv] = M.facets.adjacent(f, lv);
            }

            M.facets.connect();
            for (const auto& f : M.facets) {
                for (GEO::index_t lv = 0; lv < 3; ++lv)
                    EXPECT_EQ(current_connections[3*f+lv], M.facets.adjacent(f, lv));
            }

            /* Rollback adjacency */
            for (const auto& f : M.facets) {
                for (GEO::index_t lv = 0; lv < 3; ++lv)
                    M.facets.set_adjacent(f, lv, current_connections[3*f+lv]);
            }
        }

        /**
         * Save the current mesh result to a file named by the active test case.
         *
         * @param[in] f  Facet index used to identify the test result.
         * @param[in] lv Local vertex index used to identify the test result.
         */
        void save_results(
            const GEO::index_t f,
            const GEO::index_t lv
            ) const {
            EXPECT_TRUE(GEO::mesh_save(M, get_current_test_name() +
                                            "_f" + std::to_string(f) +
                                            "_lv" + std::to_string(lv) +
                                            ".geogram"));
        }

        GEO::Mesh M;
        GEO::Attribute<GEO::index_t> M_c_affected;
    };
}

#endif //GEOGRAMMESHUTILS_TEST_TRI_OPERATIONS_H