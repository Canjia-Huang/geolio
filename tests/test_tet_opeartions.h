//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/4/21.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOGRAMMESHUTILS_TEST_TET_OPEARTIONS_H
#define GEOGRAMMESHUTILS_TEST_TET_OPEARTIONS_H

#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_io.h>
#include <gtest/gtest.h>
#include "utils.h"
#include "geogram_mesh_utils/pair_hash.h"

namespace GEO::MeshUtils::Test
{
    /**
     * Build the Cartesian reference tetrahedral mesh used by operator tests.
     *
     * The mesh is a 4x4x4 vertex lattice where each unit cube is decomposed
     * into six tetrahedra, then cell adjacency is connected.
     *
     * @param[in,out] M The mesh to clear and rebuild.
     */
    inline void build_Cartesian_tet_mesh(
        GEO::Mesh& M
        ) {
        M.clear();

        GEO::index_t new_v = M.vertices.create_vertices(4*4*4);
        for (GEO::index_t i = 0; i < 4; ++i) {
            for (GEO::index_t j = 0; j < 4; ++j) {
                for (GEO::index_t k = 0; k < 4; ++k)
                    M.vertices.point(new_v++) = GEO::vec3(i, j, k);
            }
        }

        GEO::index_t new_c = M.cells.create_tets(3*3*3*6);
        for (GEO::index_t i = 0; i < 3; ++i) {
            for (GEO::index_t j = 0; j < 3; ++j) {
                for (GEO::index_t k = 0; k < 3; ++k) {
                    const GEO::index_t v0 = 16*i+4*j+k;
                    const GEO::index_t v1 = v0+16;
                    const GEO::index_t v2 = v0+4;
                    const GEO::index_t v3 = v2+16;
                    const GEO::index_t v4 = v0+1;
                    const GEO::index_t v5 = v4+16;
                    const GEO::index_t v6 = v4+4;
                    const GEO::index_t v7 = v6+16;
                    M.cells.set_vertex(new_c+0, 0, v4); M.cells.set_vertex(new_c+0, 1, v6); M.cells.set_vertex(new_c+0, 2, v5); M.cells.set_vertex(new_c+0, 3, v1);
                    M.cells.set_vertex(new_c+1, 0, v0); M.cells.set_vertex(new_c+1, 1, v6); M.cells.set_vertex(new_c+1, 2, v4); M.cells.set_vertex(new_c+1, 3, v1);
                    M.cells.set_vertex(new_c+2, 0, v0); M.cells.set_vertex(new_c+2, 1, v1); M.cells.set_vertex(new_c+2, 2, v2); M.cells.set_vertex(new_c+2, 3, v6);
                    M.cells.set_vertex(new_c+3, 0, v1); M.cells.set_vertex(new_c+3, 1, v3); M.cells.set_vertex(new_c+3, 2, v2); M.cells.set_vertex(new_c+3, 3, v6);
                    M.cells.set_vertex(new_c+4, 0, v1); M.cells.set_vertex(new_c+4, 1, v7); M.cells.set_vertex(new_c+4, 2, v3); M.cells.set_vertex(new_c+4, 3, v6);
                    M.cells.set_vertex(new_c+5, 0, v1); M.cells.set_vertex(new_c+5, 1, v6); M.cells.set_vertex(new_c+5, 2, v5); M.cells.set_vertex(new_c+5, 3, v7);
                    new_c += 6;
                }
            }
        }

        M.cells.connect();
    }

    /**
     * Build a regular tetrahedral reference mesh with layered planar samples.
     *
     * The mesh is assembled from predefined vertex layers and tetrahedral cell
     * index tables, then connectivity is finalized with `M.cells.connect()`.
     *
     * @param[in,out] M The mesh to clear and rebuild.
     */
    inline void build_regular_tet_mesh(
        GEO::Mesh& M
        ) {
        M.clear();

        const double SQRT_3 = std::sqrt(3.0);

        const std::array<GEO::vec2, 7> L0_VERTICES = {
            {
                GEO::vec2(-SQRT_3, 3), GEO::vec2(SQRT_3, 3),
                GEO::vec2(-2*SQRT_3, 0), GEO::vec2(0, 0), GEO::vec2(2*SQRT_3, 0),
                GEO::vec2(-SQRT_3, -3), GEO::vec2(SQRT_3, -3)
            }
        };
        const std::array<GEO::vec2, 6> L1_VERTICES = {
            {
                GEO::vec2(-SQRT_3, 1), GEO::vec2(0, 2), GEO::vec2(SQRT_3, 1),
                GEO::vec2(-SQRT_3, -1), GEO::vec2(0, -2), GEO::vec2(SQRT_3, -1)
            }
        };

        GEO::index_t new_v = M.vertices.create_vertices(7+6+7+6+7);
        for (const auto& p : L0_VERTICES)
            M.vertices.point(new_v++) = GEO::vec3(p.x, p.y, 3);
        for (const auto& p : L1_VERTICES)
            M.vertices.point(new_v++) = GEO::vec3(p.x, p.y, 1);
        for (const auto& p : L0_VERTICES)
            M.vertices.point(new_v++) = GEO::vec3(p.x, p.y, 0);
        for (const auto& p : L1_VERTICES)
            M.vertices.point(new_v++) = GEO::vec3(p.x, p.y, -2);
        for (const auto& p : L0_VERTICES)
            M.vertices.point(new_v++) = GEO::vec3(p.x, p.y, -3);

        constexpr std::array<GEO::index_t, 24> L0_CELLS = {
            {
                0, 3, 2, 7,
                0, 1, 3, 8,
                1, 4, 3, 9,
                2, 3, 5, 10,
                3, 6, 5, 11,
                3, 4, 6, 12
            }
        };
        constexpr std::array<GEO::index_t, 144> L1_CELLS = {
            {
                0, 2, 15, 7,    0, 15, 13, 7,
                1, 0, 13, 8,    1, 13, 14, 8,
                4, 1, 14, 9,    4, 14, 17, 9,
                2, 5, 18, 10,   2, 18, 15, 10,
                5, 6, 19, 11,   5, 19, 18, 11,
                6, 4, 17, 12,   6, 17, 19, 12,
                3, 0, 13, 7,    3, 13, 16, 7,   0, 3, 13, 8,    13, 3, 16, 8,
                3, 1, 14, 8,    3, 14, 16, 8,   1, 3, 14, 9,    14, 3, 16, 9,
                2, 3, 16, 7,    2, 16, 15, 7,   3, 2, 16, 10,   16, 2, 15, 10,
                3, 4, 17, 9,    3, 17, 16, 9,   4, 3, 17, 12,   17, 3, 16, 12,
                5, 3, 16, 10,   5, 16, 18, 10,  3, 5, 16, 11,   16, 5, 18, 11,
                6, 3, 16, 11,   6, 16, 19, 11,  3, 6, 16, 12,   16, 6, 19, 12
            }
        };
        constexpr std::array<GEO::index_t, 72> L2_CELLS = {
            {
                7, 15, 13, 20,      7, 13, 16, 20,      7, 16, 15, 20,
                8, 13, 14, 21,      8, 14, 16, 21,      8, 16, 13, 21,
                9, 16, 14, 22,      9, 14, 17, 22,      9, 17, 16, 22,
                10, 18, 15, 23,     10, 15, 16, 23,     10, 16, 18, 23,
                11, 18, 16, 24,     11, 16, 19, 24,     11, 19, 18, 24,
                12, 16, 17, 25,     12, 17, 19, 25,     12, 19, 16, 25
            }
        };
        constexpr std::array<GEO::index_t, 24> L3_CELLS = {
            {
                26, 28, 29, 20,
                26, 29, 27, 21,
                29, 30, 27, 22,
                28, 31, 29, 23,
                29, 31, 32, 24,
                29, 32, 30, 25
            }
        };

        GEO::index_t new_c = M.cells.create_tets((L0_CELLS.size()+L1_CELLS.size()+L2_CELLS.size()+L1_CELLS.size()+L3_CELLS.size())/4);
        for (GEO::index_t i = 0, i_end = L0_CELLS.size(); i < i_end;) {
            for (GEO::index_t lv = 0; lv < 4; ++lv)
                M.cells.set_vertex(new_c, lv, L0_CELLS[i++]);
            ++new_c;
        }
        for (GEO::index_t i = 0, i_end = L1_CELLS.size(); i < i_end;) {
            for (GEO::index_t lv = 0; lv < 4; ++lv)
                M.cells.set_vertex(new_c, lv, L1_CELLS[i++]);
            ++new_c;
        }
        for (GEO::index_t i = 0, i_end = L2_CELLS.size(); i < i_end;) {
            for (GEO::index_t lv = 0; lv < 4; ++lv)
                M.cells.set_vertex(new_c, lv, L2_CELLS[i++]);
            ++new_c;
        }
        for (GEO::index_t i = 0, i_end = L1_CELLS.size(); i < i_end;) {
            for (GEO::index_t lv = 0; lv < 4; ++lv)
                M.cells.set_vertex(new_c, lv, 13+L1_CELLS[i++]);
            ++new_c;
        }
        for (GEO::index_t i = 0, i_end = L3_CELLS.size(); i < i_end;) {
            for (GEO::index_t lv = 0; lv < 4; ++lv)
                M.cells.set_vertex(new_c, lv, L3_CELLS[i++]);
            ++new_c;
        }

        M.cells.connect();
    }

    /**
     * Build the active tetrahedral test mesh and validate cell orientation.
     *
     * This helper currently builds the Cartesian reference mesh (with an optional
     * regular-mesh alternative kept in code comments) and asserts that all cells
     * have non-negative signed volume.
     *
     * @param[in,out] M The mesh to clear and rebuild.
     */
    inline void build_tet_mesh(
        GEO::Mesh& M
        ) {
        build_Cartesian_tet_mesh(M);
        // build_regular_tet_mesh(M);

        assert([&]() {
            for (const auto& c : M.cells) {
                if (GEO::Geom::tetra_signed_volume(
                    M.cells.point(c, 0), M.cells.point(c, 1), M.cells.point(c, 2), M.cells.point(c, 3)
                    ) < 0)
                    return false;
            }
            return true;
        }());
    }

    enum TETRAHEDRAL_MESH_TEST_PARAMS_TYPE {
        INTERIOR_VERTEX_C_LV,
        BORDER_VERTEX_C_LV,
        INTERIOR_EDGE_C_LF_LV,
        BORDER_EDGE_C_LF_LV,
        INTERIOR_EDGE_C_LE,
        BORDER_EDGE_C_LE,
        INTERIOR_FACET_C_LF,
        BORDER_FACET_C_LF
    };

    /**
     * Collect test parameter tuples from the reference tetrahedral mesh.
     *
     * The returned tuple layout is `(c, x, y)` and depends on @p type:
     * - vertex/edge/facet selectors store indices in `(c, x)` and set `y` to `GEO::NO_INDEX`;
     * - facet-local-edge selectors store `(c, lf, lv)`.
     *
     * @param[in] type The category of parameters to generate.
     * @return A vector of index tuples matching the requested category.
     */
    const auto TETRAHEDRON_MESH_GET_TEST_PARAMS = [](
        const TETRAHEDRAL_MESH_TEST_PARAMS_TYPE type
        ) {
        GEO::initialize(GEO::GEOGRAM_INSTALL_ALL);

        GEO::Mesh M;
        build_tet_mesh(M);

        /* Find all border vertices and edges */
        std::vector<bool> M_v_border(M.vertices.nb(), false);
        std::unordered_set<std::pair<GEO::index_t, GEO::index_t>, GEO::MeshUtils::PairHash> border_edges;
        for (const auto& c : M.cells) {
            for (GEO::index_t lf = 0; lf < 4; ++lf) {
                if (M.cells.adjacent(c, lf) != GEO::NO_CELL)
                    continue;

                for (GEO::index_t lv = 0; lv < 3; ++lv) {
                    M_v_border[M.cells.facet_vertex(c, lf, lv)] = true;;

                    const std::pair<GEO::index_t, GEO::index_t> edge = std::minmax(
                        M.cells.facet_vertex(c, lf, lv), M.cells.facet_vertex(c, lf, (lv+1)%3));
                    border_edges.insert(edge);
                }
            }
        }

        std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>> output;
        switch (type) {
            case INTERIOR_VERTEX_C_LV:
                for (const auto& c : M.cells) {
                    for (GEO::index_t lv = 0; lv < 4; ++lv) {
                        if (!M_v_border[M.cells.vertex(c, lv)])
                            output.emplace_back(c, lv, GEO::NO_INDEX);
                    }
                }
                break;
            case BORDER_VERTEX_C_LV:
                for (const auto& c : M.cells) {
                    for (GEO::index_t lv = 0; lv < 4; ++lv) {
                        if (M_v_border[M.cells.vertex(c, lv)])
                            output.emplace_back(c, lv, GEO::NO_INDEX);
                    }
                }
                break;
            case INTERIOR_EDGE_C_LE:
                for (const auto& c : M.cells) {
                    for (GEO::index_t le = 0; le < 6; ++le) {
                        const std::pair<GEO::index_t, GEO::index_t> edge = std::minmax(
                            M.cells.edge_vertex(c, le, 0), M.cells.edge_vertex(c, le, 1));
                        if (!border_edges.contains(edge))
                            output.emplace_back(c, le, GEO::NO_INDEX);
                    }
                }
                break;
            case BORDER_EDGE_C_LE:
                for (const auto& c : M.cells) {
                    for (GEO::index_t le = 0; le < 6; ++le) {
                        const std::pair<GEO::index_t, GEO::index_t> edge = std::minmax(
                            M.cells.edge_vertex(c, le, 0), M.cells.edge_vertex(c, le, 1));
                        if (border_edges.contains(edge))
                            output.emplace_back(c, le, GEO::NO_INDEX);
                    }
                }
                break;
            case INTERIOR_EDGE_C_LF_LV:
                for (const auto& c : M.cells) {
                    for (GEO::index_t lf = 0; lf < 4; ++lf) {
                        for (GEO::index_t lv = 0; lv < 3; ++lv) {
                            const std::pair<GEO::index_t, GEO::index_t> edge = std::minmax(
                                    M.cells.facet_vertex(c, lf, lv), M.cells.facet_vertex(c, lf, (lv+1)%3));
                            if (!border_edges.contains(edge))
                                output.emplace_back(c, lf, lv);
                        }
                    }
                }
                break;
            case BORDER_EDGE_C_LF_LV:
                for (const auto& c : M.cells) {
                    for (GEO::index_t lf = 0; lf < 4; ++lf) {
                        for (GEO::index_t lv = 0; lv < 3; ++lv) {
                            const std::pair<GEO::index_t, GEO::index_t> edge = std::minmax(
                                    M.cells.facet_vertex(c, lf, lv), M.cells.facet_vertex(c, lf, (lv+1)%3));
                            if (border_edges.contains(edge))
                                output.emplace_back(c, lf, lv);
                        }
                    }
                }
                break;
            case INTERIOR_FACET_C_LF:
                for (const auto& c : M.cells) {
                    for (GEO::index_t lf = 0; lf < 4; ++lf) {
                        if (M.cells.adjacent(c, lf) != GEO::NO_CELL)
                            output.emplace_back(c, lf, GEO::NO_INDEX);
                    }
                }
                break;
            case BORDER_FACET_C_LF:
                for (const auto& c : M.cells) {
                    for (GEO::index_t lf = 0; lf < 4; ++lf) {
                        if (M.cells.adjacent(c, lf) == GEO::NO_CELL)
                            output.emplace_back(c, lf, GEO::NO_INDEX);
                    }
                }
                break;
            default:
                throw std::runtime_error("Invalid type!");
        }

        return output;
    };

    class TetrahedronOperationsTest : public ::testing::TestWithParam<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>> {
        /**
         * Build the shared reference mesh and bind per-cell test attributes.
         */
        void SetUp() override {
            build_tet_mesh(M);
            M_c_affected.bind(M.cells.attributes(), "affected");
        }

    public:
        /**
         * Remove inverted tetrahedra from the current test mesh.
         *
         * A cell is marked for deletion when its signed volume is negative,
         * then all marked cells are removed in one batch.
         */
        void clean_inverse_tets() {
            GEO::vector<GEO::index_t> cells_to_delete(M.cells.nb(), 0);
            for (const auto& c : M.cells) {
                if (GEO::Geom::tetra_signed_volume(
                    M.cells.point(c, 0), M.cells.point(c, 1), M.cells.point(c, 2), M.cells.point(c, 3)
                    ) < 0)
                    cells_to_delete[c] = 1;
            }
            M.cells.delete_elements(cells_to_delete);
        }

        /**
         * Verify that reconnecting the mesh preserves the current cell adjacency layout.
         */
        void check_connections() {
            // clean_inverse_tets();

            std::vector<GEO::index_t> current_connections(4*M.cells.nb(), GEO::NO_CELL);
            for (const auto& c : M.cells) {
                for (GEO::index_t lf = 0; lf < 4; ++lf)
                    current_connections[4*c+lf] = M.cells.adjacent(c, lf);
            }

            M.cells.connect();
            for (const auto& c : M.cells) {
                for (GEO::index_t lf = 0; lf < 4; ++lf)
                    EXPECT_EQ(current_connections[4*c+lf], M.cells.adjacent(c, lf));
            }

            /* Rollback adjacency */
            for (const auto& c : M.cells) {
                for (GEO::index_t lf = 0; lf < 4; ++lf)
                    M.cells.set_adjacent(c, lf, current_connections[4*c+lf]);
            }
        }

        /**
         * Save the current mesh result for a cell-based test case.
         *
         * @param[in] c Cell index used in the output filename.
         */
        void save_results_c(
            const GEO::index_t c
            ) const {
            EXPECT_TRUE(GEO::mesh_save(M, get_current_test_name() +
                                            "_c" + std::to_string(c) +
                                            ".geogram"));
        }

        /**
         * Save the current mesh result for a `(cell, local-facet, local-vertex)` test case.
         *
         * @param[in] c  Cell index used in the output filename.
         * @param[in] lf Local facet index used in the output filename.
         * @param[in] lv Local vertex index used in the output filename.
         */
        void save_results_c_lf_lv(
            const GEO::index_t c,
            const GEO::index_t lf,
            const GEO::index_t lv
            ) const {
            EXPECT_TRUE(GEO::mesh_save(M, get_current_test_name() +
                                            "_c" + std::to_string(c) +
                                            "_lf" + std::to_string(lf) +
                                            "_lv" + std::to_string(lv) +
                                            ".geogram"));
        }

        /**
         * Save the current mesh result for a `(cell, local-edge)` test case.
         *
         * @param[in] c  Cell index used in the output filename.
         * @param[in] le Local edge index used in the output filename.
         */
        void save_results_c_le(
            const GEO::index_t c,
            const GEO::index_t le
            ) const {
            EXPECT_TRUE(GEO::mesh_save(M, get_current_test_name() +
                                            "_c" + std::to_string(c) +
                                            "_le" + std::to_string(le) +
                                            ".geogram"));
        }

        /**
         * Save the current mesh result for a `(cell, local-facet)` test case.
         *
         * @param[in] c  Cell index used in the output filename.
         * @param[in] lf Local facet index used in the output filename.
         */
        void save_results_c_lf(
            const GEO::index_t c,
            const GEO::index_t lf
            ) const {
            EXPECT_TRUE(GEO::mesh_save(M, get_current_test_name() +
                                            "_c" + std::to_string(c) +
                                            "_lf" + std::to_string(lf) +
                                            ".geogram"));
        }

        GEO::Mesh M;
        GEO::Attribute<GEO::index_t> M_c_affected;
    };
}

#endif //GEOGRAMMESHUTILS_TEST_TET_OPEARTIONS_H