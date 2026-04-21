//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/3/13.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//

#include <random>
#include <ranges>
#include <geogram/basic/command_line_args.h>
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_io.h>
#include <gtest/gtest.h>
#include "utils.h"
#include "common/log.h"
#include "mesh_utils/tri_operations.h"

namespace
{
    void build_tri_mesh(
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

    void get_mesh_f_lv(
        const GEO::Mesh& M,
        std::vector<std::pair<GEO::index_t, GEO::index_t>>& interior_vertices_f_lv,
        std::vector<std::pair<GEO::index_t, GEO::index_t>>& border_vertices_f_lv,
        std::vector<std::pair<GEO::index_t, GEO::index_t>>& interior_edges_f_lv,
        std::vector<std::pair<GEO::index_t, GEO::index_t>>& border_edges_f_lv
        ) {
        /* Find all interior and border f_lv pair */
        std::vector<bool> M_v_border(M.vertices.nb(), false);
        for (const auto& f : M.facets) {
            for (GEO::index_t lv = 0; lv < 3; ++lv) {
                if (M.facets.adjacent(f, lv) == GEO::NO_FACET) {
                    border_edges_f_lv.emplace_back(f, lv);
                    M_v_border[M.facets.vertex(f, lv)] = true;
                    M_v_border[M.facets.vertex(f, (lv+1)%3)] = true;
                }
                else
                    interior_edges_f_lv.emplace_back(f, lv);
            }
        }
        for (const auto& f : M.facets) {
            for (GEO::index_t lv = 0; lv < 3; ++lv) {
                if (M_v_border[M.facets.vertex(f, lv)])
                    border_vertices_f_lv.emplace_back(f, lv);
                else
                    interior_vertices_f_lv.emplace_back(f, lv);
            }
        }
    }
}

namespace GEO::MeshUtils::Test
{
    using namespace GEO::MeshUtils;

    class TriangleOperationsTest : public ::testing::TestWithParam<std::pair<GEO::index_t, GEO::index_t>> {
        void SetUp() override {
            build_tri_mesh(M);
        }

    public:
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
    };

     /* == For INSTANTIATE_TEST_SUITE =============================================================================== */

     enum TRIANGLE_MESH_TEST_TYPE {
         INTERIOR_VERTICES_F_LV,
         BORDER_VERTICES_F_LV,
         INTERIOR_EDGES_F_LV,
         BORDER_EDGES_F_LV
     };

     const auto TRIANGLE_MESH_GET_TEST_PARAMS = [](
         const TRIANGLE_MESH_TEST_TYPE type
         ) {
         GEO::initialize(GEO::GEOGRAM_INSTALL_ALL);

         GEO::Mesh M;
         build_tri_mesh(M);

         std::vector<std::pair<GEO::index_t, GEO::index_t>> interior_vertices_f_lv;
         std::vector<std::pair<GEO::index_t, GEO::index_t>> border_vertices_f_lv;
         std::vector<std::pair<GEO::index_t, GEO::index_t>> interior_edges_f_lv;
         std::vector<std::pair<GEO::index_t, GEO::index_t>> border_edges_f_lv;
         get_mesh_f_lv(
             M,
             interior_vertices_f_lv,
             border_vertices_f_lv,
             interior_edges_f_lv,
             border_edges_f_lv
         );

         if (type == INTERIOR_VERTICES_F_LV)
             return interior_vertices_f_lv;
         if (type == BORDER_VERTICES_F_LV)
             return border_vertices_f_lv;
         if (type == INTERIOR_EDGES_F_LV)
             return interior_edges_f_lv;
         if (type == BORDER_EDGES_F_LV)
             return border_edges_f_lv;
         assert(0);
     };

     /* == GetIncidentTrianglesTest ================================================================================= */

     class GetIncidentTrianglesTest : public TriangleOperationsTest {
     public:
         bool compute(
             const GEO::index_t start_f,
             const GEO::index_t start_lv
             ) {
             return get_vertex_incident_triangles(M, start_f, start_lv, ordered_f_and_lv);
         }

         void check_incident(
             const GEO::index_t start_f,
             const GEO::index_t start_lv
             ) {
             const GEO::index_t v = M.facets.vertex(start_f, start_lv);
             for (const auto& [f, lv] : ordered_f_and_lv)
                 EXPECT_EQ(M.facets.vertex(f, lv), v);
         }

         void check_complete(
             const GEO::index_t start_f,
             const GEO::index_t start_lv
             ) {
             const GEO::index_t v = M.facets.vertex(start_f, start_lv);

             std::unordered_map<GEO::index_t, bool> incident_facets; // (facet, found)
             for (const auto& f : M.facets) {
                 for (GEO::index_t lv = 0; lv < 3; ++lv) {
                     if (M.facets.vertex(f, lv) == v) {
                         incident_facets.emplace(f, false);
                         break;
                     }
                 }
             }

             for (const auto &f: ordered_f_and_lv | std::views::keys) {
                 auto it = incident_facets.find(f);
                 EXPECT_FALSE(it == incident_facets.end());
                 it->second = true;
             }

             for (const auto &found: incident_facets | std::views::values)
                 EXPECT_TRUE(found);
         }

         virtual void check_loop() = 0;

         std::vector<std::pair<GEO::index_t, GEO::index_t>> ordered_f_and_lv;
     };

     class GetInteriorIncidentTrianglesTest : public GetIncidentTrianglesTest {
     public:
         void check_loop() override {
             for (GEO::index_t i = 0, i_end = ordered_f_and_lv.size(); i < i_end; ++i) {
                 const auto& [f, lv] =  ordered_f_and_lv[i];
                 EXPECT_EQ(M.facets.adjacent(f, lv), ordered_f_and_lv[(i+1)%i_end].first);
             }
         }
     };

     class GetBorderIncidentTrianglesTest : public GetIncidentTrianglesTest {
     public:
         void check_loop() override {
             for (GEO::index_t i = 0, i_end = ordered_f_and_lv.size(); i < i_end; ++i) {
                 const auto& [f, lv] =  ordered_f_and_lv[i];
                 if (i == 0)
                     EXPECT_EQ(M.facets.adjacent(f, (lv+2)%3), GEO::NO_FACET);
                 else if (i == i_end-1)
                     EXPECT_EQ(M.facets.adjacent(f, lv), GEO::NO_FACET);
                 else
                     EXPECT_EQ(M.facets.adjacent(f, lv), ordered_f_and_lv[i+1].first);
             }
         }
     };

     TEST_P(GetInteriorIncidentTrianglesTest, each_vertex) {
         auto [f, lv] = GetParam();

         EXPECT_FALSE(compute(f, lv));
         check_incident(f, lv);
         check_complete(f, lv);
         check_loop();
     }

     TEST_P(GetBorderIncidentTrianglesTest, each_vertex) {
         auto [f, lv] = GetParam();

         EXPECT_TRUE(compute(f, lv));
         check_incident(f, lv);
         check_complete(f, lv);
         check_loop();
     }

     INSTANTIATE_TEST_SUITE_P(
         TriangleOperationsTest,
         GetInteriorIncidentTrianglesTest,
         ::testing::ValuesIn(TRIANGLE_MESH_GET_TEST_PARAMS(INTERIOR_VERTICES_F_LV))
     );

     INSTANTIATE_TEST_SUITE_P(
         TriangleOperationsTest,
         GetBorderIncidentTrianglesTest,
         ::testing::ValuesIn(TRIANGLE_MESH_GET_TEST_PARAMS(BORDER_VERTICES_F_LV))
     );

     /* == SplitEdgeTest ============================================================================================ */

     class SplitEdgeTest : public TriangleOperationsTest {
     public:
         virtual void compute(
             GEO::index_t f,
             GEO::index_t lv,
             double r) = 0;
     };

     class SplitInteriorEdgeTest : public SplitEdgeTest {
     public:
         void compute(
             const GEO::index_t f,
             const GEO::index_t lv,
             const double r
             ) override {
             ASSERT_NE(M.facets.adjacent(f, lv), GEO::NO_FACET);
             const GEO::index_t new_v = M.vertices.create_vertices(1);
             const GEO::index_t new_f = M.facets.create_triangles(2);

             edge_split(
                 M,
                 f, lv,
                 r,
                 new_v,
                 new_f, new_f+1);
         }
     };

     class SplitBorderEdgeTest : public SplitEdgeTest {
     public:
         void compute(
             const GEO::index_t f,
             const GEO::index_t lv,
             const double r
             ) override {
             ASSERT_EQ(M.facets.adjacent(f, lv), GEO::NO_FACET);
             const GEO::index_t new_v = M.vertices.create_vertices(1);
             const GEO::index_t new_f = M.facets.create_triangles(1);

             edge_split(
                 M,
                 f, lv,
                 r,
                 new_v,
                 new_f, GEO::NO_FACET);
         }
     };

     TEST_P(SplitInteriorEdgeTest, each_edge) {
         auto [f, lv] = GetParam();

         compute(f, lv, GEO::Numeric::random_float32());
         check_connections();
         save_results(f, lv);
     }

     TEST_P(SplitBorderEdgeTest, each_edge) {
         auto [f, lv] = GetParam();

         compute(f, lv, GEO::Numeric::random_float32());
         check_connections();
         save_results(f, lv);
     }

     INSTANTIATE_TEST_SUITE_P(
         TriangleOperationsTest,
         SplitInteriorEdgeTest,
         ::testing::ValuesIn(TRIANGLE_MESH_GET_TEST_PARAMS(INTERIOR_EDGES_F_LV))
     );

    INSTANTIATE_TEST_SUITE_P(
        TriangleOperationsTest,
        SplitBorderEdgeTest,
        ::testing::ValuesIn(TRIANGLE_MESH_GET_TEST_PARAMS(BORDER_EDGES_F_LV))
    );

     /* == CollapseEdgeTest ========================================================================================= */

     class CollapseEdgeTest : public TriangleOperationsTest {
     public:
         virtual void compute(
             GEO::index_t f,
             GEO::index_t lv,
             double r) = 0;
     };

     class CollapseInteriorEdgeTest : public CollapseEdgeTest {
     public:
         void compute(
             const GEO::index_t f,
             const GEO::index_t lv,
             const double r
             ) override {
             GEO::index_t disuse_v, disuse_f0, disuse_f1;

             edge_collapse(
                 M,
                 f, lv,
                 r,
                 disuse_v,
                 disuse_f0, disuse_f1);

             /* Clean disuse vertices and facets */
             GEO::vector<GEO::index_t> facets_to_delete(M.facets.nb(), 0);
             facets_to_delete[disuse_f0] = 1;
             EXPECT_NE(disuse_f1, GEO::NO_FACET);
             facets_to_delete[disuse_f1] = 1;
             M.facets.delete_elements(facets_to_delete);
         }
     };

     class CollapseBorderEdgeTest : public CollapseEdgeTest {
     public:
         void compute(
             const GEO::index_t f,
             const GEO::index_t lv,
             const double r
             ) override {
             GEO::index_t disuse_v, disuse_f0, disuse_f1;

             edge_collapse(
                 M,
                 f, lv,
                 r,
                 disuse_v,
                 disuse_f0, disuse_f1);

             /* Clean disuse vertices and facets */
             GEO::vector<GEO::index_t> facets_to_delete(M.facets.nb(), 0);
             facets_to_delete[disuse_f0] = 1;
             EXPECT_EQ(disuse_f1, GEO::NO_FACET);
             M.facets.delete_elements(facets_to_delete);
         }
     };

     TEST_P(CollapseInteriorEdgeTest, each_edge) {
         auto [f, lv] = GetParam();

         compute(f, lv, GEO::Numeric::random_float32());
         check_connections();
         save_results(f, lv);
     }

     TEST_P(CollapseBorderEdgeTest, each_edge) {
         auto [f, lv] = GetParam();

         compute(f, lv, GEO::Numeric::random_float32());
         check_connections();
         save_results(f, lv);
     }

     INSTANTIATE_TEST_SUITE_P(
         TriangleOperationsTest,
         CollapseInteriorEdgeTest,
         ::testing::ValuesIn(TRIANGLE_MESH_GET_TEST_PARAMS(INTERIOR_EDGES_F_LV))
     );

     INSTANTIATE_TEST_SUITE_P(
         TriangleOperationsTest,
         CollapseBorderEdgeTest,
         ::testing::ValuesIn(TRIANGLE_MESH_GET_TEST_PARAMS(BORDER_EDGES_F_LV))
     );

     /* == FlipEdgeTest ============================================================================================= */

     class FlipEdgeTest : public TriangleOperationsTest {
     public:
         virtual void compute(
             GEO::index_t f,
             GEO::index_t lv) = 0;
     };

     class FlipInteriorEdgeTest : public FlipEdgeTest {
     public:
         void compute(
             const GEO::index_t f,
             const GEO::index_t lv
             ) override {
             edge_swap(M, f, lv);
         }
     };

     TEST_P(FlipInteriorEdgeTest, each_edge) {
         auto [f, lv] = GetParam();

         compute(f, lv);
         check_connections();
         save_results(f, lv);
     }

     INSTANTIATE_TEST_SUITE_P(
         TriangleOperationsTest,
         FlipInteriorEdgeTest,
         ::testing::ValuesIn(TRIANGLE_MESH_GET_TEST_PARAMS(INTERIOR_EDGES_F_LV))
     );
}
