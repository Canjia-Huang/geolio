//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/3/23.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//

#include <gtest/gtest.h>
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_io.h>
#include <geogram/mesh/mesh_repair.h>
#include "mesh_opt/tetrahedron_operators.h"
#include "utils.h"
#include "common/log.h"

namespace
{
    using namespace ProgressiveMeshOpt::Tet;

    class TetrahedronOperatorsTest : public ::testing::Test {
        void SetUp() override {
            ASSERT_TRUE(GEO::mesh_load(std::string(TEST_DATA_PATH) + "sphere.mesh", M_));
            LOG::DEBUG("M #V:{}, #C: {}", M_.vertices.nb(), M_.cells.nb());
            M_.facets.clear();

            M_v_to_delete_.bind(M_.vertices.attributes(), "delete");
            M_c_to_delete_.bind(M_.cells.attributes(), "delete");
            M_c_processed_.bind(M_.cells.attributes(), "processed");
        }
    public:
        void find_all_border_and_interior_cells() {
            std::vector<bool> interior_cells(M_.cells.nb(), true);
            std::stack<GEO::index_t> S;
            for (const auto& c : M_.cells) {
                for (GEO::index_t lf = 0; lf < 4; ++lf) {
                    if (M_.cells.adjacent(c, lf) == GEO::NO_CELL) {
                        interior_cells[c] = false;
                        LOG::DEBUG("border cell: {}, lf: {}", c, lf);
                        for (GEO::index_t nlf = 0; nlf < 4; ++nlf) {
                            if (const auto& nc = M_.cells.adjacent(c, nlf);
                                nc != GEO::NO_CELL)
                                interior_cells[nc] = false;
                        }
                    }
                }
            }
            for (const auto& c : M_.cells) {
                if (interior_cells[c])
                    LOG::DEBUG("interior cell: {}", c);
            }
        }

        void clean_delete_elements() {
            auto cells_to_delete = M_c_to_delete_.get_vector();
            M_.cells.delete_elements(cells_to_delete);
        }

        void check_connections() {
            std::vector<GEO::index_t> current_connections(4*M_.cells.nb(), GEO::NO_CELL);
            for (const auto& c : M_.cells) {
                for (GEO::index_t lf = 0; lf < 4; ++lf)
                    current_connections[4*c+lf] = M_.cells.adjacent(c, lf);
            }

            M_.cells.connect();
            for (const auto& c : M_.cells) {
                for (GEO::index_t lf = 0; lf < 4; ++lf)
                    EXPECT_EQ(current_connections[4*c+lf], M_.cells.adjacent(c, lf));
            }

            /* Rollback adjacency */
            for (const auto& c : M_.cells) {
                for (GEO::index_t lf = 0; lf < 4; ++lf)
                    M_.cells.set_adjacent(c, lf, current_connections[4*c+lf]);
            }
        }

        void save_results() const {
            EXPECT_TRUE(GEO::mesh_save(M_, get_current_test_name()+".geogram"));
        }

        GEO::Mesh M_;
        GEO::Attribute<GEO::index_t> M_v_to_delete_;
        GEO::Attribute<GEO::index_t> M_c_to_delete_;
        GEO::Attribute<GEO::index_t> M_c_processed_;
    };

    /* == EdgeSwap2-3 ============================================================================================== */
    class EdgeSwap23Test : public TetrahedronOperatorsTest {
    public:
        void compute(
            const GEO::index_t c,
            const GEO::index_t lf
            ) {
            ASSERT_NE(M_.cells.adjacent(c, lf), GEO::NO_CELL);
            const GEO::index_t new_c = M_.cells.create_tets(1);

            M_c_processed_[c] = 1;
            M_c_processed_[M_.cells.adjacent(c, lf)] = 1;
            M_c_processed_[new_c] = 1;

            edge_swap_2_3(
                M_,
                c, lf,
                new_c);
        }

        void compute_on_border(
            const GEO::index_t c,
            const GEO::index_t lf
            ) {
            ASSERT_EQ(M_.cells.adjacent(c, lf), GEO::NO_CELL);

            M_c_processed_[c] = 1;

            edge_swap_2_3(
                M_,
                c, lf,
                GEO::NO_CELL);
        }
    };

    TEST_F(EdgeSwap23Test, interior) {
        compute(8, 0);
        check_connections();
        save_results();
    }

    TEST_F(EdgeSwap23Test, border) {
        compute_on_border(0, 3);
        check_connections();
        save_results();
    }
}