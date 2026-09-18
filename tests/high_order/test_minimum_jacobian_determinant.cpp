//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/11.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <gtest/gtest.h>
#include <geolio/high_order/minimum_jacobian_determinant.h>
#include <geolio/high_order/hex_control_grid.h>
#include <geolio/high_order/quad_control_grid.h>
#include "../utils.h"
#include <geolio/common/log.h>

namespace geolio::test
{
    class HexMimimumJacobianDeterminantTest : public ::testing::Test {
    protected:
        void SetUp() override {
            mesh.vertices.create_vertices(12);
            mesh.vertices.point(0) = GEO::vec3(0, 0, 0);
            mesh.vertices.point(1) = GEO::vec3(1, 0, 0);
            mesh.vertices.point(2) = GEO::vec3(0, 1, 0);
            mesh.vertices.point(3) = GEO::vec3(1, 1, 0);
            mesh.vertices.point(4) = GEO::vec3(0, 0, 1);
            mesh.vertices.point(5) = GEO::vec3(1, 0, 1);
            mesh.vertices.point(6) = GEO::vec3(0, 1, 1);
            mesh.vertices.point(7) = GEO::vec3(1, 1, 1);
            mesh.vertices.point(8) = GEO::vec3(0, 2, 0);
            mesh.vertices.point(9) = GEO::vec3(1, 2, 0);
            mesh.vertices.point(10) = GEO::vec3(0, 2, 1);
            mesh.vertices.point(11) = GEO::vec3(1, 2, 1);
            mesh.cells.create_hex(0, 1, 2, 3, 4, 5, 6, 7);
            mesh.cells.create_hex(2, 8, 6, 10, 3, 9, 7, 11);
            mesh.cells.connect();

            constexpr GEO::index_t order = 4;
            control_grid = std::make_unique<HexControlGrid>(mesh, order);

            /* Inverse */
            const auto nd = control_grid->cell_nd(0, 2, 3, 1);
            control_grid->control_node(nd) += GEO::vec3(-0.2, -0.4, 0.1);
        }

        GEO::Mesh mesh;
        std::unique_ptr<HexControlGrid> control_grid;
    };

    TEST_F(HexMimimumJacobianDeterminantTest, check) {
        MinimumJacobianDeterminant<HexControlGrid> MJD(*control_grid);

        EXPECT_TRUE(MJD.contains_inverted_region(0));
        EXPECT_FALSE(MJD.contains_inverted_region(1));

        {
            std::vector<MinimumJacobianDeterminant<HexControlGrid>::Block> c0_travelled_sub_blocks;
            std::vector<MinimumJacobianDeterminant<HexControlGrid>::Block> c1_travelled_sub_blocks;
            const auto c0_bound = MJD.compute_lower_bound(0, 1e-10, &c0_travelled_sub_blocks);
            const auto c1_bound = MJD.compute_lower_bound(1, 1e-10, &c1_travelled_sub_blocks);
            EXPECT_LT(c0_bound, 0);
            EXPECT_GT(c1_bound, 0);
            {
                GEO::Mesh mesh_out;
                MJD.append_blocks_to_mesh(c0_travelled_sub_blocks, mesh_out);
                mesh_out.save(get_current_test_name()+"_c0_travelled_blocks.geogram");
            }
            {
                GEO::Mesh mesh_out;
                MJD.append_blocks_to_mesh(c1_travelled_sub_blocks, mesh_out);
                mesh_out.save(get_current_test_name()+"_c1_travelled_blocks.geogram");
            }
        }

        {
            std::vector<MinimumJacobianDeterminant<HexControlGrid>::Block> c0_invalid_sub_blocks;
            std::vector<MinimumJacobianDeterminant<HexControlGrid>::Block> c1_invalid_sub_blocks;
            MJD.collect_invalid_sub_blocks(0, c0_invalid_sub_blocks, 1);
            MJD.collect_invalid_sub_blocks(1, c1_invalid_sub_blocks, 1);
            EXPECT_TRUE(c1_invalid_sub_blocks.empty());
            {
                GEO::Mesh mesh_out;
                MJD.append_blocks_to_mesh(c0_invalid_sub_blocks, mesh_out);
                mesh_out.save(get_current_test_name()+"_c0_invalid_blocks.geogram");
            }
        }
    }

    template <GEO::index_t DIM>
    class QuadMimimumJacobianDeterminantTest : public ::testing::Test {
    protected:
        void save_high_order_mesh_facets(const std::string& filepath) {
            ASSERT_FALSE(control_grid == nullptr);

            GEO::Mesh mesh_out(DIM);
            GEO::Attribute<GEO::index_t> mesh_out_v_facet(mesh_out.vertices.attributes(), "facet");
            GEO::Attribute<GEO::vec2> mesh_out_v_uv(mesh_out.vertices.attributes(), "uv");
            GEO::Attribute<GEO::index_t> mesh_out_f_facet(mesh_out.facets.attributes(), "facet");

            control_grid->append_discretized_high_order_facets(
                mesh_out,
                20,
                &mesh_out_v_facet,
                &mesh_out_v_uv,
                &mesh_out_f_facet);

            eval_vertices_quality(mesh_out, mesh_out_v_facet, mesh_out_v_uv);

            if (const auto& v_quantities = control_grid->control_nodes_quantities();
                v_quantities.is_bound()
                ) {
                const auto dim = v_quantities.dimension();
                GEO::Attribute<double> mesh_out_v_quantities;
                mesh_out_v_quantities.create_vector_attribute(mesh_out.vertices.attributes(), "quantities", dim);
                for (const auto& v : mesh_out.vertices) {
                    control_grid->compute_facet_uv_quantities(
                        mesh_out_v_facet[v],
                        mesh_out_v_uv[v],
                        &mesh_out_v_quantities[dim*v]);
                }
                }

            EXPECT_TRUE(mesh_out.save(filepath));
        }

        void eval_vertices_quality(
            const GEO::Mesh& mesh_out,
            const GEO::Attribute<GEO::index_t>& mesh_out_v_facet,
            const GEO::Attribute<GEO::vec2>& mesh_out_v_uv
            ) const {
            GEO::Attribute<double> mesh_out_v_det_jacobian(mesh_out.vertices.attributes(), "det_jacobian");
            GEO::Attribute<double> mesh_out_v_absolute_area(mesh_out.vertices.attributes(), "absolute_area");
            GEO::Attribute<double> mesh_out_v_scaled_jacobian(mesh_out.vertices.attributes(), "scaled_jacobian");
            GEO::Attribute<double> mesh_out_v_inverse_mean_ratio(mesh_out.vertices.attributes(), "inverse_mean_ratio");
            GEO::Attribute<double> mesh_out_v_MIPS(mesh_out.vertices.attributes(), "MIPS");
            for (const auto& v : mesh_out.vertices) {
                const auto& c = mesh_out_v_facet[v];
                const auto& uv = mesh_out_v_uv[v];
                mesh_out_v_det_jacobian[v] = control_grid->compute_facet_uv_measure(
                    c, uv, QuadControlGrid<DIM>::MeasureType::DET_JACOBIAN);
                mesh_out_v_absolute_area[v] = control_grid->compute_facet_uv_measure(
                    c, uv, QuadControlGrid<DIM>::MeasureType::ABSOLUTE_SQ_AREA);
                mesh_out_v_scaled_jacobian[v] = control_grid->compute_facet_uv_measure(
                    c, uv, QuadControlGrid<DIM>::MeasureType::SCALED_JACOBIAN);
                mesh_out_v_inverse_mean_ratio[v] = control_grid->compute_facet_uv_measure(
                    c, uv, QuadControlGrid<DIM>::MeasureType::INVERSE_MEAN_RATIO);
                mesh_out_v_MIPS[v] = control_grid->compute_facet_uv_measure(
                    c, uv, QuadControlGrid<DIM>::MeasureType::MIPS);
            }
        }

        GEO::Mesh mesh;
        std::unique_ptr<QuadControlGrid<DIM>> control_grid;
    };

    template <typename DimType>
    class QuadMimimumJacobianDeterminantDIMTest : public QuadMimimumJacobianDeterminantTest<DimType::value> {
    protected:
        void SetUp() override {
            this->mesh.vertices.set_dimension(DimType::value);
            this->mesh.vertices.create_vertices(6);
            if constexpr (DimType::value == 2) {
                this->mesh.vertices.template point<2>(0) = GEO::vec2(0, 0);
                this->mesh.vertices.template point<2>(1) = GEO::vec2(1, 0);
                this->mesh.vertices.template point<2>(2) = GEO::vec2(1, 1);
                this->mesh.vertices.template point<2>(3) = GEO::vec2(0, 1);
                this->mesh.vertices.template point<2>(4) = GEO::vec2(2, 0);
                this->mesh.vertices.template point<2>(5) = GEO::vec2(2, 1);
            }
            else if constexpr (DimType::value == 3) {
                this->mesh.vertices.point(0) = GEO::vec3(0, 0, 0);
                this->mesh.vertices.point(1) = GEO::vec3(1, 0, 0);
                this->mesh.vertices.point(2) = GEO::vec3(1, 1, 0);
                this->mesh.vertices.point(3) = GEO::vec3(0, 1, 0);
                this->mesh.vertices.point(4) = GEO::vec3(2, 0, 0);
                this->mesh.vertices.point(5) = GEO::vec3(2, 1, 0);
            }
            this->mesh.facets.create_quad(0, 1, 2, 3);
            this->mesh.facets.create_quad(5, 2, 1, 4);
            this->mesh.facets.connect();

            constexpr GEO::index_t order = 5;
            this->control_grid = std::make_unique<QuadControlGrid<DimType::value>>(this->mesh, order);

            const auto nd = this->control_grid->facet_inner_nd(1, 3, 2);
            if constexpr (DimType::value == 2)
                this->control_grid->control_node(nd) += GEO::vec2(-0.4, 0.1);
            else if constexpr (DimType::value == 3)
                this->control_grid->control_node(nd) += GEO::vec3(0.2, -0.4, 0.1);
        }
    };

    TYPED_TEST_SUITE(QuadMimimumJacobianDeterminantDIMTest, DimTypes);

    TYPED_TEST(QuadMimimumJacobianDeterminantDIMTest, check_detJ) {
        constexpr GEO::index_t DIM = TypeParam::value;
        MinimumJacobianDeterminant<QuadControlGrid<DIM>> MJD(*(this->control_grid));

        this->save_high_order_mesh_facets(get_current_test_name()+"_ho_mesh.geogram");

        EXPECT_FALSE(MJD.contains_inverted_region(0));
        EXPECT_TRUE(MJD.contains_inverted_region(1));

        {
            std::vector<typename MinimumJacobianDeterminant<QuadControlGrid<DIM>>::Block> f0_travelled_sub_blocks;
            std::vector<typename MinimumJacobianDeterminant<QuadControlGrid<DIM>>::Block> f1_travelled_sub_blocks;
            const auto c0_bound = MJD.compute_lower_bound(0, 1e-10, &f0_travelled_sub_blocks);
            const auto c1_bound = MJD.compute_lower_bound(1, 1e-10, &f1_travelled_sub_blocks);
            LOG::DEBUG("c0_bound: {}, c1_bound: {}", c0_bound, c1_bound);
            EXPECT_GT(c0_bound, 0);
            EXPECT_LT(c1_bound, 0);
            {
                GEO::Mesh mesh_out;
                MJD.append_blocks_to_mesh(f0_travelled_sub_blocks, mesh_out);
                mesh_out.save(get_current_test_name()+"_f0_travelled_blocks.geogram");
            }
            {
                GEO::Mesh mesh_out;
                MJD.append_blocks_to_mesh(f1_travelled_sub_blocks, mesh_out);
                mesh_out.save(get_current_test_name()+"_f1_travelled_blocks.geogram");
            }
        }

        {
            std::vector<typename MinimumJacobianDeterminant<QuadControlGrid<DIM>>::Block> f0_invalid_sub_blocks;
            std::vector<typename MinimumJacobianDeterminant<QuadControlGrid<DIM>>::Block> f1_invalid_sub_blocks;
            MJD.collect_invalid_sub_blocks(0, f0_invalid_sub_blocks, 1);
            MJD.collect_invalid_sub_blocks(1, f1_invalid_sub_blocks, 1);
            EXPECT_TRUE(f0_invalid_sub_blocks.empty());
            {
                GEO::Mesh mesh_out;
                MJD.append_blocks_to_mesh(f1_invalid_sub_blocks, mesh_out);
                mesh_out.save(get_current_test_name()+"_f1_invalid_blocks.geogram");
            }
        }
    }

    // TYPED_TEST(QuadMimimumJacobianDeterminantDIMTest, check_absolute_area) {
    //     constexpr GEO::index_t DIM = TypeParam::value;
    //     MinimumJacobianDeterminant<QuadControlGrid<DIM>> MJD(*(this->control_grid), true);
    //
    //     this->save_high_order_mesh_facets(get_current_test_name()+"_ho_mesh.geogram");
    //
    //     EXPECT_FALSE(MJD.contains_inverted_region(0));
    //     if constexpr (DIM == 2)
    //         EXPECT_TRUE(MJD.contains_inverted_region(1));
    //     else
    //         EXPECT_FALSE(MJD.contains_inverted_region(1));
    //
    //     {
    //         std::vector<typename MinimumJacobianDeterminant<QuadControlGrid<DIM>>::Block> f0_travelled_sub_blocks;
    //         std::vector<typename MinimumJacobianDeterminant<QuadControlGrid<DIM>>::Block> f1_travelled_sub_blocks;
    //         const auto c0_bound = MJD.compute_lower_bound(0, 1e-10, &f0_travelled_sub_blocks);
    //         const auto c1_bound = MJD.compute_lower_bound(1, 1e-10, &f1_travelled_sub_blocks);
    //         LOG::DEBUG("c0_bound: {}, c1_bound: {}", c0_bound, c1_bound);
    //         EXPECT_GT(c0_bound, 0);
    //         if constexpr (DIM == 2)
    //             EXPECT_LT(c1_bound, 0);
    //         else
    //             EXPECT_GT(c1_bound, 0);
    //         {
    //             GEO::Mesh mesh_out;
    //             MJD.append_blocks_to_mesh(f0_travelled_sub_blocks, mesh_out);
    //             mesh_out.save(get_current_test_name()+"_f0_travelled_blocks.geogram");
    //         }
    //         {
    //             GEO::Mesh mesh_out;
    //             MJD.append_blocks_to_mesh(f1_travelled_sub_blocks, mesh_out);
    //             mesh_out.save(get_current_test_name()+"_f1_travelled_blocks.geogram");
    //         }
    //     }
    //
    //     {
    //         std::vector<typename MinimumJacobianDeterminant<QuadControlGrid<DIM>>::Block> f0_invalid_sub_blocks;
    //         std::vector<typename MinimumJacobianDeterminant<QuadControlGrid<DIM>>::Block> f1_invalid_sub_blocks;
    //         MJD.collect_invalid_sub_blocks(0, f0_invalid_sub_blocks, 1);
    //         MJD.collect_invalid_sub_blocks(1, f1_invalid_sub_blocks, 1);
    //         EXPECT_TRUE(f0_invalid_sub_blocks.empty());
    //         // if constexpr (DIM == 2)
    //         //     EXPECT_TRUE(f1_invalid_sub_blocks.empty());
    //         // else {
    //             EXPECT_FALSE(f1_invalid_sub_blocks.empty());
    //             {
    //                 GEO::Mesh mesh_out;
    //                 MJD.append_blocks_to_mesh(f1_invalid_sub_blocks, mesh_out);
    //                 mesh_out.save(get_current_test_name()+"_f1_invalid_blocks.geogram");
    //             }
    //         // }
    //     }
    // }
}
