//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/11.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <gtest/gtest.h>
#include <geolio/high_order/minimum_jacobian_determinant.h>
#include <geolio/high_order/hex_control_grid.h>
#include <geolio/high_order/quad_control_grid.h>

#include "../utils.h"

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
        MinimumJacobianDeterminant<3, HexControlGrid> MJD(*control_grid);

        EXPECT_TRUE(MJD.contains_inverted_region(0));
        EXPECT_FALSE(MJD.contains_inverted_region(1));

        std::vector<MinimumJacobianDeterminant<3, HexControlGrid>::Block> c0_travelled_sub_blocks;
        std::vector<MinimumJacobianDeterminant<3, HexControlGrid>::Block> c1_travelled_sub_blocks;
        const auto c0_upper_bound = MJD.compute_upper_bound(0, 1e-10, &c0_travelled_sub_blocks);
        const auto c1_upper_bound = MJD.compute_upper_bound(1, 1e-10, &c1_travelled_sub_blocks);
        EXPECT_LT(c0_upper_bound, 0);
        EXPECT_GT(c1_upper_bound, 0);
        {
            GEO::Mesh mesh_out;
            MJD.append_blocks_to_mesh(c0_travelled_sub_blocks, mesh_out);
            mesh_out.save(get_current_test_name()+"_c0_blocks.geogram");
        }
        {
            GEO::Mesh mesh_out;
            MJD.append_blocks_to_mesh(c1_travelled_sub_blocks, mesh_out);
            mesh_out.save(get_current_test_name()+"_c1_blocks.geogram");
        }
    }

    template <GEO::index_t DIM>
    class QuadMimimumJacobianDeterminantTest : public ::testing::Test {
    protected:
        GEO::Mesh mesh;
        std::unique_ptr<QuadControlGrid<DIM>> control_grid;
    };

    template<GEO::index_t DIM>
    struct DimWrapper {
        static constexpr GEO::index_t value = DIM;
    };

    using Dim2 = std::integral_constant<GEO::index_t, 2>;
    using Dim3 = std::integral_constant<GEO::index_t, 3>;
    using DimTypes = ::testing::Types<Dim2, Dim3>;

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

    TYPED_TEST(QuadMimimumJacobianDeterminantDIMTest, check) {
        constexpr GEO::index_t DIM = TypeParam::value;
        MinimumJacobianDeterminant<DIM, QuadControlGrid<DIM>> MJD(*(this->control_grid));

        EXPECT_FALSE(MJD.contains_inverted_region(0));
        EXPECT_TRUE(MJD.contains_inverted_region(1));

        std::vector<typename MinimumJacobianDeterminant<DIM, QuadControlGrid<DIM>>::Block> f0_travelled_sub_blocks;
        std::vector<typename MinimumJacobianDeterminant<DIM, QuadControlGrid<DIM>>::Block> f1_travelled_sub_blocks;
        const auto c0_upper_bound = MJD.compute_upper_bound(0, 1e-10, &f0_travelled_sub_blocks);
        const auto c1_upper_bound = MJD.compute_upper_bound(1, 1e-10, &f1_travelled_sub_blocks);
        EXPECT_GT(c0_upper_bound, 0);
        EXPECT_LT(c1_upper_bound, 0);
        {
            GEO::Mesh mesh_out;
            MJD.append_blocks_to_mesh(f0_travelled_sub_blocks, mesh_out);
            mesh_out.save(get_current_test_name()+"_f0_blocks.geogram");
        }
        {
            GEO::Mesh mesh_out;
            MJD.append_blocks_to_mesh(f1_travelled_sub_blocks, mesh_out);
            mesh_out.save(get_current_test_name()+"_f1_blocks.geogram");
        }
    }
}
