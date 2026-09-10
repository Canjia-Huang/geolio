//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/10.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <gtest/gtest.h>
#include <geolio/io/high_order_quad_mesh_io.h>

#include "../utils.h"

namespace geolio::test
{
    template <GEO::index_t DIM>
    class HighOrderQuadMeshIO : public ::testing::Test {
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
    class SingleQuadHighOrderQuadMeshIO : public HighOrderQuadMeshIO<DimType::value> {
    protected:
        void SetUp() override {
            this->mesh.vertices.set_dimension(DimType::value);
            this->mesh.vertices.create_vertices(4);
            if constexpr (DimType::value == 2) {
                this->mesh.vertices.template point<2>(0) = GEO::vec2(0, 0);
                this->mesh.vertices.template point<2>(1) = GEO::vec2(1, 0);
                this->mesh.vertices.template point<2>(2) = GEO::vec2(1, 1);
                this->mesh.vertices.template point<2>(3) = GEO::vec2(0, 1);
            }
            else {
                this->mesh.vertices.point(0) = GEO::vec3(0, 0, 0);
                this->mesh.vertices.point(1) = GEO::vec3(1, 0, 0);
                this->mesh.vertices.point(2) = GEO::vec3(1, 1, 0);
                this->mesh.vertices.point(3) = GEO::vec3(0, 1, 0);
            }
            this->mesh.facets.create_quad(0, 1, 2, 3);

            constexpr GEO::index_t order = 4;
            this->control_grid = std::make_unique<QuadControlGrid<DimType::value>>(this->mesh, order);
        }
    };

    TYPED_TEST_SUITE(SingleQuadHighOrderQuadMeshIO, DimTypes);

    TYPED_TEST(SingleQuadHighOrderQuadMeshIO, io) {
        constexpr GEO::index_t DIM = TypeParam::value;
        {
            auto& p = this->control_grid->control_node(this->control_grid->facet_edge_nd(0, 1, 1));
            for (GEO::index_t d = 0; d < DIM; ++d)
                p[d] += 0.1*GEO::Numeric::random_float32();
            if (DIM == 3)
                p[2] += 0.2;
        }
        {
            auto& p = this->control_grid->control_node(this->control_grid->facet_nd(0, 1, 3));
            for (GEO::index_t d = 0; d < DIM; ++d)
                p[d] += 0.1*GEO::Numeric::random_float32();
            if (DIM == 3)
                p[2] += -0.2;
        }

        const std::filesystem::path filepath = get_current_test_name()+".msh";
        if (const auto filedir = filepath.parent_path(); !filedir.empty())
            std::filesystem::create_directories(filedir);

        high_order_quad_mesh_save(*(this->control_grid), filepath);
    }

    template <typename DimType>
    class TwoQuadHighOrderQuadMeshIO : public HighOrderQuadMeshIO<DimType::value> {
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
            else {
                this->mesh.vertices.point(0) = GEO::vec3(0, 0, 0);
                this->mesh.vertices.point(1) = GEO::vec3(1, 0, 0);
                this->mesh.vertices.point(2) = GEO::vec3(1, 1, 0);
                this->mesh.vertices.point(3) = GEO::vec3(0, 1, 0);
                this->mesh.vertices.point(4) = GEO::vec3(2, 0, 0);
                this->mesh.vertices.point(5) = GEO::vec3(2, 1, 0);
            }
            this->mesh.facets.create_quad(0, 1, 2, 3);
            this->mesh.facets.create_quad(5, 2, 1, 4);

            constexpr GEO::index_t order = 5;
            this->control_grid = std::make_unique<QuadControlGrid<DimType::value>>(this->mesh, order);
        }
    };

    TYPED_TEST_SUITE(TwoQuadHighOrderQuadMeshIO, DimTypes);

    TYPED_TEST(TwoQuadHighOrderQuadMeshIO, io) {
        constexpr GEO::index_t DIM = TypeParam::value;
        {
            auto& p = this->control_grid->control_node(this->control_grid->facet_edge_nd(0, 1, 3));
            for (GEO::index_t d = 0; d < DIM; ++d)
                p[d] += 0.1*GEO::Numeric::random_float32();
            if (DIM == 3)
                p[2] += 0.2;
        }
        {
            auto& p = this->control_grid->control_node(this->control_grid->facet_nd(0, 2, 2));
            for (GEO::index_t d = 0; d < DIM; ++d)
                p[d] += 0.1*GEO::Numeric::random_float32();
            if (DIM == 3)
                p[2] -= 0.2;
        }
        {
            auto& p = this->control_grid->control_node(this->control_grid->facet_nd(1, 1, 4));
            for (GEO::index_t d = 0; d < DIM; ++d)
                p[d] += 0.1*GEO::Numeric::random_float32();
            if (DIM == 3)
                p[2] += 0.2;
        }

        const std::filesystem::path filepath = get_current_test_name()+".msh";
        if (const auto filedir = filepath.parent_path(); !filedir.empty())
            std::filesystem::create_directories(filedir);

        high_order_quad_mesh_save(*(this->control_grid), filepath);
    }
}
