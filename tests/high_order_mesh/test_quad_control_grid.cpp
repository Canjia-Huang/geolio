//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/4.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <geolio/high_order/quad_control_grid.h>
#include <gtest/gtest.h>
#include "../utils.h"

namespace geolio::test
{
    template <GEO::index_t DIM>
    class QuadControlGridTest : public ::testing::Test {
    public:
        void save_control_nodes(const std::string& filepath) {
            ASSERT_FALSE(control_grid == nullptr);

            GEO::Mesh mesh_out;
            GEO::Attribute<GEO::index_t> mesh_out_v_idx(mesh_out.vertices.attributes(), "idx");

            mesh_out.vertices.create_vertices(control_grid->control_nodes_nb());
            for (const auto& v : mesh_out.vertices) {
                mesh_out.vertices.point<DIM>(v) = control_grid->control_node(v);
                mesh_out_v_idx[v] = v;
            }

            if (const auto& v_quantities = control_grid->control_nodes_quantities();
                v_quantities.is_bound()
                ) {
                const auto dim = v_quantities.dimension();
                GEO::Attribute<double> mesh_out_v_quantities;
                mesh_out_v_quantities.create_vector_attribute(mesh_out.vertices.attributes(), "quantities", dim);
                for (GEO::index_t v = 0, v_end = control_grid->control_nodes_nb(); v < v_end; ++v) {
                    for (GEO::index_t d = 0; d < dim; ++d)
                        mesh_out_v_quantities[dim*v+d] = v_quantities[dim*v+d];
                }
            }

            EXPECT_TRUE(mesh_out.save(filepath));
        }

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
    class SingleQuadControlGridTest : public QuadControlGridTest<DimType::value> {
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

    TYPED_TEST_SUITE(SingleQuadControlGridTest, DimTypes);

    TYPED_TEST(SingleQuadControlGridTest, regular) {
        this->save_control_nodes(get_current_test_name()+"_nodes.geogram");
        this->save_high_order_mesh_facets(get_current_test_name()+"_facets.geogram");
    }

    TYPED_TEST(SingleQuadControlGridTest, random) {
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
        this->save_control_nodes(get_current_test_name()+"_nodes.geogram");
        this->save_high_order_mesh_facets(get_current_test_name()+"_facets.geogram");
    }

    TYPED_TEST(SingleQuadControlGridTest, quantities) {
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

        constexpr GEO::index_t dim = 4;
        this->control_grid->create_control_node_quantities(dim);
        auto& v_quantities = this->control_grid->control_nodes_quantities();
        ASSERT_TRUE(v_quantities.is_bound());
        for (GEO::index_t v = 0, v_end = this->control_grid->control_nodes_nb(); v < v_end; ++v) {
            for (GEO::index_t d = 0; d < dim; ++d)
                v_quantities[dim*v+d] = (d+1)*GEO::Numeric::random_float32();
        }
        this->save_control_nodes(get_current_test_name()+"_nodes.geogram");
        this->save_high_order_mesh_facets(get_current_test_name()+"_facets.geogram");
    }

    TYPED_TEST(SingleQuadControlGridTest, facet_normal) {
        if constexpr (constexpr GEO::index_t DIM = TypeParam::value;
            DIM == 3
            ) {
            constexpr GEO::index_t f = 0;
            {
                auto& p = this->control_grid->control_node(this->control_grid->facet_edge_nd(f, 3, 0));
                for (GEO::index_t d = 0; d < DIM; ++d)
                    p[d] += 0.1*GEO::Numeric::random_float32();
                p[2] += 0.2;
            }
            {
                auto& p = this->control_grid->control_node(this->control_grid->facet_edge_nd(f, 1, 2));
                for (GEO::index_t d = 0; d < DIM; ++d)
                    p[d] += 0.1*GEO::Numeric::random_float32();
                p[2] -= 0.2;
            }

            constexpr GEO::index_t points_nb = 100;
            GEO::Mesh mesh_out;
            GEO::index_t new_v = mesh_out.vertices.create_vertices(2*points_nb);
            GEO::index_t new_e = mesh_out.edges.create_edges(points_nb);
            for (GEO::index_t i = 0; i < points_nb; ++i) {
                const GEO::vec2 uv(GEO::Numeric::random_float32(), GEO::Numeric::random_float32());
                const auto position = this->control_grid->compute_facet_uv_position(f, uv);
                const auto normal = this->control_grid->compute_facet_uv_normal(f, uv);

                mesh_out.vertices.point(new_v) = position;
                mesh_out.vertices.point(new_v+1) = position + 0.2*normal;
                mesh_out.edges.set_vertex(new_e, 0, new_v);
                mesh_out.edges.set_vertex(new_e, 1, new_v+1);
                new_v += 2;
                ++new_e;
            }

            this->control_grid->append_discretized_high_order_facets(mesh_out, 30);
            mesh_out.save(get_current_test_name()+".geogram");
        }
    }

    TYPED_TEST(SingleQuadControlGridTest, dudv) {
        constexpr GEO::index_t DIM = TypeParam::value;

        constexpr GEO::index_t f = 0;
        {
            auto& p = this->control_grid->control_node(this->control_grid->facet_edge_nd(f, 1, 0));
            for (GEO::index_t d = 0; d < DIM; ++d)
                p[d] += 0.1*GEO::Numeric::random_float32();
            if constexpr (DIM == 3)
                p[2] += 0.2;
        }
        {
            auto& p = this->control_grid->control_node(this->control_grid->facet_edge_nd(f, 3, 2));
            for (GEO::index_t d = 0; d < DIM; ++d)
                p[d] += 0.1*GEO::Numeric::random_float32();
            if constexpr (DIM == 3)
                p[2] -= 0.2;
        }

        constexpr GEO::index_t resolution = 10;
        GEO::Mesh mesh_out(DIM);
        GEO::Attribute<GEO::index_t> mesh_out_e_axis(mesh_out.edges.attributes(), "axis");
        GEO::index_t new_v = mesh_out.vertices.create_vertices(3*std::pow(resolution+1, 2));
        GEO::index_t new_e = mesh_out.edges.create_edges(2*std::pow(resolution+1, 2));
        for (GEO::index_t i = 0; i < resolution+1; ++i) {
            const double ri = static_cast<double>(i)/resolution;
            for (GEO::index_t j = 0; j < resolution+1; ++j) {
                const double rj = static_cast<double>(j)/resolution;

                const GEO::vec2 uv(ri, rj);
                GEO::vecng<DIM, double> du, dv;
                std::vector<double> Bu, Bv, dBu, dBv;
                this->control_grid->compute_facet_uv_dudv(f, uv, du, dv, Bu, Bv, dBu, dBv);

                const auto p = this->control_grid->compute_facet_uv_position(f, uv);
                constexpr double length = 0.05;
                mesh_out.vertices.point<DIM>(new_v) = p;
                mesh_out.vertices.point<DIM>(new_v+1) = p + length*du;
                mesh_out.vertices.point<DIM>(new_v+2) = p + length*dv;
                mesh_out.edges.set_vertex(new_e, 0, new_v);
                mesh_out.edges.set_vertex(new_e, 1, new_v+1);
                mesh_out.edges.set_vertex(new_e+1, 0, new_v);
                mesh_out.edges.set_vertex(new_e+1, 1, new_v+2);
                mesh_out_e_axis[new_e] = 0;
                mesh_out_e_axis[new_e+1] = 1;

                new_v += 3;
                new_e += 2;
            }
        }

        this->control_grid->append_discretized_high_order_facets(mesh_out, 30);
        mesh_out.save(get_current_test_name()+".geogram");
    }

    template <typename DimType>
    class TwoQuadControlGridTest : public QuadControlGridTest<DimType::value> {
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

    TYPED_TEST_SUITE(TwoQuadControlGridTest, DimTypes);

    TYPED_TEST(TwoQuadControlGridTest, regular) {
        this->save_control_nodes(get_current_test_name()+"_nodes.geogram");
        this->save_high_order_mesh_facets(get_current_test_name()+"_facets.geogram");
    }

    TYPED_TEST(TwoQuadControlGridTest, random) {
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
        this->save_control_nodes(get_current_test_name()+"_nodes.geogram");
        this->save_high_order_mesh_facets(get_current_test_name()+"_facets.geogram");
    }
}