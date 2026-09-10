//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/2.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <gtest/gtest.h>
#include <geolio/high_order/node_positions.h>
#include <geolio/high_order/control_grid.h>

namespace
{
    const std::vector<std::tuple<geolio::ControlGrid<3>::NodesType, GEO::index_t, std::vector<double>>> NODE_POSITIONS_TEST_DATA = {
        {geolio::ControlGrid<3>::NodesType::EQUALLY_SPACED_NODES, 1, {0, 1}},
        {geolio::ControlGrid<3>::NodesType::EQUALLY_SPACED_NODES, 2, {0, 0.5, 1}},
        {geolio::ControlGrid<3>::NodesType::EQUALLY_SPACED_NODES, 3, {0.0, 0.3333333333, 0.6666666667, 1.0}},
        {geolio::ControlGrid<3>::NodesType::EQUALLY_SPACED_NODES, 4, {0.0, 0.25, 0.5, 0.75, 1.0}},
        {geolio::ControlGrid<3>::NodesType::EQUALLY_SPACED_NODES, 5, {0.0, 0.2, 0.4, 0.6, 0.8, 1.0}},
        {geolio::ControlGrid<3>::NodesType::EQUALLY_SPACED_NODES, 6, {0.0, 0.1666666667, 0.3333333333, 0.5, 0.6666666667, 0.8333333333, 1.0}},
        {geolio::ControlGrid<3>::NodesType::CHEBYSHEV_GAUSS, 1, {0.1464466094, 0.8535533906}},
        {geolio::ControlGrid<3>::NodesType::CHEBYSHEV_GAUSS, 2, {0.0669872981, 0.5, 0.9330127019}},
        {geolio::ControlGrid<3>::NodesType::CHEBYSHEV_GAUSS, 3, {0.0380602337, 0.3086582838, 0.6913417162, 0.9619397663}},
        {geolio::ControlGrid<3>::NodesType::CHEBYSHEV_GAUSS, 4, {0.0244717419, 0.2061073739, 0.5, 0.7938926261, 0.9755282581}},
        {geolio::ControlGrid<3>::NodesType::CHEBYSHEV_GAUSS, 5, {0.0170370869, 0.1464466094, 0.3705904774, 0.6294095226, 0.8535533906, 0.9829629131}},
        {geolio::ControlGrid<3>::NodesType::CHEBYSHEV_GAUSS, 6, {0.0125360439, 0.1090842588, 0.2830581304, 0.5, 0.7169418695, 0.8909157411, 0.987463956}},
        {geolio::ControlGrid<3>::NodesType::CHEBYSHEV_GAUSS_LOBATTO, 1, {0.0, 1.0}},
        {geolio::ControlGrid<3>::NodesType::CHEBYSHEV_GAUSS_LOBATTO, 2, {0.0, 0.5, 1.0}},
        {geolio::ControlGrid<3>::NodesType::CHEBYSHEV_GAUSS_LOBATTO, 3, {0.0, 0.25, 0.75, 1.0}},
        {geolio::ControlGrid<3>::NodesType::CHEBYSHEV_GAUSS_LOBATTO, 4, {0.0, 0.1464466094, 0.5, 0.8535533906, 1.0}},
        {geolio::ControlGrid<3>::NodesType::CHEBYSHEV_GAUSS_LOBATTO, 5, {0.0, 0.0954915028, 0.3454915028, 0.6545084972, 0.9045084972, 1.0}},
        {geolio::ControlGrid<3>::NodesType::CHEBYSHEV_GAUSS_LOBATTO, 6, {0.0, 0.0669872981, 0.25, 0.5, 0.75, 0.9330127019, 1.0}},
        {geolio::ControlGrid<3>::NodesType::LEGENDRE_GAUSS_LOBATTO, 1, {0.0, 1.0}},
        {geolio::ControlGrid<3>::NodesType::LEGENDRE_GAUSS_LOBATTO, 2, {0.0, 0.5, 1.0}},
        {geolio::ControlGrid<3>::NodesType::LEGENDRE_GAUSS_LOBATTO, 3, {0.0, 0.2763932022, 0.7236067977, 1.0}},
        {geolio::ControlGrid<3>::NodesType::LEGENDRE_GAUSS_LOBATTO, 4, {0.0, 0.1726731646, 0.5, 0.8273268353, 1.0}},
        {geolio::ControlGrid<3>::NodesType::LEGENDRE_GAUSS_LOBATTO, 5, {0.0, 0.1174723380, 0.3573842417, 0.6426157582, 0.8825276619, 1.0}},
        {geolio::ControlGrid<3>::NodesType::LEGENDRE_GAUSS_LOBATTO, 6, {0.0, 0.0848880518, 0.2655756032, 0.5, 0.7344243967, 0.9151119481, 1.0}}
    };
}

namespace geolio::test
{
    class NodePositionsTest : public testing::TestWithParam<std::tuple<ControlGrid<3>::NodesType, GEO::index_t, std::vector<double>>> {
    public:
        void compute(
            const ControlGrid<3>::NodesType nodes_type,
            const GEO::index_t order
            ) {
            switch (nodes_type) {
                case ControlGrid<3>::NodesType::EQUALLY_SPACED_NODES:
                    compute_equally_spaced_nodes(order, node_positions_);
                    break;
                case ControlGrid<3>::NodesType::CHEBYSHEV_GAUSS:
                    compute_Chebyshev_Gauss_nodes(order, node_positions_);
                    break;
                case ControlGrid<3>::NodesType::CHEBYSHEV_GAUSS_LOBATTO:
                    compute_Chebyshev_Gauss_Lobatto_nodes(order, node_positions_);
                    break;
                case ControlGrid<3>::NodesType::LEGENDRE_GAUSS_LOBATTO:
                    compute_Legendre_Gauss_Lobatto_nodes(order, node_positions_);
                    break;
                default:
                    FAIL();
            }

            EXPECT_EQ(node_positions_.size(), order+1);
        }

        void test_monotonicity() const {
            for (GEO::index_t i = 0, i_end = node_positions_.size()-1; i < i_end; ++i)
                EXPECT_GT(node_positions_[i+1], node_positions_[i]);
        }

        void test_symmetry() const {
            for (GEO::index_t i = 0, i_end = node_positions_.size()/2; i < i_end; ++i)
                EXPECT_NEAR(node_positions_[i], node_positions_[node_positions_.size()-i], 1e-20);
        }

        void test_gt(
            const std::vector<double>& gt
            ) const {
            ASSERT_EQ(node_positions_.size(), gt.size());
            for (GEO::index_t i = 0, i_end = gt.size(); i < i_end; ++i)
                EXPECT_NEAR(node_positions_[i], gt[i], 1e-8);
        }

        std::vector<double> node_positions_;
    };

    TEST_P(NodePositionsTest, each) {
        auto [nodes_type, order, node_positions_gt] = GetParam();
        compute(nodes_type, order);
        test_gt(node_positions_gt);
    }

    INSTANTIATE_TEST_SUITE_P(SomeCases, NodePositionsTest, testing::ValuesIn(NODE_POSITIONS_TEST_DATA));
}