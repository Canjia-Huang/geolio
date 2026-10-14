//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/2.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_CONTROL_GRID_H
#define GEOLIO_CONTROL_GRID_H
#include <cassert>
#include <geogram/mesh/mesh.h>
#include "node_positions.h"
#include "geolio/common/utils.h"

namespace geolio
{
    class QuadControlGrid;
    class HexControlGrid;

    class ControlNodes {
    public:
        ControlNodes(const GEO::index_t dim, const GEO::index_t order)
            : CONTROL_POINTS_NB_PER_EDGE_(order+1),
              CONTROL_POINTS_NB_PER_FACET_((order+1)*(order+1)),
              CONTROL_POINTS_NB_PER_CELL_((order+1)*(order+1)*(order+1)),
              INTERNAL_CONTROL_POINTS_NB_PER_EDGE_(order-1),
              INTERNAL_CONTROL_POINTS_NB_PER_FACET_((order-1)*(order-1)),
              INTERNAL_CONTROL_POINTS_NB_PER_CELL_((order-1)*(order-1)*(order-1))
        {
            mesh_.vertices.set_dimension(dim);
        }

        GEO::index_t nb_per_edge() const { return CONTROL_POINTS_NB_PER_EDGE_; }

        GEO::index_t nb_per_facet() const { return CONTROL_POINTS_NB_PER_FACET_; }

        GEO::index_t nb_per_cell() const { return CONTROL_POINTS_NB_PER_CELL_; }

        GEO::index_t internal_nb_per_edge() const { return INTERNAL_CONTROL_POINTS_NB_PER_EDGE_; }

        GEO::index_t internal_nb_per_facet() const { return INTERNAL_CONTROL_POINTS_NB_PER_FACET_; }

        GEO::index_t internal_nb_per_cell() const { return INTERNAL_CONTROL_POINTS_NB_PER_CELL_; }

        GEO::index_t nb() const { return mesh_.vertices.nb(); }

        GEO::index_t dimension() const {
            return mesh_.vertices.dimension();
        }

        void set_dimension(const GEO::index_t dim) {
            mesh_.vertices.set_dimension(dim);
        }

        template<GEO::index_t DIM>
        GEO::vecng<DIM, double>& point(const GEO::index_t v) {
            assert(DIM <= mesh_.vertices.dimension());
            assert(v < nb());
            return mesh_.vertices.point<DIM>(v);
        }

        template<GEO::index_t DIM>
        const GEO::vecng<DIM, double>& point(const GEO::index_t v) const {
            assert(DIM <= mesh_.vertices.dimension());
            assert(v < nb());
            return mesh_.vertices.point<DIM>(v);
        }

        template<GEO::index_t DIM>
        auto& points() {
            assert(DIM <= mesh_.vertices.dimension());
            return mesh_.vertices.points<DIM>();
        }

        template<GEO::index_t DIM>
        const auto& points() const {
            assert(DIM <= mesh_.vertices.dimension());
            return mesh_.vertices.points<DIM>();
        }

        double* point_ptr(const GEO::index_t v) {
            assert(v < nb());
            return mesh_.vertices.point_ptr(v);
        }

        const double* point_ptr(const GEO::index_t v) const {
            assert(v < nb());
            return mesh_.vertices.point_ptr(v);
        }

        const auto& mesh() const { return mesh_; }

    protected:
        const GEO::index_t CONTROL_POINTS_NB_PER_EDGE_;
        const GEO::index_t CONTROL_POINTS_NB_PER_FACET_;
        const GEO::index_t CONTROL_POINTS_NB_PER_CELL_;
        const GEO::index_t INTERNAL_CONTROL_POINTS_NB_PER_EDGE_;
        const GEO::index_t INTERNAL_CONTROL_POINTS_NB_PER_FACET_;
        const GEO::index_t INTERNAL_CONTROL_POINTS_NB_PER_CELL_;

        GEO::Mesh mesh_;

        friend class QuadControlGrid;
        friend class HexControlGrid;
    };

    class ControlGrid {
    public:
        /**
         * @brief Construct a control grid with the specified mesh vertex dimension and polynomial order.
         * @param[in] mesh_v_dim Dimension of each mesh vertex.
         * @param[in] order Polynomial order of the high-order representation.
         */
        ControlGrid(const GEO::index_t mesh_v_dim, const GEO::index_t order)
            : control_nodes(mesh_v_dim, order),
              attribute_id_(generate_random_string(22)),
              mesh_v_dim_(mesh_v_dim),
              order_(order)
        {
            initialize_node_positions_1D();
        }

        /**
         * @brief Virtual destructor.
         */
        virtual ~ControlGrid() = default;

        GEO::index_t order() const { return order_; }

        GEO::index_t mesh_vertices_dimension() const { return mesh_v_dim_; }

        enum class BasisFunctionType {
            BEZIER, // not support yet
            LAGRANGE
        };

        /**
         * @brief Get the basis-function family used by this control grid.
         * @return Current basis-function type.
         */
        [[nodiscard]] BasisFunctionType basis_function_type() const { return basis_function_type_; }

        enum class NodesType {
            EQUALLY_SPACED_NODES,
            CHEBYSHEV_GAUSS,
            CHEBYSHEV_GAUSS_LOBATTO,
            LEGENDRE_GAUSS_LOBATTO
        };

        /**
         * @brief Get the 1D node distribution type.
         * @return Current control-node distribution policy.
         */
        [[nodiscard]] NodesType nodes_type() const { return nodes_type_; }

        /**
         * @brief Set the 1D node distribution type and refresh dependent data.
         * @param[in] nodes_type New control-node distribution policy.
         */
        void set_nodes_type(const NodesType nodes_type) {
            if (nodes_type != nodes_type_)
                initialize_node_positions_1D();
        }

        /**
         * Get 1D parametric coordinates of control nodes.
         * @return const reference to node positions in [0,1]
         */
        [[nodiscard]] const auto& node_positions_1D() const { return node_positions_1D_; }

        ControlNodes control_nodes;

    protected:
        const std::string attribute_id_; // unique id

        const GEO::index_t mesh_v_dim_;
        const GEO::index_t order_;

        /* ========================================================================================================= */

        /* Local index */
        /**
         * @brief Initialize local indexing/layout rules for volume control nodes.
         */
        virtual void initialize_nodes_arrangement() = 0;
        std::vector<GEO::index_t> ELEMENT_VERTEX_CONTROL_POINTS_BEGIN_IDX_{};
        std::vector<GEO::index_t> ELEMENT_EDGE_CONTROL_POINTS_BEGIN_IDX_{};
        std::vector<int>          ELEMENT_EDGE_CONTROL_POINTS_NEXT_IDX_STEP_{};
        std::vector<GEO::index_t> ELEMENT_EDGE_INTERNAL_CONTROL_POINTS_BEGIN_IDX_{};
        std::vector<int>          ELEMENT_EDGE_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP_{};
        std::vector<GEO::index_t> ELEMENT_FACET_CONTROL_POINTS_BEGIN_IDX_{};
        std::vector<int>          ELEMENT_FACET_CONTROL_POINTS_NEXT_IDX_STEP0_{};
        std::vector<int>          ELEMENT_FACET_CONTROL_POINTS_NEXT_IDX_STEP1_{};
        std::vector<GEO::index_t> ELEMENT_FACET_INTERNAL_CONTROL_POINTS_BEGIN_IDX_{};
        std::vector<int>          ELEMENT_FACET_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP0_{};
        std::vector<int>          ELEMENT_FACET_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP1_{};
        GEO::index_t              ELEMENT_CONTROL_POINTS_BEGIN_IDX_{};
        int                       ELEMENT_CONTROL_POINTS_NEXT_IDX_STEP0_{};
        int                       ELEMENT_CONTROL_POINTS_NEXT_IDX_STEP1_{};
        int                       ELEMENT_CONTROL_POINTS_NEXT_IDX_STEP2_{};
        GEO::index_t              ELEMENT_INTERNAL_CONTROL_POINTS_BEGIN_IDX_{};
        int                       ELEMENT_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP0_{};
        int                       ELEMENT_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP1_{};
        int                       ELEMENT_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP2_{};

        /* ========================================================================================================= */

        const BasisFunctionType basis_function_type_ = BasisFunctionType::LAGRANGE;

        /* ========================================================================================================= */

        /**
         * @brief Initialize the 1D parametric node positions according to `nodes_type_`.
         */
        void initialize_node_positions_1D() {
            switch (nodes_type_) {
                case NodesType::EQUALLY_SPACED_NODES:
                    compute_equally_spaced_nodes(order_, node_positions_1D_);
                    break;
                case NodesType::CHEBYSHEV_GAUSS:
                    compute_Chebyshev_Gauss_nodes(order_, node_positions_1D_);
                    break;
                case NodesType::CHEBYSHEV_GAUSS_LOBATTO:
                    compute_Chebyshev_Gauss_Lobatto_nodes(order_, node_positions_1D_);
                    break;
                case NodesType::LEGENDRE_GAUSS_LOBATTO:
                    compute_Legendre_Gauss_Lobatto_nodes(order_, node_positions_1D_);
                    break;
                default:
                    assert(0);
            }
        }
        NodesType nodes_type_ = NodesType::EQUALLY_SPACED_NODES;
        std::vector<double> node_positions_1D_; // 1D distribution of control nodes

        /* ========================================================================================================= */

        /**
         * @brief Build control-node connectivity/geometry for the derived grid type.
         */
        virtual void initialize_control_nodes(const GEO::Mesh& mesh) = 0;

        GEO::Attribute<GEO::index_t> element_control_nodes_; // size == elements nb, dim == nodes nb per element
    };
}

#endif //GEOLIO_CONTROL_GRID_H
