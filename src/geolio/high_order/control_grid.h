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
    template<GEO::index_t DIM>
    class ControlGrid {
    public:
        /**
         * @brief Construct a control grid from a reference mesh and polynomial order.
         * @param[in] mesh Input mesh used as topology/geometry reference.
         * @param[in] order Polynomial order of the high-order representation.
         */
        ControlGrid(const GEO::Mesh& mesh, const GEO::index_t order)
            : attribute_name_(generate_random_string(22)),
            mesh_(mesh),
            order_(order),
            CONTROL_POINTS_NB_PER_EDGE_(order+1),
            CONTROL_POINTS_NB_PER_FACET_((order+1)*(order+1)),
            CONTROL_POINTS_NB_PER_CELL_((order+1)*(order+1)*(order+1)),
            INTERNAL_CONTROL_POINTS_NB_PER_EDGE_(order-1),
            INTERNAL_CONTROL_POINTS_NB_PER_FACET_((order-1)*(order-1)),
            INTERNAL_CONTROL_POINTS_NB_PER_CELL_((order-1)*(order-1)*(order-1))
        {
            assert(mesh.vertices.dimension() == DIM);
            assert(order_ > 0);

            initialize_node_positions_1D();
            control_nodes_.vertices.set_dimension(DIM);
        }

        /**
         * @brief Virtual destructor.
         */
        virtual ~ControlGrid() = default;

        /**
         * @brief Create or update the control-node quantity attribute.
         *
         * When @p dim is zero, the existing quantity attribute is removed.
         * For positive dimensions, a vector attribute with the requested
         * component count is created or recreated on the control-node mesh if
         * it does not exist or its dimension differs from @p dim.
         * @param[in] dim Number of quantity components stored per control node.
         *                Use 0 to disable and destroy the attribute.
         */
        void create_control_node_quantities(const GEO::index_t dim) {
            if (dim == 0) {
                if (control_nodes_quantities_.is_bound())
                    control_nodes_quantities_.destroy();
            }
            else {
                if (!control_nodes_quantities_.is_bound() ||
                    control_nodes_quantities_.dimension() != dim
                    ) { // need to re-create
                    if (control_nodes_quantities_.is_bound())
                       control_nodes_quantities_.destroy();
                    control_nodes_quantities_.create_vector_attribute(
                       control_nodes_.vertices.attributes(),
                       attribute_name_+":quantities",
                       dim);
                }
            }
        }

        /**
         * @brief Access the mutable control-node quantity attribute.
         */
        auto& control_nodes_quantities() { return control_nodes_quantities_; }

        /**
         * @brief Query the dimension of the control-node quantity attribute.
         *
         * Returns 0 when no quantity attribute is currently bound.
         * @return Number of scalar components per control node in the quantity
         *         attribute, or 0 if the attribute is not created.
         */
        auto control_node_quantities_dimension() const {
            return control_nodes_quantities_.is_bound() ? control_nodes_quantities_.dimension() : 0;
        }

        /**
         * @brief Access the reference mesh.
         * @return Const reference to the underlying mesh.
         */
        const auto& mesh() const { return mesh_; }

        /**
         * @brief Get the polynomial order.
         * @return Polynomial order used by this control grid.
         */
        [[nodiscard]] GEO::index_t order() const { return order_; }

        /**
         * Get the number of control points per edge.
         * @return number of control points on each edge of the grid
         */
        [[nodiscard]] auto control_nodes_nb_per_edge() const { return CONTROL_POINTS_NB_PER_EDGE_; }

        /**
         * Get the number of control points per facet.
         * @return number of control points on each facet of the grid
         */
        [[nodiscard]] auto control_nodes_nb_per_facet() const { return CONTROL_POINTS_NB_PER_FACET_; }

        /**
         * Get the number of control points per cell.
         * @return number of control points in each cell of the grid
         */
        [[nodiscard]] auto control_nodes_nb_per_cell() const { return CONTROL_POINTS_NB_PER_CELL_; }

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

        /**
         * @brief Get the total number of control nodes.
         * @return Number of vertices stored in the control-node mesh.
         */
        [[nodiscard]] GEO::index_t control_nodes_nb() const { return control_nodes_.vertices.nb(); }

        /**
         * @brief Access a mutable control node by global index.
         * @param[in] v Control-node index.
         * @return Mutable reference to the control-node position.
         */
        GEO::vecng<DIM, double>& control_node(const GEO::index_t v) {
            assert(v < control_nodes_nb());
            return control_nodes_.vertices.point<DIM>(v);
        }

        /**
         * @brief Access a const control node by global index.
         * @param[in] v Control-node index.
         * @return Const reference to the control-node position.
         */
        [[nodiscard]] const GEO::vecng<DIM, double>& control_node(const GEO::index_t v) const {
            assert(v < control_nodes_nb());
            return control_nodes_.vertices.point<DIM>(v);
        }

        /**
         * @brief Access the mutable contiguous control-node coordinate array.
         * @return Mutable view/proxy over all control-node coordinates.
         */
        const auto& control_nodes() {
            return control_nodes_.vertices.points<DIM>();
        }

        /**
         * @brief Access the const contiguous control-node coordinate array.
         * @return Const view/proxy over all control-node coordinates.
         */
        [[nodiscard]] auto control_nodes() const {
            return control_nodes_.vertices.points<DIM>();
        }

        /**
         * @brief Get a raw mutable pointer to one control-node coordinate tuple.
         * @param[in] v Control-node index.
         * @return Pointer to the first component of the indexed node position.
         */
        double* control_node_ptr(const GEO::index_t v) {
            return control_nodes_.vertices.point_ptr(v);
        }

    protected:
        const std::string attribute_name_; // unique id

        const GEO::Mesh& mesh_;

        /* ========================================================================================================= */

        const GEO::index_t order_;
        const GEO::index_t CONTROL_POINTS_NB_PER_EDGE_;
        const GEO::index_t CONTROL_POINTS_NB_PER_FACET_;
        const GEO::index_t CONTROL_POINTS_NB_PER_CELL_;
        const GEO::index_t INTERNAL_CONTROL_POINTS_NB_PER_EDGE_;
        const GEO::index_t INTERNAL_CONTROL_POINTS_NB_PER_FACET_;
        const GEO::index_t INTERNAL_CONTROL_POINTS_NB_PER_CELL_;
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

        BasisFunctionType basis_function_type_ = BasisFunctionType::LAGRANGE;

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
        virtual void initialize_control_nodes() = 0;
        GEO::Mesh control_nodes_;
        GEO::Attribute<double> control_nodes_quantities_; // [dim*nd+i] -> node's ith quantities
        std::vector<GEO::index_t> element_control_nodes_;
    };
}

#endif //GEOLIO_CONTROL_GRID_H
