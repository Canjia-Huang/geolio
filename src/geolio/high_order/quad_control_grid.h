//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/4.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef HOSM_QUAD_CONTROL_GRID_H
#define HOSM_QUAD_CONTROL_GRID_H
#include "surf_control_grid.h"
#include <Eigen/Dense>

namespace geolio
{
    /**
    * @brief Projects a 2D parametric coordinate to a 1D parameter on a quadrilateral edge.
    *
    * Projects a point (u,v) from the unit square parametric domain [0,1]^2 to
    * a specified quadrilateral edge, returning the 1D parameter t ∈ [0,1] on that edge.
    *
    * @param uv The 2D parametric coordinate in the unit cube (u, v) ∈ [0,1]^2
    * @param le  The local edge index of the quadrilateral, range [0, 4] (ev0 -> ev1 -> ev2 -> ev3, CCW)
    *
    * @return The projected 1D parameter t ∈ [0,1], representing the position on the edge
    *         (t=0 corresponds to the start of the edge, t=1 to the end)
    */
    inline double project_uv_to_quad_le_t(const GEO::vec2& uv, const GEO::index_t le) {
        assert(le < 4);
        switch (le) {
            case 0: return uv.x;
            case 1: return uv.y;
            case 2: return 1-uv.x;
            case 3: return 1-uv.y;
            default: return -1;
        }
    }

    /**
     * @brief Projects a quadrilateral vertex to 2D parametric coordinates.
     *
     * Given a local vertex index in a quadrilateral (0-3), returns the corresponding
     * 2D parametric coordinate (u,v) in the unit cube domain [0,1]^2 for that vertex.
     * Vertex indexing follows standard quadrilateral topology.
     *
     * @param[in] lv Local vertex index in the quadrilateral, range [0, 3].
     *
     * @return The 2D parametric coordinate (u,v) ∈ {0,1}^2 corresponding to the vertex.
     */
    inline GEO::vec2 project_quad_lv_to_uv(const GEO::index_t lv) {
        assert(lv < 4);
        GEO::vec2 uvw;
        switch (lv) {
            case 0: uvw = GEO::vec2(0, 0); break;
            case 1: uvw = GEO::vec2(1, 0); break;
            case 2: uvw = GEO::vec2(1, 1); break;
            case 3: uvw = GEO::vec2(0, 1); break;
            default: uvw = GEO::vec2(-1, -1);
        }
        return uvw;
    }

    /**
     * @brief Projects a 1D parameter on a quadrilateral edge back to 2D parametric coords.
     *
     * Given a scalar parameter t in [0,1] defined along the local quadrilateral edge
     * identified by `le`, return the corresponding 2D parametric coordinate (u,v)
     * in the unit cube domain [0,1]^2 that lies on that edge.
     *
     * @param[in] t  The 1D parameter along the edge (t=0 -> edge start, t=1 -> edge end).
     * @param[in] le Local edge index in the quadrilateral (0..3).
     * @return The 2D parametric coordinate (u,v) in [0,1]^2 corresponding to the edge
     *         parameter.
     */
    inline GEO::vec2 project_quad_le_t_to_uv(const double t, const GEO::index_t le) {
        assert(le < 4);
        GEO::vec2 uv;
        switch (le) {
            case 0: uv = GEO::vec2(t, 0); break;
            case 1: uv = GEO::vec2(1, t); break;
            case 2: uv = GEO::vec2(1-t, 1); break;
            case 3: uv = GEO::vec2(0, 1-t); break;
            default: uv = GEO::vec2(-1, -1);;
        }
        return uv;
    }

    template<GEO::index_t DIM>
    class QuadControlGrid : public SurfaceControlGrid<DIM> {
    public:
        QuadControlGrid(const GEO::Mesh& mesh, GEO::index_t order);

        /**
         * @brief Map a parametric coordinate inside a facet to physical space.
         *
         * Evaluates the high-order facet mapping at the given parametric coordinate
         * `uv` ∈ [0,1]^2 and returns the corresponding physical-space point. The
         * mapping is built from the facet's control-point positions stored in the
         * internal control grid.
         *
         * @param[in] f Index of the facet (0..mesh_.facets.nb()-1).
         * @param[in] uv Parametric coordinate in the facet-local domain [0,1]^2.
         *
         * @return Physical-space position corresponding to the input parametric point.
         *
         * @note Preconditions: `f` must be a valid facet index and the components of
         *       `uv` are expected to be in [0,1] for meaningful results. The
         *       function uses the control-point coordinates currently stored in
         *       the internal control grid to compute the mapped position.
         */
        [[nodiscard]] GEO::vecng<DIM, double> compute_facet_uv_position(GEO::index_t f, const GEO::vec2& uv) const;

        /**
         * @brief Map a parametric coordinate to physical space using provided control positions.
         *
         * Variant of `compute_facet_uv_position` that evaluates the facet mapping
         * using an externally supplied flat array of control-node positions. This
         * is useful for evaluating hypothetical configurations (e.g., during
         * optimization) without modifying the internal control grid.
         *
         * @param[in] f Index of the facet (0..mesh_.facets.nb()-1).
         * @param[in] uv Parametric coordinate in the facet-local domain [0,1]^2.
         * @param[in] cur_control_nodes_ptr Pointer to a flat array of control-node
         *            coordinates in the same layout used by the optimizer
         *            (`[x0,y0,z0,x1,y1,z1,...]` for all control vertices). The
         *            array must contain at least `3 * control_points_nb()` entries.
         *
         * @return Physical-space position corresponding to the input parametric point
         *         evaluated with the provided control-node positions.
         *
         * @note The function does not take ownership of `cur_control_nodes_ptr` and
         *       treats it as read-only. Caller must ensure the buffer is valid.
         */
        [[nodiscard]] GEO::vecng<DIM, double> compute_facet_uv_position(GEO::index_t f, const GEO::vec2& uv, const double* cur_control_nodes_ptr) const;

        /**
         * Evaluate the (non-unit) physical normal of a high-order facet.
         *
         * The normal is computed from the tangent vectors obtained by differentiating
         * the facet mapping with respect to its local parametric coordinates.
         *
         * @param[in] f Facet index, 0,1,...,mesh_.facets.nb()-1.
         * @param[in] uv Facet-local parameter point `(u, v)` in [0, 1]^2.
         *          - u: facet-local parameter along the first axis, typically in [0,1]
         *          - v: facet-local parameter along the second axis, typically in [0,1]
         * @return Outward facet normal vector at the given parameter point in physical space.
         * @pre `f < mesh_.facets.nb()`.
         * @pre `uv.x` and `uv.y` are in [0, 1].
         * @note The returned vector is not normalized; its magnitude equals the local area scaling.
         */
        [[nodiscard]] GEO::vec3 compute_facet_uv_normal(GEO::index_t f, const GEO::vec2& uv) const requires (DIM == 3);

        /**
         * Evaluate the first-order parametric derivatives of the facet mapping at a parameter point.
         *
         * This helper computes the physical-space tangent vectors with respect to the two parametric
         * directions, together with the corresponding 1D basis values and basis derivatives used in the
         * tensor-product evaluation.
         *
         * @param[in] f Facet index, 0,1,...,mesh_.facets.nb()-1.
         * @param[in] uv Parameter point in the facet parameter domain [0, 1]^2.
         * @param[out] du Output tangent vector for partial derivative with respect to u.
         * @param[out] dv Output tangent vector for partial derivative with respect to v.
         * @param[out] Bu Output buffer that receives the 1D basis values in the u direction.
         * @param[out] Bv Output buffer that receives the 1D basis values in the v direction.
         * @param[out] dBu Output buffer that receives the 1D basis derivatives in the u direction.
         * @param[out] dBv Output buffer that receives the 1D basis derivatives in the v direction.
         */
        void compute_facet_uv_dudv(
            GEO::index_t f, const GEO::vec2& uv,
            GEO::vecng<DIM, double>& du, GEO::vecng<DIM, double>& dv,
            std::vector<double>& Bu, std::vector<double>& Bv,
            std::vector<double>& dBu, std::vector<double>& dBv) const;

        /**
         * Evaluate one scalar physical quantity at a parameter point in a facet.
         * @param[in] f facet index, 0,1,...,quad_mesh.facets.nb()-1
         * @param[in] uv parameter point in the facet parameter domain [0, 1]^2
         * @param[in] d physical quantity component index, 0,1,...,PHYS_DIM_-1
         * @return interpolated physical quantity value of component \p d
         */
        [[nodiscard]] double compute_facet_uv_quantity(GEO::index_t f, const GEO::vec2& uv, GEO::index_t d) const;

        /**
         * Evaluate all physical quantity components at a parameter point in a facet.
         * @param[in] f cell index, 0,1,...,quad_mesh.facets.nb()-1
         * @param[in] uv parameter point in the facet parameter domain [0, 1]^2
         * @param[out] q output buffer with length at least PHYS_DIM_; receives interpolated values
         */
        void compute_facet_uv_quantities(GEO::index_t f, const GEO::vec2& uv, double* q) const;

        /**
         * @brief Compute a unit-length reference normal for a 3D quadrilateral facet.
         *
         * The reference normal is computed from the four corner control-node positions
         * using the cross product of the two diagonal directions:
         *   n_ref = normalize( (p2 - p0) x (p3 - p1) )
         *
         * It provides a stable, consistently-oriented unit normal associated with the facet
         * (orientation follows the facet's vertex ordering). The reference normal is used
         * to determine the sign of the Jacobian (via dot(cross(du,dv), n_ref)) and for
         * other orientation-dependent computations.
         *
         * @param[in] f Facet index (0..mesh_.facets.nb()-1)
         * @return Unit-length reference normal vector in physical space.
         * @pre DIM == 3
         * @pre f < mesh_.facets.nb()
         */
        [[nodiscard]] GEO::vec3 compute_facet_reference_normal(GEO::index_t f) const requires (DIM == 3);

        enum class MeasureType {
            DET_JACOBIAN,         // Signed Jacobian determinant; non-positive values indicate inversion or degeneration.
            ABSOLUTE_SQ_AREA,     // 0.5 * detJ^2
            SCALED_JACOBIAN,      // Skew measure / normalized Jacobian; 1.0 is ideal, and non-positive values indicate collapse or inversion.
            INVERSE_MEAN_RATIO,   // Shape-quality metric combining angle and aspect-ratio distortion; 1.0 is best and 0 indicates degeneration.
            MIPS                  // Minimizes shear and anisotropic stretching; 1.0 is best and the value grows toward infinity near degeneration.
        };

        /**
         * Evaluate a geometric quality metric at a point in the facet parameter domain.
         *
         * @param[in] f facet index, 0,1,...,quad_mesh.facets.nb()-1
         * @param[in] uv parameter point in the facet parameter domain [0, 1]^3
         * @param[in] quality_type quality metric to evaluate:
         *                       - `QualityType::JACOBIAN`: signed Jacobian determinant, [-inf, inf]
         *                       - `QualityType::SCALED_JACOBIAN`: normalized Jacobian / skew measure, [-1, 1], best: 1
         *                       - `QualityType::INVERSE_MEAN_RATIO`: shape quality measure based on Jacobian and metric lengths, [0, 1], best: 1
         *                       - `QualityType::MIPS`: penalizes shear and anisotropic stretching, [1, inf], best: 1
         * @return the requested quality value at the given parameter point
         */
        [[nodiscard]] double compute_facet_uv_measure(GEO::index_t f, const GEO::vec2& uv, MeasureType quality_type) const;

        /**
         * Evaluate the gradient of the Jacobian determinant at a facet parameter point.
         *
         * The output stores \f$\partial\det(J)/\partial x\f$ with respect to all local control-point
         * coordinates of facet \p f, flattened in xy order.
         *
         * @param[in] f Facet index, 0,1,...,quad_mesh.facets.nb()-1.
         * @param[in] uv Parameter point in the cell parameter domain [0, 1]^2.
         * @param[out] gradient Output buffer of size `2 * control_points_nb_per_facet()`.
         *                      For local control point `N`, entries are:
         *                      - `gradient[2*N+0] = d(detJ)/dP_N.x`
         *                      - `gradient[2*N+1] = d(detJ)/dP_N.y`
         */
        void compute_facet_uv_detJ_gradient(GEO::index_t f, const GEO::vec2& uv, std::vector<double>& gradient) const;

        /**
         * @brief Compute gradient of the squared absolute surface area objective at a parametric point.
         *
         * Computes the gradient of the objective
         *   F = 0.5 * (det(J))^2  (when DIM==2)
         * or
         *   F = 0.5 * || du x dv ||^2  (when DIM==3)
         * with respect to all local control-point coordinates of facet \p f. The routine
         * internally evaluates the first-order parametric derivatives (du, dv) and
         * assembles the per-control-point contribution using the tensor-product basis
         * derivatives.
         *
         * @param[in] f Facet index (0..mesh_.facets.nb()-1).
         * @param[in] uv Parameter point in the facet parameter domain [0,1]^2.
         * @param[out] gradient Output buffer that receives the gradient flattened per-control-point.
         *                      The buffer is resized to DIM * CONTROL_POINTS_NB_PER_FACET_. For
         *                      DIM==2 entries are stored as [dF/dP0.x, dF/dP0.y, dF/dP1.x, dF/dP1.y, ...].
         *                      For DIM==3 entries are stored as [dF/dP0.x, dF/dP0.y, dF/dP0.z, ...].
         * @pre f < mesh_.facets.nb()
         * @pre uv.x and uv.y in [0,1]
         * @note The function supports both 2D and 3D control grids (templated by DIM) and will
         *       compute the appropriate analytical gradient for each case using the Lagrange
         *       basis derivatives returned by compute_facet_uv_dudv.
         */
        void compute_facet_uv_absolute_area_sq_gradient(GEO::index_t f, const GEO::vec2& uv, std::vector<double>& gradient) const;

        /**
         * @brief Compute the reference area of every facet in the mesh.
         *
         * @param[out] areas Output array that receives one reference area per
         *            facet, in the same order as `quad_mesh_.facets`.
         * @note The caller is responsible for providing storage for all facets.
         */
        void compute_facets_area(std::vector<double>& areas);

        /**
         * Assemble physical control-point coordinates of one facet into a dense matrix.
         * @param[in] f facet index, 0,1,...,quad_mesh.facets.nb()-1
         * @param[out] P matrix of control-point positions for facet \\p f
         */
        void compute_cell_vertices_position_matrix(GEO::index_t f, Eigen::MatrixXd& P) const;

        /**
         * Assemble basis gradients at a parameter point for all local control points.
         * @param[in] uv parameter point in [0,1]^2
         * @param[out] Bg gradient matrix of tensor-product basis values
         * @pre Bg.size == CONTROL_POINTS_NB_PER_FACET * 2
         */
        void compute_basis_gradient_matrix(const GEO::vec2& uv, Eigen::MatrixXd& Bg) const;

        /**
         * @brief Append a discretized surfacic mesh of all high-order facets (for visualization purposes).
         *
         * The routine samples each quadrilateral facet with a regular
         * `resolution x resolution x resolution` grid in parametric space and appends
         * the generated facet elements to \p mesh_out.
         *
         * @param[in,out] mesh_out Output mesh that receives the discretized facets.
         * @param[in] resolution Number of samples per parametric direction inside each facet.
         *                      Must be greater than 0; larger values produce finer subdivision.
         * @param[out] mesh_out_v_facet Optional vertex attribute storing the source facet index
         *                             for each generated output vertex.
         * @param[out] mesh_out_v_uv Optional vertex attribute storing the corresponding
         *                            parametric coordinate of each generated output vertex.
         * @param[out] mesh_out_f_facet Optional facet attribute storing the source cell index
         *                             for each generated output facet element.
         */
        void append_discretized_high_order_facets(
            GEO::Mesh& mesh_out,
            GEO::index_t resolution = 10,
            GEO::Attribute<GEO::index_t>* mesh_out_v_facet = nullptr,
            GEO::Attribute<GEO::vec2>* mesh_out_v_uv = nullptr,
            GEO::Attribute<GEO::index_t>* mesh_out_f_facet = nullptr) const;

    protected:
        /**
             * @brief Initialize local indexing/layout rules for hexahedral control nodes.
             */
        void initialize_nodes_arrangement() override;

        /**
         * @brief Build global control-node coordinates and cell-to-control-node connectivity.
         */
        void initialize_control_nodes() override;
    };

    extern template class QuadControlGrid<2>;
    extern template class QuadControlGrid<3>;
}

#endif //HOSM_QUAD_CONTROL_GRID_H
