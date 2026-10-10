//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/4.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_QUAD_CONTROL_GRID_H
#define GEOLIO_QUAD_CONTROL_GRID_H
#include <geolio/common/pair_hash.h>
#include "basis_functions.h"
#include "surf_control_grid.h"
#include <geolio/common/Gauss_Legendre_quadrature_quad.h>
#include <geolio/common/vecg.h>

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
        QuadControlGrid(
            const GEO::Mesh& mesh,
            GEO::index_t order
            ): SurfaceControlGrid<DIM>(mesh.vertices.dimension(), order)
        {
            assert(mesh.facets.nb() > 0);
            assert([&]() {
                for (const auto& f : mesh.facets) {
                    if (mesh.facets.nb_vertices(f) != 4)
                        return false;
                 }
                 return true;
             }());

            QuadControlGrid::initialize_nodes_arrangement();
            QuadControlGrid::initialize_control_nodes(mesh);
        }

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
        [[nodiscard]] GEO::vecng<DIM, double> compute_facet_uv_position(
            const GEO::index_t f,
            const GEO::vec2& uv
            ) const {
            assert(f < this->control_nodes_mesh_.facets.nb());
            assert(uv.x >= 0 && uv.x <= 1);
            assert(uv.y >= 0 && uv.y <= 1);

            GEO::vecng<DIM, double> p;

            std::vector<double> Bu(this->order_+1);
            std::vector<double> Bv(this->order_+1);
            Lagrange_basis_1D(uv.x, this->node_positions_1D_, Bu);
            Lagrange_basis_1D(uv.y, this->node_positions_1D_, Bv);

            for (GEO::index_t i = 0; i <= this->order_; ++i) {
                for (GEO::index_t j = 0; j <= this->order_; ++j) {
                    const double lag_basis = Bu[i] * Bv[j];
                    p += lag_basis * this->control_node(this->facet_nd(f, i, j));
                }
            }

            return p;
        }

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
        [[nodiscard]] GEO::vecng<DIM, double> compute_facet_uv_position(
            const GEO::index_t f,
            const GEO::vec2& uv,
            const double* cur_control_nodes_ptr
            ) const {
            assert(f < this->control_nodes_mesh_.facets.nb());
            assert(uv.x >= 0 && uv.x <= 1);
            assert(uv.y >= 0 && uv.y <= 1);

            GEO::vecng<DIM, double> p;

            std::vector<double> Bu(this->order_+1);
            std::vector<double> Bv(this->order_+1);
            Lagrange_basis_1D(uv.x, this->node_positions_1D_, Bu);
            Lagrange_basis_1D(uv.y, this->node_positions_1D_, Bv);

            for (GEO::index_t i = 0; i <= this->order_; ++i) {
                for (GEO::index_t j = 0; j <= this->order_; ++j) {
                    const auto& nd = this->facet_nd(f, i, j);
                    const double lag_basis = Bu[i] * Bv[j];
                    p += lag_basis * GEO::vecng<DIM, double>(cur_control_nodes_ptr+DIM*nd);
                }
            }

            return p;
        }

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
        [[nodiscard]] GEO::vec3 compute_facet_uv_normal(
            const GEO::index_t f,
            const GEO::vec2& uv
            ) {
            assert(this->mesh_v_dim_ == 3);
            assert(f < this->control_nodes_mesh_.facets.nb());
            assert(uv.x >= 0 && uv.x <= 1);
            assert(uv.y >= 0 && uv.y <= 1);

            std::vector<double> Bu(this->order_+1);
            std::vector<double> Bv(this->order_+1);
            std::vector<double> dBu(this->order_+1);
            std::vector<double> dBv(this->order_+1);
            Lagrange_basis_1D(uv.x, this->node_positions_1D_, Bu);
            Lagrange_basis_1D(uv.y, this->node_positions_1D_, Bv);
            Lagrange_basis_deriv_1D(uv.x, this->node_positions_1D_, dBu);
            Lagrange_basis_deriv_1D(uv.y, this->node_positions_1D_, dBv);

            GEO::vec3 Tu(0, 0, 0), Tv(0, 0, 0);
            for (GEO::index_t i = 0; i <= this->order_; ++i) {
                for (GEO::index_t j = 0; j <= this->order_; ++j) {
                    const GEO::vec3& p = GEO::Memory::pointer_as_reference<GEO::vec3>(
                        this->control_node_ptr(this->facet_nd(f, i, j)));
                    Tu += p * dBu[i] * Bv[j];
                    Tv += p * Bu[i] * dBv[j];
                }
            }
            return -GEO::cross(Tu, Tv); /* The orientation of the vertices of the cell facet is towards the interior of the
                cell, so the normal direction needs to be reversed. */
        }

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
            const GEO::index_t f,
            const GEO::vec2& uv,
            GEO::vecng<DIM, double>& du,
            GEO::vecng<DIM, double>& dv,
            std::vector<double>& Bu,
            std::vector<double>& Bv,
            std::vector<double>& dBu,
            std::vector<double>& dBv
            ) const {
            assert(f < this->control_nodes_mesh_.facets.nb());
            assert(uv.x >= 0 && uv.x <= 1);
            assert(uv.y >= 0 && uv.y <= 1);

            std::fill_n(du.data(), DIM, 0.0);
            std::fill_n(dv.data(), DIM, 0.0);

            Bu.resize(this->order_+1);
            Bv.resize(this->order_+1);
            dBu.resize(this->order_+1);
            dBv.resize(this->order_+1);
            Lagrange_basis_1D(uv.x, this->node_positions_1D_, Bu);
            Lagrange_basis_1D(uv.y, this->node_positions_1D_, Bv);
            Lagrange_basis_deriv_1D(uv.x, this->node_positions_1D_, dBu);
            Lagrange_basis_deriv_1D(uv.y, this->node_positions_1D_, dBv);

            for (GEO::index_t i = 0; i <= this->order_; ++i) {
                for (GEO::index_t j = 0; j <= this->order_; ++j) {
                    const double lag_basis_duv = dBu[i] * Bv[j];
                    const double lag_basis_udv = Bu[i] * dBv[j];
                    du += lag_basis_duv * this->control_node(this->facet_nd(f, i, j));
                    dv += lag_basis_udv * this->control_node(this->facet_nd(f, i, j));
                }
            }
        }

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
        [[nodiscard]] GEO::vec3 compute_facet_reference_normal(
            const GEO::index_t f
            ) const {
            assert(this->mesh_v_dim_ == 3);
            assert(f < this->control_nodes_mesh_.facets.nb());
            const auto nd0 = GEO::Memory::pointer_as_reference<GEO::vec3>(this->control_node_ptr(this->facet_vertex_nd(f, 0)));
            const auto nd1 = GEO::Memory::pointer_as_reference<GEO::vec3>(this->control_node_ptr(this->facet_vertex_nd(f, 1)));
            const auto nd2 = GEO::Memory::pointer_as_reference<GEO::vec3>(this->control_node_ptr(this->facet_vertex_nd(f, 2)));
            const auto nd3 = GEO::Memory::pointer_as_reference<GEO::vec3>(this->control_node_ptr(this->facet_vertex_nd(f, 3)));
            return GEO::normalize(GEO::cross(nd2-nd0, nd3-nd1));
        }

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
        [[nodiscard]] double compute_facet_uv_measure(
            const GEO::index_t f,
            const GEO::vec2& uv,
            const MeasureType quality_type
            ) const {
            assert(f < this->control_nodes_mesh_.facets.nb());
            assert(uv.x >= 0 && uv.x <= 1);
            assert(uv.y >= 0 && uv.y <= 1);

            GEO::vecng<DIM, double> du_all, dv_all;
            std::vector<double> Bu, Bv, dBu, dBv;
            this->compute_facet_uv_dudv(f, uv, du_all, dv_all, Bu, Bv, dBu, dBv);

            double det_J = 0;
            double du_length, dv_length;
            if (this->mesh_v_dim_ == 2) {
                const GEO::vec2 du = GEO::Memory::pointer_as_reference<GEO::vec2>(du_all.data());
                const GEO::vec2 dv = GEO::Memory::pointer_as_reference<GEO::vec2>(dv_all.data());
                det_J = geolio::cross(du, dv);
                du_length = du.length();
                dv_length = dv.length();
            }
            else if (this->mesh_v_dim_ == 3) {
                const GEO::vec3 du = GEO::Memory::pointer_as_reference<GEO::vec3>(du_all.data());
                const GEO::vec3 dv = GEO::Memory::pointer_as_reference<GEO::vec3>(dv_all.data());
                const auto cross = GEO::cross(du, dv);
                if (quality_type == MeasureType::ABSOLUTE_SQ_AREA)
                    det_J = GEO::length(cross);
                else {
                    const auto ref_normal = compute_facet_reference_normal(f);
                    det_J = GEO::dot(cross, ref_normal);
                }
                du_length = du.length();
                dv_length = dv.length();
            }
            else
                assert(0);

            switch (quality_type) {
                case MeasureType::DET_JACOBIAN: {
                    return det_J;
                }
                case MeasureType::ABSOLUTE_SQ_AREA: {
                    return 0.5*det_J*det_J;
                }
                case MeasureType::MIPS: {
                    const double F_sq_norm = du_length*du_length + dv_length*dv_length;
                    return F_sq_norm / (2.0 * std::abs(det_J));
                }
                case MeasureType::SCALED_JACOBIAN: {
                    return det_J/(du_length*dv_length);
                }
                case MeasureType::INVERSE_MEAN_RATIO: {
                    const double F_sq_norm = du_length*du_length + dv_length*dv_length;
                    return 2.0*std::abs(det_J)/F_sq_norm;
                }
                default: assert(0);
            }

            return 0;
        }

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
        void compute_facet_uv_detJ_gradient(
            const GEO::index_t f,
            const GEO::vec2& uv,
            std::vector<double>& gradient
            ) const {
            assert(f < this->control_nodes_mesh_.facets.nb());
            assert(uv.x >= 0 && uv.x <= 1);
            assert(uv.y >= 0 && uv.y <= 1);

            GEO::vecng<DIM, double> du_all, dv_all;
            std::vector<double> Bu, Bv, dBu, dBv;
            this->compute_facet_uv_dudv(f, uv, du_all, dv_all, Bu, Bv, dBu, dBv);

            gradient.resize(this->mesh_v_dim_ * this->CONTROL_POINTS_NB_PER_FACET_);

            if (this->mesh_v_dim_ == 2) {
                const GEO::vec2 du = GEO::Memory::pointer_as_reference<GEO::vec2>(du_all.data());
                const GEO::vec2 dv = GEO::Memory::pointer_as_reference<GEO::vec2>(dv_all.data());
                const GEO::vec2 perp_dv(dv.y, -dv.x); // (-dv_y, dv_x)
                const GEO::vec2 perp_du(-du.y, du.x);  // (du_y, -du_x)
                for (GEO::index_t j = 0; j < this->CONTROL_POINTS_NB_PER_EDGE_; ++j) {
                    for (GEO::index_t i = 0; i < this->CONTROL_POINTS_NB_PER_EDGE_; ++i) {
                        const double lag_basis_duv = dBu[i] * Bv[j];
                        const double lag_basis_udv = Bu[i] * dBv[j];
                        const auto& lnd = this->facet_lnd(i, j);
                        const auto& g = lag_basis_duv*perp_dv + lag_basis_udv*perp_du;
                        gradient[2*lnd] = g.x;
                        gradient[2*lnd+1] = g.y;
                    }
                }
            }
            else if (this->mesh_v_dim_ == 3) { // gradient 0.5 * \Vert cross(du, dv) \Vert^2
                const GEO::vec3 du = GEO::Memory::pointer_as_reference<GEO::vec3>(du_all.data());
                const GEO::vec3 dv = GEO::Memory::pointer_as_reference<GEO::vec3>(dv_all.data());
                const auto ref_normal = compute_facet_reference_normal(f);
                const auto perp_du = GEO::cross(ref_normal, du);
                const auto perp_dv = GEO::cross(dv, ref_normal);

                for (GEO::index_t j = 0; j < this->CONTROL_POINTS_NB_PER_EDGE_; ++j) {
                    for (GEO::index_t i = 0; i < this->CONTROL_POINTS_NB_PER_EDGE_; ++i) {
                        const double lag_basis_duv = dBu[i] * Bv[j];
                        const double lag_basis_udv = Bu[i] * dBv[j];
                        const auto& lnd = this->facet_lnd(i, j);
                        const auto g = lag_basis_duv * perp_dv + lag_basis_udv * perp_du;
                        gradient[3*lnd] = g.x;
                        gradient[3*lnd+1] = g.y;
                        gradient[3*lnd+2] = g.z;
                    }
                }
            }
            else
                assert(0);
        }

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
        void compute_facet_uv_absolute_area_sq_gradient(
            const GEO::index_t f,
            const GEO::vec2& uv,
            std::vector<double>& gradient
            ) const {
            assert(f < this->control_nodes_mesh_.facets.nb());
            assert(uv.x >= 0 && uv.x <= 1);
            assert(uv.y >= 0 && uv.y <= 1);

            GEO::vecng<DIM, double> du_all, dv_all;
            std::vector<double> Bu, Bv, dBu, dBv;
            this->compute_facet_uv_dudv(f, uv, du_all, dv_all, Bu, Bv, dBu, dBv);

            gradient.resize(this->mesh_v_dim_ * this->CONTROL_POINTS_NB_PER_FACET_);

            if (this->mesh_v_dim_ == 2) {
                const GEO::vec2 du = GEO::Memory::pointer_as_reference<GEO::vec2>(du_all.data());
                const GEO::vec2 dv = GEO::Memory::pointer_as_reference<GEO::vec2>(dv_all.data());
                // 2D objective function: f = 0.5 * det(J)^2
                // det(J) = du.x * dv.y - du.y * dv.x
                const double detJ = du.x * dv.y - du.y * dv.x;
                const GEO::vec2 perp_dv(dv.y, -dv.x);
                const GEO::vec2 perp_du(-du.y, du.x);

                for (GEO::index_t j = 0; j < this->CONTROL_POINTS_NB_PER_EDGE_; ++j) {
                    for (GEO::index_t i = 0; i < this->CONTROL_POINTS_NB_PER_EDGE_; ++i) {
                        const double lag_basis_duv = dBu[i] * Bv[j]; // α
                        const double lag_basis_udv = Bu[i] * dBv[j]; // β
                        const auto& lnd = this->facet_lnd(i, j);

                        const auto g_detJ = lag_basis_duv * perp_dv + lag_basis_udv * perp_du;

                        // ∇(0.5 * det(J)^2) = det(J) * ∇(det(J))
                        gradient[2*lnd]   = detJ * g_detJ.x;
                        gradient[2*lnd+1] = detJ * g_detJ.y;
                    }
                }
            }
            else if (this->mesh_v_dim_ == 3) {
                const GEO::vec3 du = GEO::Memory::pointer_as_reference<GEO::vec3>(du_all.data());
                const GEO::vec3 dv = GEO::Memory::pointer_as_reference<GEO::vec3>(dv_all.data());
                // 3D objective function: f = 0.5 * || du x dv ||^2
                const GEO::vec3 du_3d(du.x, du.y, du.z);
                const GEO::vec3 dv_3d(dv.x, dv.y, dv.z);

                // normal: N = du x dv
                const GEO::vec3 N = GEO::cross(du_3d, dv_3d);

                for (GEO::index_t j = 0; j < this->CONTROL_POINTS_NB_PER_EDGE_; ++j) {
                    for (GEO::index_t i = 0; i < this->CONTROL_POINTS_NB_PER_EDGE_; ++i) {
                        const double lag_basis_duv = dBu[i] * Bv[j]; // α
                        const double lag_basis_udv = Bu[i] * dBv[j]; // β
                        const auto& lnd = this->facet_lnd(i, j);

                        // V_ij = α * dv - β * du
                        const GEO::vec3 V_ij(
                            lag_basis_duv * dv.x - lag_basis_udv * du.x,
                            lag_basis_duv * dv.y - lag_basis_udv * du.y,
                            lag_basis_duv * dv.z - lag_basis_udv * du.z
                        );

                        // 0.5 * ||N||^2 -> V_ij x N
                        const auto g = GEO::cross(V_ij, N);

                        gradient[3*lnd]   = g.x;
                        gradient[3*lnd+1] = g.y;
                        gradient[3*lnd+2] = g.z;
                    }
                }
            }
            else
                assert(0);
        }

        /**
         * @brief Compute the reference area of every facet in the mesh.
         *
         * @param[out] areas Output array that receives one reference area per
         *            facet, in the same order as `quad_mesh_.facets`.
         * @note The caller is responsible for providing storage for all facets.
         */
        void compute_facets_area(
            std::vector<double>& areas
            ) {
            areas.resize(this->control_nodes_mesh_.facets.nb());

            /*
             * DIM == 2: 2n-1 >= 2p-1, n >= p
             * DIM == 3: appro n = p+1 or p+2
             */
            std::vector<std::pair<GEO::vec2, double>> points_and_weights;
            geolio::get_Gauss_Legendre_quadrature_quad(std::ceil(this->order_ + DIM-2), points_and_weights);

            for (const auto& f : this->control_nodes_mesh_.facets) {
                auto& S = areas[f];
                S = 0;
                for (const auto& [uv, w] : points_and_weights)
                    S += w * std::abs(compute_facet_uv_measure(f, uv, QuadControlGrid::MeasureType::DET_JACOBIAN));

            }
        }

        /**
         * Assemble basis gradients at a parameter point for all local control points.
         * @param[in] uv parameter point in [0,1]^2
         * @param[out] Bg gradient matrix of tensor-product basis values
         * @pre Bg.size == CONTROL_POINTS_NB_PER_FACET * 2
         */
        void compute_basis_gradient_matrix(
            const GEO::vec2& uv,
            Eigen::MatrixXd& Bg
            ) const {
            assert(uv.x >= 0 && uv.x <= 1);
            assert(uv.y >= 0 && uv.y <= 1);
            assert(Bg.rows() == this->CONTROL_POINTS_NB_PER_FACET_);
            assert(Bg.cols() == 2);

            std::vector<double> Bu(this->order_+1);
            std::vector<double> Bv(this->order_+1);
            std::vector<double> dBu(this->order_+1);
            std::vector<double> dBv(this->order_+1);
            Lagrange_basis_1D(uv.x, this->node_positions_1D_, Bu);
            Lagrange_basis_1D(uv.y, this->node_positions_1D_, Bv);
            Lagrange_basis_deriv_1D(uv.x, this->node_positions_1D_, dBu);
            Lagrange_basis_deriv_1D(uv.y, this->node_positions_1D_, dBv);
            for (GEO::index_t i = 0; i < this->CONTROL_POINTS_NB_PER_EDGE_; ++i) {
                for (GEO::index_t j = 0; j < this->CONTROL_POINTS_NB_PER_EDGE_; ++j) {
                    const auto N = this->facet_lnd(i, j);
                    Bg(N, 0) = dBu[i]*Bv[j];
                    Bg(N, 1) = Bu[i]*dBv[j];
                }
            }
        }

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
            GEO::Attribute<GEO::index_t>* mesh_out_f_facet = nullptr
            ) const {
            if (mesh_out_v_facet != nullptr) {
                assert(mesh_out_v_facet->is_bound());
                assert(mesh_out_v_facet->size() == mesh_out.vertices.nb());
            }
            if (mesh_out_v_uv != nullptr) {
                assert(mesh_out_v_uv->is_bound());
                assert(mesh_out_v_uv->size() == mesh_out.vertices.nb());
            }
            if (mesh_out_f_facet != nullptr) {
                assert(mesh_out_f_facet->is_bound());
                assert(mesh_out_f_facet->size() == mesh_out.facets.nb());
            }

            const GEO::index_t VERTICES_NB_PER_EDGE = resolution+1;

            mesh_out.vertices.set_dimension(DIM);
            GEO::index_t new_v = mesh_out.vertices.create_vertices(this->control_nodes_mesh_.facets.nb() * (resolution+1) * (resolution+1));
            GEO::index_t new_f = mesh_out.facets.create_quads(this->control_nodes_mesh_.facets.nb() * resolution * resolution);
            for (const auto& f : this->control_nodes_mesh_.facets) {
                const auto PREV_M_VERTICES = new_v;

                /*
                 * Vertices:
                 * y & j
                 *   |
                 *  ...
                 *   |         |       ...       |         |
                 * (0,1) --- (1,1) --- ... --- (n,1) -- (n+1,1)
                 *   |         |       ...       |         |
                 * (0,0) --- (1,0) --- ... --- (n,0) -- (n+1,0) -> x & i
                 */
                for (GEO::index_t i = 0; i < VERTICES_NB_PER_EDGE; ++i) {
                    const double u = static_cast<double>(i)/resolution;
                    for (GEO::index_t j = 0; j < VERTICES_NB_PER_EDGE; ++j) {
                        const double v = static_cast<double>(j)/resolution;

                        const GEO::vec2 uv(u, v);

                        mesh_out.vertices.point<DIM>(new_v) = compute_facet_uv_position(f, uv);

                        if (mesh_out_v_facet != nullptr)
                            (*mesh_out_v_facet)[new_v] = f;
                        if (mesh_out_v_uv != nullptr)
                            (*mesh_out_v_uv)[new_v] = uv;

                        ++new_v;
                    }
                }

                /*
                 * Facets:
                 * ...
                 * +-----+-----+- ... -+-----+      v3 --- v2
                 * | n+1 | n+2 |  ...  |2n+1 |       |     |
                 * +-----+-----+- ... -+-----+      v0 --- v1
                 * |  0  |  1  |  ...  |  n  |
                 * +-----+-----+- ... -+-----+
                 */
                for (GEO::index_t i = 0; i < resolution; ++i) {
                    for (GEO::index_t j = 0; j < resolution; ++j) {
                        const GEO::index_t v0 = VERTICES_NB_PER_EDGE*i+j;
                        const GEO::index_t v1 = v0+VERTICES_NB_PER_EDGE;
                        const GEO::index_t v2 = v1+1;
                        const GEO::index_t v3 = v0+1;
                        assert(v0 < mesh_out.vertices.nb());
                        assert(v1 < mesh_out.vertices.nb());
                        assert(v2 < mesh_out.vertices.nb());
                        assert(v3 < mesh_out.vertices.nb());
                        mesh_out.facets.set_vertex(new_f, 0, PREV_M_VERTICES+v0);
                        mesh_out.facets.set_vertex(new_f, 1, PREV_M_VERTICES+v1);
                        mesh_out.facets.set_vertex(new_f, 2, PREV_M_VERTICES+v2);
                        mesh_out.facets.set_vertex(new_f, 3, PREV_M_VERTICES+v3);

                        if (mesh_out_f_facet != nullptr)
                            (*mesh_out_f_facet)[new_f] = f;

                        ++new_f;
                    }
                }
            }

            mesh_out.facets.connect();
        }

    protected:
        /**
             * @brief Initialize local indexing/layout rules for hexahedral control nodes.
             */
        void initialize_nodes_arrangement() override {
            const GEO::index_t LAST_LAYER_BEGIN_IDX = (this->CONTROL_POINTS_NB_PER_EDGE_-1)*this->CONTROL_POINTS_NB_PER_EDGE_;

            /* == Vertex =============================================================================================== */
            this->ELEMENT_VERTEX_CONTROL_POINTS_BEGIN_IDX_ = {
                0,
                this->CONTROL_POINTS_NB_PER_EDGE_-1,
                this->CONTROL_POINTS_NB_PER_FACET_-1,
                LAST_LAYER_BEGIN_IDX
            };

            /* == Edge ================================================================================================= */
            this->ELEMENT_EDGE_CONTROL_POINTS_BEGIN_IDX_ = this->ELEMENT_VERTEX_CONTROL_POINTS_BEGIN_IDX_;
            this->ELEMENT_EDGE_CONTROL_POINTS_NEXT_IDX_STEP_ = {
                1,
                static_cast<int>(this->CONTROL_POINTS_NB_PER_EDGE_),
                -1,
                -static_cast<int>(this->CONTROL_POINTS_NB_PER_EDGE_),
            };
            this->ELEMENT_EDGE_INTERNAL_CONTROL_POINTS_BEGIN_IDX_ = {
                1,
                2*this->CONTROL_POINTS_NB_PER_EDGE_-1,
                this->CONTROL_POINTS_NB_PER_FACET_-2,
                (this->CONTROL_POINTS_NB_PER_EDGE_-2)*this->CONTROL_POINTS_NB_PER_EDGE_
            };
            this->ELEMENT_EDGE_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP_ = this->ELEMENT_EDGE_CONTROL_POINTS_NEXT_IDX_STEP_;

            /* == Facet ================================================================================================ */
            this->ELEMENT_CONTROL_POINTS_BEGIN_IDX_ = 0;
            this->ELEMENT_CONTROL_POINTS_NEXT_IDX_STEP0_ = 1;
            this->ELEMENT_CONTROL_POINTS_NEXT_IDX_STEP1_ = static_cast<int>(this->CONTROL_POINTS_NB_PER_EDGE_);
            this->ELEMENT_INTERNAL_CONTROL_POINTS_BEGIN_IDX_ = this->CONTROL_POINTS_NB_PER_EDGE_+1;
            this->ELEMENT_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP0_ = this->ELEMENT_CONTROL_POINTS_NEXT_IDX_STEP0_;
            this->ELEMENT_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP1_ = this->ELEMENT_CONTROL_POINTS_NEXT_IDX_STEP1_;
        }

        /**
         * @brief Build global control-node coordinates and cell-to-control-node connectivity.
         */
        void initialize_control_nodes(const GEO::Mesh& mesh) override {
            assert(this->node_positions_1D_.size() == this->order_+1);

            /* == Get all shared edges ============================================================================= */
            std::unordered_map<std::pair<GEO::index_t, GEO::index_t>, std::vector<GEO::index_t>, PairHash> quad_edges_control_points; /* (ev0, ev1), ev0 < ev1 -> control vertices from ev0 -> ev1 */
            {
                for (const auto& f : mesh.facets) {
                    for (GEO::index_t lv = 0; lv < 4; ++lv) {
                        const std::pair<GEO::index_t, GEO::index_t> edge = std::minmax(
                            mesh.facets.vertex(f, lv),
                            mesh.facets.vertex(f, (lv+1)%4));
                        quad_edges_control_points.emplace(edge, std::vector<GEO::index_t>(this->INTERNAL_CONTROL_POINTS_NB_PER_EDGE_, GEO::NO_VERTEX));
                    }
                }
            }

            /* == Create grid vertices ============================================================================= */
            this->control_nodes_mesh_.vertices.clear();
            GEO::index_t new_v = this->control_nodes_mesh_.vertices.create_vertices(
                                mesh.vertices.nb() + // vertices
                                quad_edges_control_points.size() * this->INTERNAL_CONTROL_POINTS_NB_PER_EDGE_ + // edges
                                mesh.facets.nb() * this->INTERNAL_CONTROL_POINTS_NB_PER_FACET_ // facets
                                );
            assert(new_v == 0);

            /* == For vertices == */
            for (const auto& v : mesh.vertices)
                std::copy_n(mesh.vertices.point_ptr(v), this->mesh_v_dim_, this->control_node_ptr(new_v++));

            /* == For edges == */
            for (auto& [edge, control_vertices] : quad_edges_control_points) {
                GEO::vecng<DIM, double> ep0, ep1;
                std::copy_n(mesh.vertices.point_ptr(edge.first), this->mesh_v_dim_, ep0.data());
                std::copy_n(mesh.vertices.point_ptr(edge.second), this->mesh_v_dim_, ep1.data());
                for (GEO::index_t i = 0, i_end = this->INTERNAL_CONTROL_POINTS_NB_PER_EDGE_; i < i_end; ++i) {
                    const double r = this->node_positions_1D_[i+1];
                    this->control_node(new_v) = (1-r)*ep0 + r*ep1;
                    control_vertices[i] = new_v;
                    ++new_v;
                }
            }

            /* == For facets == */
            std::vector<std::vector<GEO::index_t>> quad_facets_control_points(mesh.facets.nb()); /*
                [f] -> the idx of control points of this facet  */
            for (const auto& f : mesh.facets) {
                auto& f_control_points = quad_facets_control_points[f];
                f_control_points.reserve(this->INTERNAL_CONTROL_POINTS_NB_PER_FACET_);

                assert(mesh.facets.nb_vertices(f) == 4);
                GEO::vecng<DIM, double> f_p0, f_p1, f_p2, f_p3;
                std::copy_n(mesh.vertices.point_ptr(mesh.facets.vertex(f, 0)), this->mesh_v_dim_, f_p0.data());
                std::copy_n(mesh.vertices.point_ptr(mesh.facets.vertex(f, 1)), this->mesh_v_dim_, f_p1.data());
                std::copy_n(mesh.vertices.point_ptr(mesh.facets.vertex(f, 2)), this->mesh_v_dim_, f_p2.data());
                std::copy_n(mesh.vertices.point_ptr(mesh.facets.vertex(f, 3)), this->mesh_v_dim_, f_p3.data());

                for (GEO::index_t i = 0; i < this->INTERNAL_CONTROL_POINTS_NB_PER_EDGE_; ++i) {
                    const double ri = this->node_positions_1D_[i+1];
                    for (GEO::index_t j = 0; j < this->INTERNAL_CONTROL_POINTS_NB_PER_EDGE_; ++j) {
                        const double rj = this->node_positions_1D_[j+1];

                        this->control_node(new_v) = (1-ri)*(1-rj)*f_p0
                                                  + ri*(1-rj)*f_p3
                                                  + (1-ri)*rj*f_p1
                                                  + ri*rj*f_p2;
                            f_control_points.push_back(new_v);
                            ++new_v;
                    }
                }
            }

            assert(new_v == this->control_nodes_nb());

            /* == Create grid facets =============================================================================== */
            this->control_nodes_mesh_.facets.create_quads(mesh.facets.nb());
            for (const auto& f : mesh.facets) {
                for (GEO::index_t lv = 0; lv < 4; ++lv)
                    this->control_nodes_mesh_.facets.set_vertex(f, lv, mesh.facets.vertex(f, lv));
            }
            this->control_nodes_mesh_.facets.connect();

            /* Initialize other dimension */
            this->initialize_control_node_quantities();

            /* == Create regular index ============================================================================= */
            this->element_control_nodes_.create_vector_attribute(
                this->control_nodes_mesh_.facets.attributes(),
                "control_nodes",
                this->CONTROL_POINTS_NB_PER_FACET_);
            this->element_control_nodes_.fill(GEO::NO_INDEX);

            for (const auto& f : mesh.facets) {
                const GEO::index_t FACET_BEGIN_IDX = f*this->CONTROL_POINTS_NB_PER_FACET_;

                /* For vertices */
                for (GEO::index_t lv = 0; lv < 4; ++lv)
                    this->element_control_nodes_[
                            FACET_BEGIN_IDX +
                            this->facet_vertex_lnd(lv)
                            ] = mesh.facets.vertex(f, lv);

                /* For edges */
                for (GEO::index_t le = 0; le < 4; ++le) {
                    const auto& ev0 = mesh.facets.vertex(f, le);
                    const auto& ev1 = mesh.facets.vertex(f, (le+1)%4);
                    const std::pair<GEO::index_t, GEO::index_t> edge = std::minmax(ev0, ev1);

                    assert(quad_edges_control_points.contains(edge));
                    const auto& edge_control_points = quad_edges_control_points.at(edge);
                    assert(edge_control_points.size() == this->INTERNAL_CONTROL_POINTS_NB_PER_EDGE_);

                    if (ev0 == edge.first) { // do not need to inverse
                        for (GEO::index_t lv = 0; lv < this->INTERNAL_CONTROL_POINTS_NB_PER_EDGE_; ++lv)
                            this->element_control_nodes_[
                                FACET_BEGIN_IDX +
                                this->facet_edge_inner_lnd(le, lv)
                                ] = edge_control_points[lv];
                    }
                    else { // need to inverse
                        for (GEO::index_t lv = 0; lv < this->INTERNAL_CONTROL_POINTS_NB_PER_EDGE_; ++lv)
                            this->element_control_nodes_[
                                FACET_BEGIN_IDX +
                                this->facet_edge_inner_lnd(le, lv)
                                ] = edge_control_points[this->INTERNAL_CONTROL_POINTS_NB_PER_EDGE_-1-lv];
                    }
                }

                /* For facets */
                const auto& facet_control_points = quad_facets_control_points[f];
                assert(facet_control_points.size() == this->INTERNAL_CONTROL_POINTS_NB_PER_FACET_);

                for (GEO::index_t lv1 = 0; lv1 < this->INTERNAL_CONTROL_POINTS_NB_PER_EDGE_; ++lv1) {
                    for (GEO::index_t lv0 = 0; lv0 < this->INTERNAL_CONTROL_POINTS_NB_PER_EDGE_; ++lv0) {
                        this->element_control_nodes_[
                            FACET_BEGIN_IDX +
                            this->facet_inner_lnd(lv0, lv1)
                            ] = facet_control_points[lv1*this->INTERNAL_CONTROL_POINTS_NB_PER_EDGE_ + lv0];
                    }
                }
            }
        }
    };

    template <typename T>
    struct isQuadControlGrid : std::false_type {};

    template <GEO::index_t DIM>
    struct isQuadControlGrid<QuadControlGrid<DIM>> : std::true_type {};

    template <typename T>
    struct QuadControlGridDim;

    template <GEO::index_t DIM>
    struct QuadControlGridDim<QuadControlGrid<DIM>> : std::integral_constant<GEO::index_t, DIM> {};
}

#endif //GEOLIO_QUAD_CONTROL_GRID_H
