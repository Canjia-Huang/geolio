//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/2.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_HEX_CONTROL_GRID_H
#define GEOLIO_HEX_CONTROL_GRID_H
#include <cassert>
#include <geolio/common/Gauss_Legendre_quadrature_cube.h>
#include <geolio/common/log.h>
#include <geolio/common/pair_hash.h>
#include <geolio/mesh/hex_operations.h>
#include "basis_functions.h"
#include "volume_control_grid.h"

namespace geolio
{
    /**
     * @brief Projects a 3D parametric coordinate to a 1D parameter on a hexahedron edge.
     *
     * Projects a point (u,v,w) from the unit cubic parametric domain [0,1]^3 to
     * a specified hexahedron edge, returning the 1D parameter t ∈ [0,1] on that edge.
     *
     * @param uvw The 3D parametric coordinate in the unit cube (u, v, w) ∈ [0,1]^3
     * @param le  The local edge index of the hexahedron, range [0, 11]
     *
     * @return The projected 1D parameter t ∈ [0,1], representing the position on the edge
     *         (t=0 corresponds to the start of the edge, t=1 to the end)
     */
    inline double project_uvw_to_hex_le_t(const GEO::vec3& uvw, const GEO::index_t le) {
        assert(le < 12);
        switch (le) {
            case 0: return uvw.x;
            case 1: return uvw.y;
            case 2: return 1-uvw.x;
            case 3: return 1-uvw.y;
            case 4: return uvw.x;
            case 5: return uvw.y;
            case 6: return 1-uvw.x;
            case 7: return 1-uvw.y;
            case 8: [[fallthrough]];
            case 9: [[fallthrough]];
            case 10: [[fallthrough]];
            case 11: return uvw.z;
            default: return -1;
        }
    }

    /**
     * @brief Projects a 3D parametric coordinate to 2D parameters on a hexahedron facet.
     *
     * Projects a point (u,v,w) from the unit cubic parametric domain [0,1]^3 to
     * a specified hexahedron facet, returning the 2D parameters (u', v') ∈ [0,1]^2 on that facet.
     *
     * @param uvw The 3D parametric coordinate in the unit cube (u, v, w) ∈ [0,1]^3
     * @param lf  The local facet index of the hexahedron, range [0, 5]
     *
     * @return The projected 2D parameters (u', v') ∈ [0,1]^2, representing the position on the facet
     */
    inline GEO::vec2 project_uvw_to_hex_lf_uv(const GEO::vec3& uvw, const GEO::index_t lf) {
        assert(lf < 6);
        GEO::vec2 uv;
        switch (lf) {
            case 0: uv = GEO::vec2(uvw.y, uvw.z); break;
            case 1: uv = GEO::vec2(1-uvw.y, uvw.z); break;
            case 2: uv = GEO::vec2(1-uvw.x, uvw.z); break;
            case 3: uv = GEO::vec2(uvw.x, uvw.z); break;
            case 4: uv = GEO::vec2(uvw.y, 1-uvw.x); break;
            case 5: uv = GEO::vec2(uvw.y, uvw.x); break;
            default: uv = GEO::vec2(-1, -1);
        }
        return uv;
    }

    /**
     * @brief Projects a hexahedron vertex to 3D parametric coordinates.
     *
     * Given a local vertex index in a hexahedron (0-7), returns the corresponding
     * 3D parametric coordinate (u,v,w) in the unit cube domain [0,1]^3 for that vertex.
     * Vertex indexing follows standard hexahedron topology: bottom face vertices 0-3
     * (z=0), top face vertices 4-7 (z=1).
     *
     * @param[in] lv Local vertex index in the hexahedron, range [0, 7].
     *
     * @return The 3D parametric coordinate (u,v,w) ∈ {0,1}^3 corresponding to the vertex.
     */
    inline GEO::vec3 project_hex_lv_to_uvw(const GEO::index_t lv) {
        assert(lv < 8);
        GEO::vec3 uvw;
        switch (lv) {
            case 0: uvw = GEO::vec3(0, 0, 0); break;
            case 1: uvw = GEO::vec3(1, 0, 0); break;
            case 2: uvw = GEO::vec3(0, 1, 0); break;
            case 3: uvw = GEO::vec3(1, 1, 0); break;
            case 4: uvw = GEO::vec3(0, 0, 1); break;
            case 5: uvw = GEO::vec3(1, 0, 1); break;
            case 6: uvw = GEO::vec3(0, 1, 1); break;
            case 7: uvw = GEO::vec3(1, 1, 1); break;
            default: uvw = GEO::vec3(-1, -1, -1);
        }
        return uvw;
    }

    /**
     * @brief Projects a 1D parameter on a hexahedron edge back to 3D parametric coords.
     *
     * Given a scalar parameter t in [0,1] defined along the local hexahedron edge
     * identified by `le`, return the corresponding 3D parametric coordinate (u,v,w)
     * in the unit cube domain [0,1]^3 that lies on that edge.
     *
     * @param[in] t  The 1D parameter along the edge (t=0 -> edge start, t=1 -> edge end).
     * @param[in] le Local edge index in the hexahedron (0..11).
     * @return The 3D parametric coordinate (u,v,w) in [0,1]^3 corresponding to the edge
     *         parameter.
     */
    inline GEO::vec3 project_hex_le_t_to_uvw(const double t, const GEO::index_t le) {
        assert(le < 12);
        GEO::vec3 uvw;
        switch (le) {
            case 0: uvw = GEO::vec3(t, 0, 0); break;
            case 1: uvw = GEO::vec3(1, t, 0); break;
            case 2: uvw = GEO::vec3(1-t, 1, 0); break;
            case 3: uvw = GEO::vec3(0, 1-t, 0); break;
            case 4: uvw = GEO::vec3(t, 0, 1); break;
            case 5: uvw = GEO::vec3(1, t, 1); break;
            case 6: uvw = GEO::vec3(1-t, 1, 1); break;
            case 7: uvw = GEO::vec3(0, 1-t, 1); break;
            case 8: uvw = GEO::vec3(0, 0, t); break;
            case 9: uvw = GEO::vec3(1, 0, t); break;
            case 10: uvw = GEO::vec3(1, 1, t); break;
            case 11: uvw = GEO::vec3(0, 1, t); break;
            default: uvw = GEO::vec3(-1, -1, -1);
        }
        return uvw;
    }

    /**
     * @brief Projects 2D parameters on a hexahedron facet back to 3D parametric coordinates.
     *
     * Performs the inverse operation of proj_uvw_to_hex_lf_uv(). Given 2D parameters (u', v')
     * on a specified hexahedron facet, returns the corresponding 3D parametric coordinate (u, v, w)
     * in the unit cubic domain [0,1]^3.
     *
     * @param uv The 2D parameter on the hexahedron facet (u', v') ∈ [0,1]^2
     * @param lf The local facet index of the hexahedron, range [0, 5]
     *
     * @return The 3D parametric coordinate (u, v, w) ∈ [0,1]^3 corresponding to the input 2D parameters
     */
    inline GEO::vec3 project_hex_lf_uv_to_uvw(const GEO::vec2& uv, const GEO::index_t lf) {
        assert(lf < 6);
        GEO::vec3 uvw;
        switch(lf) {
            case 0: uvw = GEO::vec3(0, uv.x, uv.y); break;
            case 1: uvw = GEO::vec3(1, 1-uv.x, uv.y); break;
            case 2: uvw = GEO::vec3(1-uv.x, 0, uv.y); break;
            case 3: uvw = GEO::vec3(uv.x, 1, uv.y); break;
            case 4: uvw = GEO::vec3(1-uv.y, uv.x, 0); break;
            case 5: uvw = GEO::vec3(uv.y, uv.x, 1); break;
            default: uvw = GEO::vec3(-1, -1, -1);
        }
        return uvw;
    }

    template<GEO::index_t MESH_DIM, GEO::index_t QUANTITIES_DIM = 0>
    class HexControlGrid : public VolumeControlGrid<MESH_DIM+QUANTITIES_DIM> {
        static_assert(MESH_DIM == 3);
        static constexpr GEO::index_t DIM = MESH_DIM + QUANTITIES_DIM;
    public:
        /**
         * @brief Construct a hexahedral high-order control grid.
         * @param[in] mesh Input hexahedral mesh used as the reference topology/geometry.
         * @param[in] order Polynomial order of the tensor-product hexahedral mapping.
         */
        HexControlGrid(
            const GEO::Mesh& mesh,
            GEO::index_t order
            ) : VolumeControlGrid<DIM>(order)
        {
            assert(mesh.cells.nb() > 0);
            assert(std::all_of(
                mesh.cells.cell_type_ptr(0),
                mesh.cells.cell_type_ptr(0)+mesh.cells.nb(),
                [&](const auto cell_type) { return cell_type == GEO::MESH_HEX; })); // check all-hex mesh

            HexControlGrid::initialize_nodes_arrangement();
            HexControlGrid::initialize_control_nodes(mesh);
        }

        /**
         * @brief Map a parametric coordinate inside a cell to physical space.
         *
         * Evaluates the high-order cell mapping at the given parametric coordinate
         * `uvw` ∈ [0,1]^3 and returns the corresponding physical-space point. The
         * mapping is built from the cell's control-point positions stored in the
         * internal control grid `grid_`.
         *
         * @param[in] c Index of the hexahedral cell (0..hex_mesh_.cells.nb()-1).
         * @param[in] uvw Parametric coordinate in the cell-local domain [0,1]^3.
         *
         * @return Physical-space position corresponding to the input parametric point.
         *
         * @note Preconditions: `c` must be a valid cell index and the components of
         *       `uvw` are expected to be in [0,1] for meaningful results. The
         *       function uses the control-point coordinates currently stored in
         *       `grid_` to compute the mapped position.
         */
        [[nodiscard]] GEO::vecng<DIM, double> compute_cell_uvw_position(
            const GEO::index_t c,
            const GEO::vec3& uvw
            ) const {
            assert(c < this->control_nodes_mesh_.cells.nb());
            assert(uvw.x >= 0 && uvw.x <= 1);
            assert(uvw.y >= 0 && uvw.y <= 1);
            assert(uvw.z >= 0 && uvw.z <= 1);

            GEO::vecng<DIM, double> p;
            std::fill_n(p.data(), DIM, 0.0);

            std::vector<double> Bu(this->order_+1);
            std::vector<double> Bv(this->order_+1);
            std::vector<double> Bw(this->order_+1);
            Lagrange_basis_1D(uvw.x, this->node_positions_1D_, Bu);
            Lagrange_basis_1D(uvw.y, this->node_positions_1D_, Bv);
            Lagrange_basis_1D(uvw.z, this->node_positions_1D_, Bw);

            for (GEO::index_t i = 0; i <= this->order_; ++i) {
                for (GEO::index_t j = 0; j <= this->order_; ++j) {
                    const double basis_uv = Bu[i] * Bv[j];
                    for (GEO::index_t k = 0; k <= this->order_; ++k) {
                        const double lag_basis = basis_uv * Bw[k];
                        p += lag_basis * this->control_node(this->cell_nd(c, i, j, k));
                    }
                }
            }

            return p;
        }

        /**
         * @brief Map a parametric coordinate to physical space using provided control positions.
         *
         * Variant of `compute_cell_uvw_position` that evaluates the cell mapping
         * using an externally supplied flat array of control-node positions. This
         * is useful for evaluating hypothetical configurations (e.g., during
         * optimization) without modifying the internal control grid.
         *
         * @param[in] c Index of the hexahedral cell (0..hex_mesh_.cells.nb()-1).
         * @param[in] uvw Parametric coordinate in the cell-local domain [0,1]^3.
         * @param[in] cur_control_nodes_ptr Pointer to a flat array of control-node
         *            coordinates in the same layout used by the optimizer
         *            (`[x0,y0,z0,x1,y1,z1,...]` for all control vertices). The
         *            array must contain at least `3 * control_points_nb()` entries.
         *
         * @return Physical-space position corresponding to the input parametric point
         *         evaluated with the provided control-node positions.
         *
         * @note The function does not take ownership of `control_nodes_position` and
         *       treats it as read-only. Caller must ensure the buffer is valid.
         */
        [[nodiscard]] GEO::vecng<DIM, double> compute_cell_uvw_position(
            const GEO::index_t c,
            const GEO::vec3& uvw,
            const double* cur_control_nodes_ptr
            ) const {
            assert(c < this->control_nodes_mesh_.cells.nb());
            assert(uvw.x >= 0 && uvw.x <= 1);
            assert(uvw.y >= 0 && uvw.y <= 1);
            assert(uvw.z >= 0 && uvw.z <= 1);

            GEO::vecng<DIM, double> p(0, 0, 0);

            std::vector<double> Bu(this->order_+1);
            std::vector<double> Bv(this->order_+1);
            std::vector<double> Bw(this->order_+1);
            Lagrange_basis_1D(uvw.x, this->node_positions_1D_, Bu);
            Lagrange_basis_1D(uvw.y, this->node_positions_1D_, Bv);
            Lagrange_basis_1D(uvw.z, this->node_positions_1D_, Bw);

            for (GEO::index_t i = 0; i <= this->order_; ++i) {
                for (GEO::index_t j = 0; j <= this->order_; ++j) {
                    const double basis_uv = Bu[i] * Bv[j];
                    for (GEO::index_t k = 0; k <= this->order_; ++k) {
                        const auto& nd = this->cell_nd(c, i, j, k);
                        const double lag_basis = basis_uv * Bw[k];
                        p += lag_basis * GEO::vec3(
                            cur_control_nodes_ptr[3*nd],
                            cur_control_nodes_ptr[3*nd+1],
                            cur_control_nodes_ptr[3*nd+2]);
                    }
                }
            }

            return p;
        }

        /**
         * Evaluate the (non-unit) physical normal of a high-order cell facet.
         *
         * The normal is computed from the cross product of two facet tangents,
         * obtained by differentiating the facet mapping with respect to its local
         * parametric coordinates.
         *
         * @param[in] c Cell index, 0,1,...,hex_mesh.cells.nb()-1.
         * @param[in] lf Local facet index in the cell, 0,1,...,5.
         * @param[in] uv Facet-local parameter point `(u, v)` in [0, 1]^2.
         *          - u: facet-local parameter along the first facet axis (from cell_facet_vertex 0 to 1), typically in [0,1]
         *          - v: facet-local parameter along the second facet axis (from cell_facet_vertex 0 to 3), typically in [0,1]
         * @return Outward facet normal vector at the given parameter point in physical space.
         * @pre `c < hex_mesh.cells.nb()`.
         * @pre `lf < 6`.
         * @pre `parameter_point.x` and `parameter_point.y` are in [0, 1].
         * @note The returned vector is not normalized; its magnitude equals the local area scaling.
         */
        [[nodiscard]] GEO::vec3 compute_cell_facet_uv_normal(
            const GEO::index_t c,
            const GEO::index_t lf,
            const GEO::vec2& uv
            ) const {
            assert(c < this->control_nodes_mesh_.cells.nb());
            assert(lf < this->control_nodes_mesh_.cells.nb_facets(c));
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
                    const auto& p = GEO::Memory::pointer_as_reference<GEO::vec3>(
                        this->control_node_ptr(this->cell_facet_nd(c, lf, i, j)));
                    Tu += p * dBu[i] * Bv[j];
                    Tv += p * Bu[i] * dBv[j];
                }
            }
            return -GEO::cross(Tu, Tv); /* The orientation of the vertices of the cell facet is towards the interior of the
                cell, so the normal direction needs to be reversed. */
        }

        /**
         * Evaluate the first-order parametric derivatives of the cell mapping at a parameter point.
         *
         * This helper computes the physical-space tangent vectors with respect to the three parametric
         * directions, together with the corresponding 1D basis values and basis derivatives used in the
         * tensor-product evaluation.
         *
         * @param[in] c Cell index, 0,1,...,hex_mesh.cells.nb()-1.
         * @param[in] uvw Parameter point in the cell parameter domain [0, 1]^3.
         * @param[out] du Output tangent vector for partial derivative with respect to u.
         * @param[out] dv Output tangent vector for partial derivative with respect to v.
         * @param[out] dw Output tangent vector for partial derivative with respect to w.
         * @param[out] Bu Output buffer that receives the 1D basis values in the u direction.
         * @param[out] Bv Output buffer that receives the 1D basis values in the v direction.
         * @param[out] Bw Output buffer that receives the 1D basis values in the w direction.
         * @param[out] dBu Output buffer that receives the 1D basis derivatives in the u direction.
         * @param[out] dBv Output buffer that receives the 1D basis derivatives in the v direction.
         * @param[out] dBw Output buffer that receives the 1D basis derivatives in the w direction.
         */
        void compute_cell_uvw_dudvdw(
            const GEO::index_t c,
            const GEO::vec3& uvw,
            GEO::vecng<DIM, double>& du,
            GEO::vecng<DIM, double>& dv,
            GEO::vecng<DIM, double>& dw,
            std::vector<double>& Bu,
            std::vector<double>& Bv,
            std::vector<double>& Bw,
            std::vector<double>& dBu,
            std::vector<double>& dBv,
            std::vector<double>& dBw
            ) const {
            assert(c < this->control_nodes_mesh_.cells.nb());
            assert(uvw.x >= 0 && uvw.x <= 1);
            assert(uvw.y >= 0 && uvw.y <= 1);
            assert(uvw.z >= 0 && uvw.z <= 1);

            std::fill_n(du.data(), DIM, 0.0);
            std::fill_n(dv.data(), DIM, 0.0);
            std::fill_n(dw.data(), DIM, 0.0);

            Bu.resize(this->order_+1);
            Bv.resize(this->order_+1);
            Bw.resize(this->order_+1);
            dBu.resize(this->order_+1);
            dBv.resize(this->order_+1);
            dBw.resize(this->order_+1);
            Lagrange_basis_1D(uvw.x, this->node_positions_1D_, Bu);
            Lagrange_basis_1D(uvw.y, this->node_positions_1D_, Bv);
            Lagrange_basis_1D(uvw.z, this->node_positions_1D_, Bw);
            Lagrange_basis_deriv_1D(uvw.x, this->node_positions_1D_, dBu);
            Lagrange_basis_deriv_1D(uvw.y, this->node_positions_1D_, dBv);
            Lagrange_basis_deriv_1D(uvw.z, this->node_positions_1D_, dBw);

            for (GEO::index_t k = 0; k <= this->order_; ++k) {
                for (GEO::index_t j = 0; j <= this->order_; ++j) {
                    const double basis_vw = Bv[j] * Bw[k];
                    const double basis_dvw= dBv[j] * Bw[k];
                    const double basis_vdw= Bv[j] * dBw[k];
                    for (GEO::index_t i = 0; i <= this->order_; ++i) {
                        const double lag_basis_duvw = dBu[i] * basis_vw;
                        const double lag_basis_udvw = Bu[i] * basis_dvw;
                        const double lag_basis_uvdw = Bu[i] * basis_vdw;
                        du += lag_basis_duvw * this->control_node(this->cell_nd(c, i, j, k));
                        dv += lag_basis_udvw * this->control_node(this->cell_nd(c, i, j, k));
                        dw += lag_basis_uvdw * this->control_node(this->cell_nd(c, i, j, k));
                    }
                }
            }
        }

        /**
         * Evaluate the Jacobian matrix of the cell mapping at a parameter point.
         *
         * The Jacobian is a 3x3 matrix containing the partial derivatives of the physical
         * position with respect to the three parametric directions (u, v, w).
         *
         * @param[in] c Cell index, 0,1,...,hex_mesh.cells.nb()-1.
         * @param[in] uvw Parameter point in the cell parameter domain [0, 1]^3.
         * @param[out] J Output 3x3 Jacobian matrix at the given parameter point.
         *               Entry J(i,j) represents \f$\partial x_i / \partial u_j\f$ where:
         *               - columns 0, 1, 2 correspond to u, v, w derivatives
         *               - rows 0, 1, 2 correspond to x, y, z physical coordinates
         */
        void compute_cell_uvw_Jacobian(
            const GEO::index_t c,
            const GEO::vec3& uvw,
            Eigen::Matrix3d& J
            ) const {
            assert(c < this->control_nodes_mesh_.cells.nb());
            assert(uvw.x >= 0 && uvw.x <= 1);
            assert(uvw.y >= 0 && uvw.y <= 1);
            assert(uvw.z >= 0 && uvw.z <= 1);

            GEO::vecng<DIM, double> du, dv, dw;
            std::vector<double> Bu, Bv, Bw, dBu, dBv, dBw;
            compute_cell_uvw_dudvdw(c, uvw, du, dv, dw, Bu, Bv, Bw, dBu, dBv, dBw);

            J(0, 0) = du[0];
            J(1, 0) = du[1];
            J(2, 0) = du[2];
            J(0, 1) = dv[0];
            J(1, 1) = dv[1];
            J(2, 1) = dv[2];
            J(0, 2) = dw[0];
            J(1, 2) = dw[1];
            J(2, 2) = dw[2];
        }

        enum class MeasureType {
            DET_JACOBIAN,             // Signed Jacobian determinant; non-positive values indicate inversion or degeneration.
            SCALED_JACOBIAN,      // Skew measure / normalized Jacobian; 1.0 is ideal, and non-positive values indicate collapse or inversion.
            INVERSE_MEAN_RATIO,   // Shape-quality metric combining angle and aspect-ratio distortion; 1.0 is best and 0 indicates degeneration.
            MIPS                  // Minimizes shear and anisotropic stretching; 1.0 is best and the value grows toward infinity near degeneration.
        };

        /**
         * Evaluate a geometric quality metric at a point in the cell parameter domain.
         *
         * @param[in] c cell index, 0,1,...,hex_mesh.cells.nb()-1
         * @param[in] uvw parameter point in the cell parameter domain [0, 1]^3
         * @param[in] quality_type quality metric to evaluate:
         *                       - `QualityType::JACOBIAN`: signed Jacobian determinant, [-inf, inf]
         *                       - `QualityType::SCALED_JACOBIAN`: normalized Jacobian / skew measure, [-1, 1], best: 1
         *                       - `QualityType::INVERSE_MEAN_RATIO`: shape quality measure based on Jacobian and metric lengths, [0, 1], best: 1
         *                       - `QualityType::MIPS`: penalizes shear and anisotropic stretching, [1, inf], best: 1
         * @return the requested quality value at the given parameter point
         */
        [[nodiscard]] double compute_cell_uvw_measure(
            const GEO::index_t c,
            const GEO::vec3& uvw,
            const MeasureType quality_type
            ) const {
            assert(c < this->control_nodes_mesh_.cells.nb());
            assert(uvw.x >= 0 && uvw.x <= 1);
            assert(uvw.y >= 0 && uvw.y <= 1);
            assert(uvw.z >= 0 && uvw.z <= 1);

            GEO::vecng<DIM, double> du_all, dv_all, dw_all;
            std::vector<double> Bu, Bv, Bw, dBu, dBv, dBw;
            compute_cell_uvw_dudvdw(c, uvw, du_all, dv_all, dw_all, Bu, Bv, Bw, dBu, dBv, dBw);

            const GEO::vec3 du = GEO::Memory::pointer_as_reference<GEO::vec3>(du_all.data());
            const GEO::vec3 dv = GEO::Memory::pointer_as_reference<GEO::vec3>(dv_all.data());
            const GEO::vec3 dw = GEO::Memory::pointer_as_reference<GEO::vec3>(dw_all.data());
            const double det_J = GEO::dot(dw, GEO::cross(du,dv));
            switch (quality_type) {
                case MeasureType::DET_JACOBIAN: {
                    return det_J;
                }
                case MeasureType::MIPS: {
                    const double F_sq_norm = du.length2()+dv.length2()+dw.length2();
                    return F_sq_norm / (3.0*std::cbrt(det_J*det_J));
                }
                case MeasureType::SCALED_JACOBIAN: {
                    return det_J/(du.length()*dv.length()*dw.length());
                }
                case MeasureType::INVERSE_MEAN_RATIO: {
                    const double F_sq_norm = du.length2()+dv.length2()+dw.length2();
                    return 3.0*std::cbrt(det_J*det_J)/F_sq_norm;
                }
                default: assert(0);
            }
            return 0;
        }

        /**
         * Evaluate the gradient of the Jacobian determinant at a cell parameter point.
         *
         * The output stores \f$\partial\det(J)/\partial x\f$ with respect to all local control-point
         * coordinates of cell \p c, flattened in xyz order.
         *
         * @param[in] c Cell index, 0,1,...,hex_mesh.cells.nb()-1.
         * @param[in] uvw Parameter point in the cell parameter domain [0, 1]^3.
         * @param[out] gradient Output buffer of size `3 * control_points_nb_per_cell()`.
         *                      For local control point `N`, entries are:
         *                      - `gradient[3*N+0] = d(detJ)/dP_N.x`
         *                      - `gradient[3*N+1] = d(detJ)/dP_N.y`
         *                      - `gradient[3*N+2] = d(detJ)/dP_N.z`
         */
        void compute_cell_uvw_detJ_gradient(
            const GEO::index_t c,
            const GEO::vec3& uvw,
            std::vector<double>& gradient
            ) const {
            assert(c < this->control_nodes_mesh_.cells.nb());
            assert(uvw.x >= 0 && uvw.x <= 1);
            assert(uvw.y >= 0 && uvw.y <= 1);
            assert(uvw.z >= 0 && uvw.z <= 1);

            GEO::vecng<DIM, double> du_all, dv_all, dw_all;
            std::vector<double> Bu, Bv, Bw, dBu, dBv, dBw;
            compute_cell_uvw_dudvdw(c, uvw, du_all, dv_all, dw_all, Bu, Bv, Bw, dBu, dBv, dBw);

            const GEO::vec3 du = GEO::Memory::pointer_as_reference<GEO::vec3>(du_all.data());
            const GEO::vec3 dv = GEO::Memory::pointer_as_reference<GEO::vec3>(dv_all.data());
            const GEO::vec3 dw = GEO::Memory::pointer_as_reference<GEO::vec3>(dw_all.data());
            const GEO::vec3 cross_dvdw = GEO::cross(dv, dw);
            const GEO::vec3 cross_dwdu = GEO::cross(dw, du);
            const GEO::vec3 cross_dudv = GEO::cross(du, dv);

            gradient.resize(3*this->CONTROL_POINTS_NB_PER_CELL_);

            for (GEO::index_t k = 0; k < this->CONTROL_POINTS_NB_PER_EDGE_; ++k) {
                for (GEO::index_t j = 0; j < this->CONTROL_POINTS_NB_PER_EDGE_; ++j) {
                    const double basis_vw = Bv[j] * Bw[k];
                    const double basis_dvw= dBv[j] * Bw[k];
                    const double basis_vdw= Bv[j] * dBw[k];
                    for (GEO::index_t i = 0; i < this->CONTROL_POINTS_NB_PER_EDGE_; ++i) {
                        const double lag_basis_duvw = dBu[i] * basis_vw;
                        const double lag_basis_udvw = Bu[i] * basis_dvw;
                        const double lag_basis_uvdw = Bu[i] * basis_vdw;
                        const auto& lcv = this->cell_lnd(i, j, k);
                        const auto& g = lag_basis_duvw*cross_dvdw + lag_basis_udvw*cross_dwdu + lag_basis_uvdw*cross_dudv;
                        gradient[3*lcv] = g.x;
                        gradient[3*lcv+1] = g.y;
                        gradient[3*lcv+2] = g.z;
                    }
                }
            }
        }

        /**
         * @brief Compute the reference volume of every cell in the mesh.
         *
         * @param[out] volumes Output array that receives one reference volume per
         *            cell, in the same order as `hex_mesh_.cells`.
         * @note The caller is responsible for providing storage for all cells.
         */
        void compute_cells_volume(
            std::vector<double>& volumes
            ) const {
            volumes.resize(this->control_nodes_mesh_.cells.nb());

            /* 2k-1 >= 3*order-1  ->  k >= 1.5*order */
            std::vector<std::pair<GEO::vec3, double>> points_and_weights;
            geolio::get_Gauss_Legendre_quadrature_cube(std::ceil(1.5*this->order_), points_and_weights);

            for (const auto& c : this->control_nodes_mesh_.cells) {
                auto& V = volumes[c];
                V = 0;
                for (const auto& [uvw, w] : points_and_weights)
                    V += w * std::abs(compute_cell_uvw_measure(c, uvw, HexControlGrid::MeasureType::DET_JACOBIAN));
            }
        }

        /**
         * Assemble basis gradients at a parameter point for all local control points.
         * @param[in] uvw parameter point in [0,1]^3
         * @param[out] Bg gradient matrix of tensor-product basis values
         * @pre Bg.size == CONTROL_POINTS_NB_PER_CELL * 3
         */
        void compute_basis_gradient_matrix(
            const GEO::vec3& uvw,
            Eigen::MatrixXd& Bg
            ) const {
            assert(uvw.x >= 0 && uvw.x <= 1);
            assert(uvw.y >= 0 && uvw.y <= 1);
            assert(uvw.z >= 0 && uvw.z <= 1);
            assert(Bg.rows() == this->CONTROL_POINTS_NB_PER_CELL_);
            assert(Bg.cols() == 3);

            std::vector<double> Bu(this->order_+1);
            std::vector<double> Bv(this->order_+1);
            std::vector<double> Bw(this->order_+1);
            std::vector<double> dBu(this->order_+1);
            std::vector<double> dBv(this->order_+1);
            std::vector<double> dBw(this->order_+1);
            Lagrange_basis_1D(uvw.x, this->node_positions_1D_, Bu);
            Lagrange_basis_1D(uvw.y, this->node_positions_1D_, Bv);
            Lagrange_basis_1D(uvw.z, this->node_positions_1D_, Bw);
            Lagrange_basis_deriv_1D(uvw.x, this->node_positions_1D_, dBu);
            Lagrange_basis_deriv_1D(uvw.y, this->node_positions_1D_, dBv);
            Lagrange_basis_deriv_1D(uvw.z, this->node_positions_1D_, dBw);
            for (GEO::index_t i = 0; i < this->CONTROL_POINTS_NB_PER_EDGE_; ++i) {
                for (GEO::index_t j = 0; j < this->CONTROL_POINTS_NB_PER_EDGE_; ++j) {
                    const auto dBu_Bv = dBu[i]*Bv[j];
                    const auto Bu_dBv = Bu[i]*dBv[j];
                    const auto Bu_Bv = Bu[i]*Bv[j];
                    for (GEO::index_t k = 0; k < this->CONTROL_POINTS_NB_PER_EDGE_; ++k) {
                        const auto N = this->cell_lnd(i, j, k);
                        Bg(N, 0) = dBu_Bv*Bw[k];
                        Bg(N, 1) = Bu_dBv*Bw[k];
                        Bg(N, 2) = Bu_Bv*dBw[k];
                    }
                }
            }
        }

        /**
         * @brief Append a discretized surface mesh of all high-order cell facets (for visualization purposes).
         *
         * @param[in,out] mesh_out Output mesh that receives the discretized facets.
         * @param[in] resolution Number of samples per parametric direction on each facet.
         *                      Must be greater than 0; larger values produce finer tessellation.
         * @param[out] mesh_out_v_cell Optional vertex attribute storing the source cell index
         *                             for each generated output vertex.
         * @param[out] mesh_out_v_uvw Optional vertex attribute storing the corresponding
         *                            parametric coordinate of each generated output vertex.
         * @param[out] mesh_out_f_cell Optional face attribute storing the source cell index
         *                             for each generated output facet.
         */
        void append_discretized_high_order_cells_border(
            GEO::Mesh& mesh_out,
            GEO::index_t resolution = 10,
            GEO::Attribute<GEO::index_t>* mesh_out_v_cell = nullptr,
            GEO::Attribute<GEO::vec3>* mesh_out_v_uvw = nullptr,
            GEO::Attribute<GEO::index_t>* mesh_out_f_cell = nullptr
            ) const {
            if (mesh_out_v_cell != nullptr) {
                assert(mesh_out_v_cell->is_bound());
                assert(mesh_out_v_cell->size() == mesh_out.vertices.nb());
            }
            if (mesh_out_v_uvw != nullptr) {
                assert(mesh_out_v_uvw->is_bound());
                assert(mesh_out_v_uvw->size() == mesh_out.vertices.nb());
            }
            if (mesh_out_f_cell != nullptr) {
                assert(mesh_out_f_cell->is_bound());
                assert(mesh_out_f_cell->size() == mesh_out.facets.nb());
            }

            const GEO::index_t VERTICES_NB_PER_EDGE = resolution+1;

            GEO::index_t new_v = mesh_out.vertices.create_vertices(6*this->control_nodes_mesh_.cells.nb() * (resolution+1) * (resolution+1));
            GEO::index_t new_f = mesh_out.facets.create_quads(6*this->control_nodes_mesh_.cells.nb() * resolution * resolution);
            for (const auto& c : this->control_nodes_mesh_.cells) {
                for (GEO::index_t lf = 0; lf < 6; ++lf) {
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

                            const GEO::vec3 uvw = project_hex_lf_uv_to_uvw(GEO::vec2(u,v), lf);

                            const auto& p = compute_cell_uvw_position(c, uvw);
                            std::copy_n(p.data(), this->mesh_v_dim_, mesh_out.vertices.point_ptr(new_v));

                            if (mesh_out_v_cell != nullptr)
                                (*mesh_out_v_cell)[new_v] = c;
                            if (mesh_out_v_uvw != nullptr)
                                (*mesh_out_v_uvw)[new_v] = uvw;

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

                            if (mesh_out_f_cell != nullptr)
                                (*mesh_out_f_cell)[new_f] = c;

                            ++new_f;
                        }
                    }
                }
            }

            mesh_out.facets.connect();
        }

        /**
         * @brief Append a discretized volumetric mesh of all high-order cells (for visualization purposes).
         *
         * The routine samples each hexahedral cell with a regular
         * `resolution x resolution x resolution` grid in parametric space and appends
         * the generated volume elements to \p mesh_out.
         *
         * @param[in,out] mesh_out Output mesh that receives the discretized cells.
         * @param[in] resolution Number of samples per parametric direction inside each cell.
         *                      Must be greater than 0; larger values produce finer subdivision.
         * @param[out] mesh_out_v_cell Optional vertex attribute storing the source cell index
         *                             for each generated output vertex.
         * @param[out] mesh_out_v_uvw Optional vertex attribute storing the corresponding
         *                            parametric coordinate of each generated output vertex.
         * @param[out] mesh_out_c_cell Optional cell attribute storing the source cell index
         *                             for each generated output volume element.
         */
        void append_discretized_high_order_cells(
            GEO::Mesh& mesh_out,
            GEO::index_t resolution = 10,
            GEO::Attribute<GEO::index_t>* mesh_out_v_cell = nullptr,
            GEO::Attribute<GEO::vec3>* mesh_out_v_uvw = nullptr,
            GEO::Attribute<GEO::index_t>* mesh_out_c_cell = nullptr
            ) const {
            if (mesh_out_v_cell != nullptr) {
                assert(mesh_out_v_cell->is_bound());
                assert(mesh_out_v_cell->size() == mesh_out.vertices.nb());
            }
            if (mesh_out_v_uvw != nullptr) {
                assert(mesh_out_v_uvw->is_bound());
                assert(mesh_out_v_uvw->size() == mesh_out.vertices.nb());
            }
            if (mesh_out_c_cell != nullptr) {
                assert(mesh_out_c_cell->is_bound());
                assert(mesh_out_c_cell->size() == mesh_out.cells.nb());
            }

            const GEO::index_t VERTICES_NB_PER_EDGE = resolution+1;
            const GEO::index_t VERTICES_NB_PER_FACET = VERTICES_NB_PER_EDGE * VERTICES_NB_PER_EDGE;
            mesh_out.vertices.set_dimension(DIM);
            GEO::index_t new_v = mesh_out.vertices.create_vertices(this->control_nodes_mesh_.cells.nb() * (resolution+1) * (resolution+1) * (resolution+1));
            GEO::index_t new_c = mesh_out.cells.create_hexes(this->control_nodes_mesh_.cells.nb() * resolution * resolution * resolution);
            for (const auto& c : this->control_nodes_mesh_.cells) {
                const auto PREV_M_VERTICES = new_v;

                /* Vertices */
                for (GEO::index_t i = 0; i < VERTICES_NB_PER_EDGE; ++i) {
                    const double u = static_cast<double>(i)/resolution;
                    for (GEO::index_t j = 0; j < VERTICES_NB_PER_EDGE; ++j) {
                        const double v = static_cast<double>(j)/resolution;
                        for (GEO::index_t k = 0; k < VERTICES_NB_PER_EDGE; ++k) {
                            const double w = static_cast<double>(k)/resolution;
                            const GEO::vec3 uvw(u, v, w);

                            mesh_out.vertices.point<DIM>(new_v) = compute_cell_uvw_position(c, uvw);

                            if (mesh_out_v_cell != nullptr)
                                (*mesh_out_v_cell)[new_v] = c;
                            if (mesh_out_v_uvw != nullptr)
                                (*mesh_out_v_uvw)[new_v] = uvw;

                            ++new_v;
                        }
                    }
                }

                /* Cells
                 *    +Z                4-------6
                 *    |                /|      /|
                 *    o --- +Y        5-------7 |
                 *   /                | 0-----|-2
                 *  +X                |/      |/
                 *                    1-------3
                 */
                for (GEO::index_t i = 0; i < resolution; ++i) {
                    for (GEO::index_t j = 0; j < resolution; ++j) {
                        for (GEO::index_t k = 0; k < resolution; ++k) {
                            const GEO::index_t v0 = VERTICES_NB_PER_FACET*i+VERTICES_NB_PER_EDGE*j+k;
                            const GEO::index_t v1 = v0+VERTICES_NB_PER_FACET;
                            const GEO::index_t v2 = v0+VERTICES_NB_PER_EDGE;
                            const GEO::index_t v3 = v1+VERTICES_NB_PER_EDGE;
                            const GEO::index_t v4 = v0+1;
                            const GEO::index_t v5 = v4+VERTICES_NB_PER_FACET;
                            const GEO::index_t v6 = v4+VERTICES_NB_PER_EDGE;
                            const GEO::index_t v7 = v5+VERTICES_NB_PER_EDGE;
                            assert(v0 < mesh_out.vertices.nb());
                            assert(v1 < mesh_out.vertices.nb());
                            assert(v2 < mesh_out.vertices.nb());
                            assert(v3 < mesh_out.vertices.nb());
                            assert(v4 < mesh_out.vertices.nb());
                            assert(v5 < mesh_out.vertices.nb());
                            assert(v6 < mesh_out.vertices.nb());
                            assert(v7 < mesh_out.vertices.nb());
                            mesh_out.cells.set_vertex(new_c, 0, PREV_M_VERTICES+v0);
                            mesh_out.cells.set_vertex(new_c, 1, PREV_M_VERTICES+v1);
                            mesh_out.cells.set_vertex(new_c, 2, PREV_M_VERTICES+v2);
                            mesh_out.cells.set_vertex(new_c, 3, PREV_M_VERTICES+v3);
                            mesh_out.cells.set_vertex(new_c, 4, PREV_M_VERTICES+v4);
                            mesh_out.cells.set_vertex(new_c, 5, PREV_M_VERTICES+v5);
                            mesh_out.cells.set_vertex(new_c, 6, PREV_M_VERTICES+v6);
                            mesh_out.cells.set_vertex(new_c, 7, PREV_M_VERTICES+v7);

                            if (mesh_out_c_cell != nullptr)
                                (*mesh_out_c_cell)[new_c] = c;

                            ++new_c;
                        }
                    }
                }
            }

            mesh_out.cells.connect();
        }

    protected:
        /**
         * @brief Initialize local indexing/layout rules for hexahedral control nodes.
         */
        void initialize_nodes_arrangement() override {
            const GEO::index_t LAST_LAYER_BEGIN_IDX = (this->CONTROL_POINTS_NB_PER_EDGE_-1)*this->CONTROL_POINTS_NB_PER_FACET_;

            /* == Vertex =============================================================================================== */
            this->ELEMENT_VERTEX_CONTROL_POINTS_BEGIN_IDX_ = {
                0,
                this->CONTROL_POINTS_NB_PER_EDGE_-1,
                (this->CONTROL_POINTS_NB_PER_EDGE_-1)*this->CONTROL_POINTS_NB_PER_EDGE_,
                this->CONTROL_POINTS_NB_PER_FACET_-1,
                LAST_LAYER_BEGIN_IDX,
                LAST_LAYER_BEGIN_IDX+this->CONTROL_POINTS_NB_PER_EDGE_-1,
                LAST_LAYER_BEGIN_IDX+(this->CONTROL_POINTS_NB_PER_EDGE_-1)*this->CONTROL_POINTS_NB_PER_EDGE_,
                LAST_LAYER_BEGIN_IDX+this->CONTROL_POINTS_NB_PER_FACET_-1
            };

            /* == Edge ================================================================================================= */
            this->ELEMENT_EDGE_CONTROL_POINTS_BEGIN_IDX_ = {
                0,
                this->CONTROL_POINTS_NB_PER_EDGE_-1,
                this->CONTROL_POINTS_NB_PER_FACET_-1,
                (this->CONTROL_POINTS_NB_PER_EDGE_-1)*this->CONTROL_POINTS_NB_PER_EDGE_,
                LAST_LAYER_BEGIN_IDX,
                LAST_LAYER_BEGIN_IDX+this->CONTROL_POINTS_NB_PER_EDGE_-1,
                LAST_LAYER_BEGIN_IDX+this->CONTROL_POINTS_NB_PER_FACET_-1,
                LAST_LAYER_BEGIN_IDX+(this->CONTROL_POINTS_NB_PER_EDGE_-1)*this->CONTROL_POINTS_NB_PER_EDGE_,
                0,
                this->CONTROL_POINTS_NB_PER_EDGE_-1,
                this->CONTROL_POINTS_NB_PER_FACET_-1,
                (this->CONTROL_POINTS_NB_PER_EDGE_-1)*this->CONTROL_POINTS_NB_PER_EDGE_
            };
            this->ELEMENT_EDGE_CONTROL_POINTS_NEXT_IDX_STEP_ = {
                1,
                static_cast<int>(this->CONTROL_POINTS_NB_PER_EDGE_),
                -1,
                -static_cast<int>(this->CONTROL_POINTS_NB_PER_EDGE_),
                1,
                static_cast<int>(this->CONTROL_POINTS_NB_PER_EDGE_),
                -1,
                -static_cast<int>(this->CONTROL_POINTS_NB_PER_EDGE_),
                static_cast<int>(this->CONTROL_POINTS_NB_PER_FACET_),
                static_cast<int>(this->CONTROL_POINTS_NB_PER_FACET_),
                static_cast<int>(this->CONTROL_POINTS_NB_PER_FACET_),
                static_cast<int>(this->CONTROL_POINTS_NB_PER_FACET_)
            };
            this->ELEMENT_EDGE_INTERNAL_CONTROL_POINTS_BEGIN_IDX_ = {
                1,
                2*this->CONTROL_POINTS_NB_PER_EDGE_-1,
                this->CONTROL_POINTS_NB_PER_FACET_-2,
                (this->CONTROL_POINTS_NB_PER_EDGE_-2)*this->CONTROL_POINTS_NB_PER_EDGE_,
                LAST_LAYER_BEGIN_IDX+1,
                LAST_LAYER_BEGIN_IDX+2*this->CONTROL_POINTS_NB_PER_EDGE_-1,
                LAST_LAYER_BEGIN_IDX+this->CONTROL_POINTS_NB_PER_FACET_-2,
                LAST_LAYER_BEGIN_IDX+(this->CONTROL_POINTS_NB_PER_EDGE_-2)*this->CONTROL_POINTS_NB_PER_EDGE_,
                this->CONTROL_POINTS_NB_PER_FACET_,
                this->CONTROL_POINTS_NB_PER_EDGE_-1+this->CONTROL_POINTS_NB_PER_FACET_,
                this->CONTROL_POINTS_NB_PER_FACET_-1+this->CONTROL_POINTS_NB_PER_FACET_,
                (this->CONTROL_POINTS_NB_PER_EDGE_-1)*this->CONTROL_POINTS_NB_PER_EDGE_+this->CONTROL_POINTS_NB_PER_FACET_
            };
            this->ELEMENT_EDGE_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP_ = this->ELEMENT_EDGE_CONTROL_POINTS_NEXT_IDX_STEP_;

            /* == Facet ================================================================================================ */
            this->ELEMENT_FACET_CONTROL_POINTS_BEGIN_IDX_ = {
                0,
                this->CONTROL_POINTS_NB_PER_FACET_-1,
                this->CONTROL_POINTS_NB_PER_EDGE_-1,
                (this->CONTROL_POINTS_NB_PER_EDGE_-1)*this->CONTROL_POINTS_NB_PER_EDGE_,
                this->CONTROL_POINTS_NB_PER_EDGE_-1,
                LAST_LAYER_BEGIN_IDX
            };
            this->ELEMENT_FACET_CONTROL_POINTS_NEXT_IDX_STEP0_ = {
                static_cast<int>(this->CONTROL_POINTS_NB_PER_EDGE_),
                -static_cast<int>(this->CONTROL_POINTS_NB_PER_EDGE_),
                -1,
                1,
                static_cast<int>(this->CONTROL_POINTS_NB_PER_EDGE_),
                static_cast<int>(this->CONTROL_POINTS_NB_PER_EDGE_)
            };
            this->ELEMENT_FACET_CONTROL_POINTS_NEXT_IDX_STEP1_ = {
                static_cast<int>(this->CONTROL_POINTS_NB_PER_FACET_),
                static_cast<int>(this->CONTROL_POINTS_NB_PER_FACET_),
                static_cast<int>(this->CONTROL_POINTS_NB_PER_FACET_),
                static_cast<int>(this->CONTROL_POINTS_NB_PER_FACET_),
                -1,
                1
            };
            this->ELEMENT_FACET_INTERNAL_CONTROL_POINTS_BEGIN_IDX_ = {
                this->CONTROL_POINTS_NB_PER_EDGE_+this->CONTROL_POINTS_NB_PER_FACET_,
                (this->CONTROL_POINTS_NB_PER_EDGE_-1)*this->CONTROL_POINTS_NB_PER_EDGE_-1+this->CONTROL_POINTS_NB_PER_FACET_,
                this->CONTROL_POINTS_NB_PER_EDGE_-2+this->CONTROL_POINTS_NB_PER_FACET_,
                (this->CONTROL_POINTS_NB_PER_EDGE_-1)*this->CONTROL_POINTS_NB_PER_EDGE_+1+this->CONTROL_POINTS_NB_PER_FACET_,
                2*this->CONTROL_POINTS_NB_PER_EDGE_-2,
                LAST_LAYER_BEGIN_IDX+this->CONTROL_POINTS_NB_PER_EDGE_+1
            };
            this->ELEMENT_FACET_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP0_ = this->ELEMENT_FACET_CONTROL_POINTS_NEXT_IDX_STEP0_;
            this->ELEMENT_FACET_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP1_ = this->ELEMENT_FACET_CONTROL_POINTS_NEXT_IDX_STEP1_;

            /* == Cell ================================================================================================= */
            this->ELEMENT_CONTROL_POINTS_BEGIN_IDX_ = 0;
            this->ELEMENT_CONTROL_POINTS_NEXT_IDX_STEP0_ = 1;
            this->ELEMENT_CONTROL_POINTS_NEXT_IDX_STEP1_ = static_cast<int>(this->CONTROL_POINTS_NB_PER_EDGE_);
            this->ELEMENT_CONTROL_POINTS_NEXT_IDX_STEP2_ = static_cast<int>(this->CONTROL_POINTS_NB_PER_FACET_);
            this->ELEMENT_INTERNAL_CONTROL_POINTS_BEGIN_IDX_ = this->CONTROL_POINTS_NB_PER_EDGE_+1+this->CONTROL_POINTS_NB_PER_FACET_;
            this->ELEMENT_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP0_ = this->ELEMENT_CONTROL_POINTS_NEXT_IDX_STEP0_;
            this->ELEMENT_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP1_ = this->ELEMENT_CONTROL_POINTS_NEXT_IDX_STEP1_;
            this->ELEMENT_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP2_ = this->ELEMENT_CONTROL_POINTS_NEXT_IDX_STEP2_;
        }

        /**
         * @brief Build global control-node coordinates and cell-to-control-node connectivity.
         */
        void initialize_control_nodes(const GEO::Mesh& mesh) override {
            assert(this->node_positions_1D_.size() == this->order_+1);

            /* == Get all shared edges and facets ================================================================== */
            GEO::index_t hex_facets_nb = 0;
            std::unordered_map<std::pair<GEO::index_t, GEO::index_t>, std::vector<GEO::index_t>, PairHash> hex_edges_control_points; /* (ev0, ev1), ev0 < ev1 -> control vertices from ev0 -> ev1 */
            {
                std::vector<bool> processed_hex_cf(8*mesh.cells.nb(), false); // only need the first 6 facets
                for (const auto& c : mesh.cells) {
                    /* For all facets */
                    for (GEO::index_t lf = 0; lf < 6; ++lf) {
                        if (processed_hex_cf[8*c+lf]) // this facet is already been processed
                            continue;

                        ++hex_facets_nb;

                        processed_hex_cf[8*c+lf] = true;
                        if (const auto nc = mesh.cells.adjacent(c, lf);
                            nc != GEO::NO_CELL) {
                            const auto nlf = find_hex_facet(
                                mesh,
                                nc,
                                mesh.cells.facet_vertex(c, lf, 2),
                                mesh.cells.facet_vertex(c, lf, 1),
                                mesh.cells.facet_vertex(c, lf, 0));
                            assert(nlf != GEO::NO_INDEX);
                            processed_hex_cf[8*nc+nlf] = true;
                        }
                    }

                    /* For all edges */
                    for (GEO::index_t le = 0; le < 12; ++le) {
                        const std::pair<GEO::index_t, GEO::index_t> edge = std::minmax(
                            mesh.cells.edge_vertex(c, le, 0),
                            mesh.cells.edge_vertex(c, le, 1));
                        hex_edges_control_points.emplace(edge, std::vector<GEO::index_t>(this->INTERNAL_CONTROL_POINTS_NB_PER_EDGE_, GEO::NO_VERTEX));
                    }
                }
            }

            // LOG::DEBUG("found {} facets and {} edges in the hex mesh", hex_facets_nb, hex_edges_control_points.size());

            /* == Create grid vertices ============================================================================= */
            this->control_nodes_mesh_.vertices.clear();
            GEO::index_t new_v = this->control_nodes_mesh_.vertices.create_vertices(
                                mesh.vertices.nb() + // vertices
                                hex_edges_control_points.size() * this->INTERNAL_CONTROL_POINTS_NB_PER_EDGE_ + // edges
                                hex_facets_nb * this->INTERNAL_CONTROL_POINTS_NB_PER_FACET_ + // facets
                                mesh.cells.nb() * this->INTERNAL_CONTROL_POINTS_NB_PER_CELL_ // cells
                                );
            assert(new_v == 0);

            /* == For vertices == */
            for (const auto& v : mesh.vertices)
                std::copy_n(mesh.vertices.point_ptr(v), this->mesh_v_dim_, this->control_node_ptr(new_v++));

            /* == For edges == */
            for (auto& [edge, control_vertices] : hex_edges_control_points) {
                GEO::vecng<DIM, double> ep0, ep1;
                std::copy_n(mesh.vertices.point_ptr(edge.first), this->mesh_v_dim_, ep0.data());
                std::copy_n(mesh.vertices.point_ptr(edge.second), this->mesh_v_dim_, ep1.data());
                for (GEO::index_t i = 0; i < this->INTERNAL_CONTROL_POINTS_NB_PER_EDGE_; ++i) {
                    const double r = this->node_positions_1D_[i+1];
                    this->control_node(new_v) = (1-r)*ep0 + r*ep1;
                    control_vertices[i] = new_v;
                    ++new_v;
                }
            }

            /* == For facets == */
            std::vector<std::vector<GEO::index_t>> hex_facets_control_points(8*mesh.cells.nb()); /*
                [8*c+lf] -> the idx of the control points of this cell facet,
                            from fv0 -> fv1, ..., fv3 -> fv2 */
            for (const auto& c : mesh.cells) {
                assert(mesh.cells.nb_facets(c) == 6);
                for (GEO::index_t lf = 0; lf < 6; ++lf) {
                    if (!hex_facets_control_points[8*c+lf].empty())
                        continue;

                    assert(mesh.cells.facet_nb_vertices(c, lf) == 4);
                    const auto& lf_v0 = mesh.cells.facet_vertex(c, lf, 0);
                    const auto& lf_v1 = mesh.cells.facet_vertex(c, lf, 1);
                    const auto& lf_v2 = mesh.cells.facet_vertex(c, lf, 2);
                    const auto& lf_v3 = mesh.cells.facet_vertex(c, lf, 3);
                    GEO::vecng<DIM, double> lf_p0, lf_p1, lf_p2, lf_p3;
                    std::copy_n(mesh.vertices.point_ptr(lf_v0), this->mesh_v_dim_, lf_p0.data());
                    std::copy_n(mesh.vertices.point_ptr(lf_v1), this->mesh_v_dim_, lf_p1.data());
                    std::copy_n(mesh.vertices.point_ptr(lf_v2), this->mesh_v_dim_, lf_p2.data());
                    std::copy_n(mesh.vertices.point_ptr(lf_v3), this->mesh_v_dim_, lf_p3.data());

                    auto& lf_control_points = hex_facets_control_points[8*c+lf];
                    lf_control_points.reserve(this->INTERNAL_CONTROL_POINTS_NB_PER_FACET_);

                    for (GEO::index_t i = 0; i < this->INTERNAL_CONTROL_POINTS_NB_PER_EDGE_; ++i) {
                        const double ri = this->node_positions_1D_[i+1];
                        for (GEO::index_t j = 0; j < this->INTERNAL_CONTROL_POINTS_NB_PER_EDGE_; ++j) {
                            const double rj = this->node_positions_1D_[j+1];

                            this->control_node(new_v) = (1-ri)*(1-rj)*lf_p0
                                                + ri*(1-rj)*lf_p3
                                                + (1-ri)*rj*lf_p1
                                                + ri*rj*lf_p2;
                            lf_control_points.push_back(new_v);
                            ++new_v;
                        }
                    }

                    /* Assign to adjacent cell facet */
                    if (const auto& nc = mesh.cells.adjacent(c, lf);
                        nc != GEO::NO_CELL) {
                        const auto nlf = find_hex_facet(
                            mesh,
                            nc,
                            mesh.cells.facet_vertex(c, lf, 2),
                            mesh.cells.facet_vertex(c, lf, 1),
                            mesh.cells.facet_vertex(c, lf, 0));
                        assert(nlf != GEO::NO_INDEX);

                        auto& nclf_control_points = hex_facets_control_points[8*nc+nlf];
                        nclf_control_points.reserve(this->INTERNAL_CONTROL_POINTS_NB_PER_FACET_);

                        assert(mesh.cells.facet_nb_vertices(nc, nlf) == 4);
                        if (const auto& nclf_v0 = mesh.cells.facet_vertex(nc, nlf, 0);
                            nclf_v0 == lf_v0
                            ) {
                            for (GEO::index_t i = 0, i_end = this->order_-1; i < i_end; ++i) {
                                for (GEO::index_t j = 0, j_end = this->order_-1; j < j_end; ++j)
                                    nclf_control_points.push_back(lf_control_points[i+j*(this->order_-1)]);
                            }
                        }
                        else if (nclf_v0 == lf_v1) {
                            for (GEO::index_t i = 0, i_end = this->order_-1; i < i_end; ++i) {
                                for (GEO::index_t j = 0, j_end = this->order_-1; j < j_end; ++j)
                                    nclf_control_points.push_back(lf_control_points[(this->order_-1)*(i+1)-1-j]);
                            }
                        }
                        else if (nclf_v0 == lf_v2) {
                            for (GEO::index_t i = 0, i_end = this->order_-1; i < i_end; ++i) {
                                for (GEO::index_t j = 0, j_end = this->order_-1; j < j_end; ++j)
                                    nclf_control_points.push_back(lf_control_points[(this->order_-1)*(this->order_-1)-1-i-(this->order_-1)*j]);
                            }
                        }
                        else if (nclf_v0 == lf_v3) {
                            for (GEO::index_t i = 0, i_end = this->order_-1; i < i_end; ++i) {
                                for (GEO::index_t j = 0, j_end = this->order_-1; j < j_end; ++j)
                                    nclf_control_points.push_back(lf_control_points[(this->order_-1)*(this->order_-2-i)+j]);
                            }
                        }
                        else
                            assert(0);
                    }
                }
            }

            /* == For cells == */
            std::vector<std::vector<GEO::index_t>> hex_cells_control_points(mesh.cells.nb()); /*
                [c] -> the idx of control points of this cell
                       from cv0 -> cv1, cv2 -> cv3, ..., cv4 -> cv5, ..., cv6 -> cv7 */
            for (const auto& c : mesh.cells) {
                auto& c_control_points = hex_cells_control_points[c];
                c_control_points.reserve(this->INTERNAL_CONTROL_POINTS_NB_PER_CELL_);

                assert(mesh.cells.nb_vertices(c) == 8);
                GEO::vecng<DIM, double> c_p0, c_p1, c_p2, c_p3, c_p4, c_p5, c_p6, c_p7;
                std::copy_n(mesh.vertices.point_ptr(mesh.cells.vertex(c, 0)), this->mesh_v_dim_, c_p0.data());
                std::copy_n(mesh.vertices.point_ptr(mesh.cells.vertex(c, 1)), this->mesh_v_dim_, c_p1.data());
                std::copy_n(mesh.vertices.point_ptr(mesh.cells.vertex(c, 2)), this->mesh_v_dim_, c_p2.data());
                std::copy_n(mesh.vertices.point_ptr(mesh.cells.vertex(c, 3)), this->mesh_v_dim_, c_p3.data());
                std::copy_n(mesh.vertices.point_ptr(mesh.cells.vertex(c, 4)), this->mesh_v_dim_, c_p4.data());
                std::copy_n(mesh.vertices.point_ptr(mesh.cells.vertex(c, 5)), this->mesh_v_dim_, c_p5.data());
                std::copy_n(mesh.vertices.point_ptr(mesh.cells.vertex(c, 6)), this->mesh_v_dim_, c_p6.data());
                std::copy_n(mesh.vertices.point_ptr(mesh.cells.vertex(c, 7)), this->mesh_v_dim_, c_p7.data());
                for (GEO::index_t k = 0; k < this->INTERNAL_CONTROL_POINTS_NB_PER_EDGE_; ++k) {
                    const double rk = this->node_positions_1D_[k+1];
                    for (GEO::index_t j = 0; j < this->INTERNAL_CONTROL_POINTS_NB_PER_EDGE_; ++j) {
                        const double rj = this->node_positions_1D_[j+1];
                        for (GEO::index_t i = 0; i < this->INTERNAL_CONTROL_POINTS_NB_PER_EDGE_; ++i) {
                            const double ri = this->node_positions_1D_[i+1];
                            this->control_node(new_v) = (1-ri)*(1-rj)*(1-rk)*c_p0
                                                    + ri*(1-rj)*(1-rk)*c_p1
                                                    + (1-ri)*rj*(1-rk)*c_p2
                                                    + ri*rj*(1-rk)*c_p3
                                                    + (1-ri)*(1-rj)*rk*c_p4
                                                    + ri*(1-rj)*rk*c_p5
                                                    + (1-ri)*rj*rk*c_p6
                                                    + ri*rj*rk*c_p7;
                            c_control_points.push_back(new_v);
                            ++new_v;
                        }
                    }
                }
            }

            assert(new_v == this->control_nodes_nb());

            /* == Create grid cells ================================================================================ */
            this->control_nodes_mesh_.cells.create_hexes(mesh.cells.nb());
            for (const auto& c : mesh.cells) {
                for (GEO::index_t lv = 0; lv < 8; ++lv)
                    this->control_nodes_mesh_.cells.set_vertex(c, lv, mesh.cells.vertex(c, lv));
            }
            this->control_nodes_mesh_.cells.connect();

            /* Initialize other dimension */
            this->initialize_control_node_quantities();

            /* == Create regular index ================================================================================= */
            this->element_control_nodes_.create_vector_attribute(
                this->control_nodes_mesh_.cells.attributes(),
                "control_nodes",
                this->CONTROL_POINTS_NB_PER_CELL_);
            this->element_control_nodes_.fill(GEO::NO_INDEX);
            /* [(order+1)^3 * c + lv] -> hex cell c's control vertex lv
                For a hex (0, 1, 2, 3, 4, 5, 6, 7),

                  +Z                4-------6        lf-0: (0-2-6-4)     le-0:  (0-1)
                  |                /|      /|        lf-1: (3-1-5-7)     le-1:  (1-3)
                  o --- +Y        5-------7 |        lf-2: (1-0-4-5)     le-2:  (3-2)
                 /                | 0-----|-2        lf-3: (2-3-7-6)     le-3:  (2-0)
                +X                |/      |/         lf-4: (1-3-2-0)     le-4:  (4-5)
                                  1-------3          lf-5: (4-6-7-5)     le-5:  (5-7)
                                                                         le-6:  (7-6)
                                                                         le-7:  (6-4)
                                                                         le-8:  (0-4)
                                                                         le-9:  (1-5)
                                                                         le-10: (3-7)
                                                                         le-11: (2-6)

                the arrangement of the control points is:
                    dimension 1: cv0 -> cv1, dimension 2: cv0 -> cv2 ,dimension 3: cv0 -> cv4 */

            for (const auto& c : mesh.cells) {
                const GEO::index_t CELL_BEGIN_IDX = c*this->CONTROL_POINTS_NB_PER_CELL_;

                /* For vertices */
                for (GEO::index_t lv = 0; lv < 8; ++lv)
                    this->element_control_nodes_[
                        CELL_BEGIN_IDX +
                        this->cell_vertex_lnd(lv)
                        ] = mesh.cells.vertex(c, lv);

                /* For edges */
                for (GEO::index_t le = 0; le < 12; ++le) {
                    const auto& ev0 = mesh.cells.edge_vertex(c, le, 0);
                    const auto& ev1 = mesh.cells.edge_vertex(c, le, 1);
                    const std::pair<GEO::index_t, GEO::index_t> edge = std::minmax(ev0, ev1);

                    assert(hex_edges_control_points.contains(edge));
                    const auto& edge_control_points = hex_edges_control_points.at(edge);
                    assert(edge_control_points.size() == this->INTERNAL_CONTROL_POINTS_NB_PER_EDGE_);

                    if (ev0 == edge.first) { // do not need to inverse
                        for (GEO::index_t lv = 0; lv < this->INTERNAL_CONTROL_POINTS_NB_PER_EDGE_; ++lv)
                            this->element_control_nodes_[
                                CELL_BEGIN_IDX +
                                this->cell_edge_inner_lnd(le, lv)
                                ] = edge_control_points[lv];
                    }
                    else { // need to inverse
                        for (GEO::index_t lv = 0; lv < this->INTERNAL_CONTROL_POINTS_NB_PER_EDGE_; ++lv)
                            this->element_control_nodes_[
                                CELL_BEGIN_IDX +
                                this->cell_edge_inner_lnd(le, lv)
                                ] = edge_control_points[this->INTERNAL_CONTROL_POINTS_NB_PER_EDGE_-1-lv];
                    }
                }

                /* For facets */
                for (GEO::index_t lf = 0; lf < 6; ++lf) {
                    const auto& facet_control_points = hex_facets_control_points[8*c+lf];
                    assert(facet_control_points.size() == this->INTERNAL_CONTROL_POINTS_NB_PER_FACET_);

                    for (GEO::index_t lv1 = 0; lv1 < this->INTERNAL_CONTROL_POINTS_NB_PER_EDGE_; ++lv1) {
                        for (GEO::index_t lv0 = 0; lv0 < this->INTERNAL_CONTROL_POINTS_NB_PER_EDGE_; ++lv0)
                            this->element_control_nodes_[
                                CELL_BEGIN_IDX +
                                this->cell_facet_inner_lnd(lf, lv0, lv1)
                                ] = facet_control_points[lv1*this->INTERNAL_CONTROL_POINTS_NB_PER_EDGE_ + lv0];
                    }
                }

                /* For cells */
                const auto& cell_control_points = hex_cells_control_points[c];
                assert(cell_control_points.size() == this->INTERNAL_CONTROL_POINTS_NB_PER_CELL_);

                for (GEO::index_t lv2 = 0; lv2 < this->INTERNAL_CONTROL_POINTS_NB_PER_EDGE_; ++lv2) {
                    for (GEO::index_t lv1 = 0; lv1 < this->INTERNAL_CONTROL_POINTS_NB_PER_EDGE_; ++lv1) {
                        for (GEO::index_t lv0 = 0; lv0 < this->INTERNAL_CONTROL_POINTS_NB_PER_EDGE_; ++lv0) {
                            this->element_control_nodes_[
                                CELL_BEGIN_IDX +
                                this->cell_inner_lnd(lv0, lv1, lv2)
                                ] = cell_control_points[lv2*this->INTERNAL_CONTROL_POINTS_NB_PER_FACET_ + lv1*this->INTERNAL_CONTROL_POINTS_NB_PER_EDGE_ + lv0];
                        }
                    }
                }
            }
        }
    };

    template <typename T>
    struct isHexControlGrid : std::false_type {};

    template <GEO::index_t MESH_DIM, GEO::index_t QUANTITIES_DIM>
    struct isHexControlGrid<HexControlGrid<MESH_DIM, QUANTITIES_DIM>> : std::true_type {};
}

#endif //GEOLIO_HEX_CONTROL_GRID_H
