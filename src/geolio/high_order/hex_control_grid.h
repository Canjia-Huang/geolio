//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/2.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_HEX_CONTROL_GRID_H
#define GEOLIO_HEX_CONTROL_GRID_H
#include "volume_control_grid.h"
#include <Eigen/Dense>

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
     *         If `lv` is out of range, returns (-1,-1,-1).
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
     *         parameter. If `le` is out of range, returns (-1,-1,-1).
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

    class HexControlGrid : public VolumeControlGrid {
    public:
        /**
         * @brief Construct a hexahedral high-order control grid.
         * @param[in] mesh Input hexahedral mesh used as the reference topology/geometry.
         * @param[in] order Polynomial order of the tensor-product hexahedral mapping.
         */
        HexControlGrid(const GEO::Mesh& mesh, GEO::index_t order);

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
                GEO::index_t c,
                const GEO::vec3& uvw,
                Eigen::Matrix3d& J) const;

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
                GEO::index_t c,
                const GEO::vec3& uvw,
                MeasureType quality_type) const;

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
            GEO::index_t c,
            const GEO::vec3& uvw,
            std::vector<double>& gradient) const;

        /**
         * @brief Compute the reference volume of every cell in the mesh.
         *
         * @param[out] volumes Output array that receives one reference volume per
         *            cell, in the same order as `hex_mesh_.cells`.
         * @note The caller is responsible for providing storage for all cells.
         */
        void compute_cells_volume(std::vector<double>& volumes) const;

        /**
         * Assemble physical control-point coordinates of one cell into a dense matrix.
         * @param[in] c cell index, 0,1,...,hex_mesh.cells.nb()-1
         * @param[out] P matrix of control-point positions for cell \\p c
         */
        void compute_cell_vertices_position_matrix(
            GEO::index_t c,
            Eigen::MatrixXd& P);

        /**
         * Assemble basis gradients at a parameter point for all local control points.
         * @param[in] uvw parameter point in [0,1]^3
         * @param[out] Bg gradient matrix of tensor-product basis values
         * @pre Bg.size == CONTROL_POINTS_NB_PER_CELL * 3
         */
        void compute_basis_gradient_matrix(
            const GEO::vec3& uvw,
            Eigen::MatrixXd& Bg) const;

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
            GEO::Attribute<GEO::index_t>* mesh_out_f_cell = nullptr) const;

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
            GEO::Attribute<GEO::index_t>* mesh_out_c_cell = nullptr) const;

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
}

#endif //GEOLIO_HEX_CONTROL_GRID_H
