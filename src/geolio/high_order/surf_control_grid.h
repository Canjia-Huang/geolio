//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/2.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef HOSM_SURF_CONTROL_GRID_H
#define HOSM_SURF_CONTROL_GRID_H
#include "control_grid.h"

namespace geolio
{
    template <GEO::index_t DIM>
    class SurfaceControlGrid : public ControlGrid<DIM> {
    public:
        /**
         * @brief Construct a surface control grid.
         * @param[in] mesh Input surface mesh.
         * @param[in] order Polynomial order of the high-order surface representation.
         */
        SurfaceControlGrid(const GEO::Mesh& mesh, const GEO::index_t order)
            : ControlGrid<DIM>(mesh, order)
        {
            assert(this->mesh_.facets.nb() > 0);
        }

        /**
         * Get a local control node index by local vertex index.
         * @param[in] lv local vertex index in the facet, 0,1,...,mesh_.facets.nb_vertices
         * @return local control node index, 0,1,...,CONTROL_POINTS_NB_PER_FACET
         */
        [[nodiscard]] GEO::index_t facet_vertex_lnd(const GEO::index_t lv) const {
            assert(lv < this->mesh_.facets.nb_vertices(0));
            return this->ELEMENT_VERTEX_CONTROL_POINTS_BEGIN_IDX_[lv];
        }

        /**
         * Get a local control node index by local edge index and local vertex index in the edge.
         * @param[in] le local edge index, 0,1,...,mesh_.facets.nb_edges
         * @param[in] lv local vertex index in the edge (ev0 -> ev1), 0,1,...,order
         * @return local control node index, 0,1,...,CONTROL_POINTS_NB_PER_FACET
         */
        [[nodiscard]] GEO::index_t facet_edge_lnd(const GEO::index_t le, const GEO::index_t lv) const {
            assert(le < this->mesh_.facets.nb_vertices(0));
            assert(lv < this->CONTROL_POINTS_NB_PER_EDGE_);
            return this->ELEMENT_EDGE_CONTROL_POINTS_BEGIN_IDX_[le] + lv*this->ELEMENT_EDGE_CONTROL_POINTS_NEXT_IDX_STEP_[le];
        }

        /**
         * Get a local internal control node index by local edge index and local vertex index in the edge.
         * @param[in] le local edge index, 0,1,...,mesh_.facets.nb_edges
         * @param[in] lv local vertex index in the edge (ev0 -> ev1), 0,1,...,order-2
         * @return local control node index, 0,1,...,CONTROL_POINTS_NB_PER_FACET
         */
        [[nodiscard]] GEO::index_t facet_edge_inner_lnd(const GEO::index_t le, const GEO::index_t lv) const {
            assert(le < this->mesh_.facets.nb_vertices(0));
            assert(lv < this->INTERNAL_CONTROL_POINTS_NB_PER_EDGE_);
            return this->ELEMENT_EDGE_INTERNAL_CONTROL_POINTS_BEGIN_IDX_[le] + lv*this->ELEMENT_EDGE_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP_[le];
        }

        /**
         * Get a local control node index by two local vertex indexes in the facet
         * @param[in] lv0 local vertex index in the facet (cv0 -> cv1), 0,1,...,order
         * @param[in] lv1 local vertex index in the facet (cv0 -> cv2), 0,1,...,order
         * @return local control node index, 0,1,...,CONTROL_POINTS_NB_PER_FACET
         */
        [[nodiscard]] GEO::index_t facet_lnd(const GEO::index_t lv0, const GEO::index_t lv1) const {
            assert(lv0 < this->CONTROL_POINTS_NB_PER_EDGE_);
            assert(lv1 < this->CONTROL_POINTS_NB_PER_EDGE_);
            return this->ELEMENT_CONTROL_POINTS_BEGIN_IDX_ + lv0*this->ELEMENT_CONTROL_POINTS_NEXT_IDX_STEP0_ + lv1*this->ELEMENT_CONTROL_POINTS_NEXT_IDX_STEP1_;
        }

        /**
         * Get a local internal control node index by two local vertex indexes in the facet
         * @param[in] lv0 local vertex index in the facet (cv0 -> cv1), 0,1,...,order-2
         * @param[in] lv1 local vertex index in the facet (cv0 -> cv2), 0,1,...,order-2
         * @return local control node index, 0,1,...,CONTROL_POINTS_NB_PER_FACET
         */
        [[nodiscard]] GEO::index_t facet_inner_lnd(const GEO::index_t lv0, const GEO::index_t lv1) const {
            assert(lv0 < this->INTERNAL_CONTROL_POINTS_NB_PER_EDGE_);
            assert(lv1 < this->INTERNAL_CONTROL_POINTS_NB_PER_EDGE_);
            return this->ELEMENT_INTERNAL_CONTROL_POINTS_BEGIN_IDX_ + lv0*this->ELEMENT_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP0_ + lv1*this->ELEMENT_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP1_;
        }

        /**
         * Get a control node index by local vertex index in the facet.
         * @param[in] f facet index, 0,1,...,hex_mesh.facets.nb()-1
         * @param[in] lv local vertex index in the facet, 0,1,...,3
         * @return control node index
         */
        [[nodiscard]] GEO::index_t facet_vertex_nd(const GEO::index_t f, const GEO::index_t lv) const {
            assert(f < this->mesh_.facets.nb());
            return this->element_control_nodes_[f*this->CONTROL_POINTS_NB_PER_FACET_ + this->facet_vertex_lnd(lv)];
        }

        /**
         * Get a control node index by local edge index and local vertex index in the edge.
         * @param[in] f facet index, 0,1,...,hex_mesh.facets.nb()-1
         * @param[in] le local edge index, 0,1,...,3
         * @param[in] lv local vertex index in the edge (ev0 -> ev1), 0,1,...,order
         * @return control node index
         */
        [[nodiscard]] GEO::index_t facet_edge_nd(const GEO::index_t f, const GEO::index_t le, const GEO::index_t lv) const {
            assert(f < this->mesh_.facets.nb());
            return this->element_control_nodes_[f*this->CONTROL_POINTS_NB_PER_FACET_ + this->facet_edge_lnd(le, lv)];
        }

        /**
         * Get an internal control node index by local edge index and local vertex index in the edge.
         * @param[in] f facet index, 0,1,...,hex_mesh.facets.nb()-1
         * @param[in] le local edge index, 0,1,...,3
         * @param[in] lv local vertex index in the edge (ev0 -> ev1), 0,1,...,order-2
         * @return control node index
         */
        [[nodiscard]] GEO::index_t facet_edge_inner_nd(const GEO::index_t f, const GEO::index_t le, const GEO::index_t lv) const {
            assert(f < this->mesh_.facets.nb());
            return this->element_control_nodes_[f*this->CONTROL_POINTS_NB_PER_FACET_ + this->facet_edge_inner_lnd(le, lv)];
        }

        /**
         * Get a control node index by flattened local control-point index in one facet.
         * @param[in] f facet index, 0,1,...,hex_mesh.facets.nb()-1
         * @param[in] lv flattened local control-point index, 0,1,...,CONTROL_POINTS_NB_PER_FACET-1
         * @return control node index
         */
        [[nodiscard]] GEO::index_t facet_nd(const GEO::index_t f, const GEO::index_t lv) const {
            assert(f < this->mesh_.facets.nb());
            assert(lv < this->CONTROL_POINTS_NB_PER_FACET_);
            return this->element_control_nodes_[f*this->CONTROL_POINTS_NB_PER_FACET_ + lv];
        }

        /**
         * Get a control node index by two local vertex indexes in the facet
         * @param[in] f facet index, 0,1,...,hex_mesh.facets.nb()-1
         * @param[in] lv0 local vertex index in the facet (cv0 -> cv1), 0,1,...,order
         * @param[in] lv1 local vertex index in the facet (cv0 -> cv2), 0,1,...,order
         * @return control node index
         */
        [[nodiscard]] GEO::index_t facet_nd(const GEO::index_t f, const GEO::index_t lv0, const GEO::index_t lv1) const {
            assert(f < this->mesh_.facets.nb());
            return this->element_control_nodes_[f*this->CONTROL_POINTS_NB_PER_FACET_ + this->facet_lnd(lv0, lv1)];
        }

        /**
         * Get an internal control node index by two local vertex indexes in the facet
         * @param[in] f facet index, 0,1,...,hex_mesh.facets.nb()-1
         * @param[in] lv0 local vertex index in the facet (cv0 -> cv1), 0,1,...,order-2
         * @param[in] lv1 local vertex index in the facet (cv0 -> cv2), 0,1,...,order-2
         * @return control node index
         */
        [[nodiscard]] GEO::index_t facet_inner_nd(const GEO::index_t f, const GEO::index_t lv0, const GEO::index_t lv1) const {
            assert(f < this->mesh_.facets.nb());
            return this->element_control_nodes_[f*this->CONTROL_POINTS_NB_PER_FACET_ + this->facet_inner_lnd(lv0, lv1)];
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
    };

    extern template class SurfaceControlGrid<2>;
    extern template class SurfaceControlGrid<3>;
}

#endif //HOSM_SURF_CONTROL_GRID_H
