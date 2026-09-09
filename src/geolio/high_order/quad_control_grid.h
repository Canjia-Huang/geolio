//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/4.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef HOSM_QUAD_CONTROL_GRID_H
#define HOSM_QUAD_CONTROL_GRID_H
#include "surf_control_grid.h"

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
    inline double project_uv_quad_le_t(const GEO::vec2& uv, const GEO::index_t le) {
        assert(le < 4);
        switch (le) {
            case 0: return uv.x;
            case 1: return uv.y;
            case 2: return 1-uv.x;
            case 3: return 1-uv.y;
            default: return -1;
        }
    }

    template<GEO::index_t DIM>
    class QuadControlGrid : public SurfaceControlGrid<DIM> {
    public:
        QuadControlGrid(const GEO::Mesh& mesh, GEO::index_t order);

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
