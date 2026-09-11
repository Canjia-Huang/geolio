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
    };
}

#endif //HOSM_SURF_CONTROL_GRID_H
