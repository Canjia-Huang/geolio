//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/2.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef HOSM_VOLUME_CONTROL_GRID_H
#define HOSM_VOLUME_CONTROL_GRID_H
#include "control_grid.h"

namespace geolio
{
    class VolumeControlGrid : public ControlGrid<3> {
    public:
        /**
         * @brief Construct a volume control grid.
         * @param[in] mesh Input volume mesh.
         * @param[in] order Polynomial order of the high-order volume representation.
         */
        VolumeControlGrid(const GEO::Mesh& mesh, const GEO::index_t order)
            : ControlGrid(mesh, order)
        {
            assert(mesh_.cells.nb() > 0);
        }

        /**
         * Get a local control node index by local vertex index.
         * @param[in] lv local vertex index in the cell, 0,1,...,mesh_.cells.nb_vertices
         * @return local control node index, 0,1,...,CONTROL_POINTS_NB_PER_CELL
         */
        [[nodiscard]] GEO::index_t cell_vertex_lnd(const GEO::index_t lv) const {
            assert(lv < mesh_.cells.nb_vertices(0));
            return ELEMENT_VERTEX_CONTROL_POINTS_BEGIN_IDX_[lv];
        }

        /**
         * Get a local control node index by local edge index and local vertex index in the edge.
         * @param[in] le local edge index, 0,1,...,mesh_.cells.nb_edges
         * @param[in] lv local vertex index in the edge (ev0 -> ev1), 0,1,...,order
         * @return local control node index, 0,1,...,CONTROL_POINTS_NB_PER_CELL
         */
        [[nodiscard]] GEO::index_t cell_edge_lnd(const GEO::index_t le, const GEO::index_t lv) const {
            assert(le < mesh_.cells.nb_edges(0));
            assert(lv < CONTROL_POINTS_NB_PER_EDGE_);
            return ELEMENT_EDGE_CONTROL_POINTS_BEGIN_IDX_[le] + lv*ELEMENT_EDGE_CONTROL_POINTS_NEXT_IDX_STEP_[le];
        }

        /**
         * Get a local internal control node index by local edge index and local vertex index in the edge.
         * @param[in] le local edge index, 0,1,...,mesh_.cells.nb_edges
         * @param[in] lv local vertex index in the edge (ev0 -> ev1), 0,1,...,order-2
         * @return local control node index, 0,1,...,CONTROL_POINTS_NB_PER_CELL
         */
        [[nodiscard]] GEO::index_t cell_edge_inner_lnd(const GEO::index_t le, const GEO::index_t lv) const {
            assert(le < mesh_.cells.nb_edges(0));
            assert(lv < INTERNAL_CONTROL_POINTS_NB_PER_EDGE_);
            return ELEMENT_EDGE_INTERNAL_CONTROL_POINTS_BEGIN_IDX_[le] + lv*ELEMENT_EDGE_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP_[le];
        }

        /**
         * Get a local control node index by local facet index and two local vertex index in the facet.
         * @param[in] lf local facet index, 0,1,...,mesh_.cells.nb_facets
         * @param[in] lv0 local vertex index in the facet (fv0 -> fv1), 0,1,...,order
         * @param[in] lv1 local vertex index in the facet (fv0 -> fv3), 0,1,...,order
         * @return local control node index, 0,1,...,CONTROL_POINTS_NB_PER_CELL
         */
        [[nodiscard]] GEO::index_t cell_facet_lnd(const GEO::index_t lf, const GEO::index_t lv0, const GEO::index_t lv1) const {
            assert(lf < mesh_.cells.nb_facets(0));
            assert(lv0 < CONTROL_POINTS_NB_PER_EDGE_);
            assert(lv1 < CONTROL_POINTS_NB_PER_EDGE_);
            return ELEMENT_FACET_CONTROL_POINTS_BEGIN_IDX_[lf] + lv0*ELEMENT_FACET_CONTROL_POINTS_NEXT_IDX_STEP0_[lf] + lv1*ELEMENT_FACET_CONTROL_POINTS_NEXT_IDX_STEP1_[lf];
        }

        /**
         * Get a local internal control node index by local facet index and two local vertex index in the facet.
         * @param[in] lf local facet index, 0,1,...,mesh_.cells.nb_facets
         * @param[in] lv0 local vertex index in the facet (fv0 -> fv1), 0,1,...,order-2
         * @param[in] lv1 local vertex index in the facet (fv0 -> fv3), 0,1,...,order-2
         * @return local control node index, 0,1,...,CONTROL_POINTS_NB_PER_CELL
         */
        [[nodiscard]] GEO::index_t cell_facet_inner_lnd(const GEO::index_t lf, const GEO::index_t lv0, const GEO::index_t lv1) const {
            assert(lf < mesh_.cells.nb_facets(0));
            assert(lv0 < INTERNAL_CONTROL_POINTS_NB_PER_EDGE_);
            assert(lv1 < INTERNAL_CONTROL_POINTS_NB_PER_EDGE_);
            return ELEMENT_FACET_INTERNAL_CONTROL_POINTS_BEGIN_IDX_[lf] + lv0*ELEMENT_FACET_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP0_[lf] + lv1*ELEMENT_FACET_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP1_[lf];
        }

        /**
         * Get a local control node index by three local vertex indexes in the cell
         * @param[in] lv0 local vertex index in the facet (cv0 -> cv1), 0,1,...,order
         * @param[in] lv1 local vertex index in the facet (cv0 -> cv2), 0,1,...,order
         * @param[in] lv2 local vertex index in the facet (cv0 -> cv4), 0,1,...,order
         * @return local control node index, 0,1,...,CONTROL_POINTS_NB_PER_CELL
         */
        [[nodiscard]] GEO::index_t cell_lnd(const GEO::index_t lv0, const GEO::index_t lv1, const GEO::index_t lv2) const {
            assert(lv0 < CONTROL_POINTS_NB_PER_EDGE_);
            assert(lv1 < CONTROL_POINTS_NB_PER_EDGE_);
            assert(lv2 < CONTROL_POINTS_NB_PER_EDGE_);
            return ELEMENT_CONTROL_POINTS_BEGIN_IDX_ + lv0*ELEMENT_CONTROL_POINTS_NEXT_IDX_STEP0_ + lv1*ELEMENT_CONTROL_POINTS_NEXT_IDX_STEP1_ + lv2*ELEMENT_CONTROL_POINTS_NEXT_IDX_STEP2_;
        }

        /**
         * Get a local internal control node index by three local vertex indexes in the cell
         * @param[in] lv0 local vertex index in the facet (cv0 -> cv1), 0,1,...,order-2
         * @param[in] lv1 local vertex index in the facet (cv0 -> cv2), 0,1,...,order-2
         * @param[in] lv2 local vertex index in the facet (cv0 -> cv4), 0,1,...,order-2
         * @return local control node index, 0,1,...,CONTROL_POINTS_NB_PER_CELL
         */
        [[nodiscard]] GEO::index_t cell_inner_lnd(const GEO::index_t lv0, const GEO::index_t lv1, const GEO::index_t lv2) const {
            assert(lv0 < INTERNAL_CONTROL_POINTS_NB_PER_EDGE_);
            assert(lv1 < INTERNAL_CONTROL_POINTS_NB_PER_EDGE_);
            assert(lv2 < INTERNAL_CONTROL_POINTS_NB_PER_EDGE_);
            return ELEMENT_INTERNAL_CONTROL_POINTS_BEGIN_IDX_ + lv0*ELEMENT_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP0_ + lv1*ELEMENT_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP1_ + lv2*ELEMENT_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP2_;
        }

        /**
         * Get a control node index by local vertex index in the cell.
         * @param[in] c cell index, 0,1,...,hex_mesh.cells.nb()-1
         * @param[in] lv local vertex index in the cell, 0,1,...,7
         * @return control node index
         */
        [[nodiscard]] GEO::index_t cell_vertex_nd(const GEO::index_t c, const GEO::index_t lv) const {
            assert(c < mesh_.cells.nb());
            return element_control_nodes_[c*CONTROL_POINTS_NB_PER_CELL_ + cell_vertex_lnd(lv)];
        }

        /**
         * Get a control node index by local edge index and local vertex index in the edge.
         * @param[in] c cell index, 0,1,...,hex_mesh.cells.nb()-1
         * @param[in] le local edge index, 0,1,...,11
         * @param[in] lv local vertex index in the edge (ev0 -> ev1), 0,1,...,order
         * @return control node index
         */
        [[nodiscard]] GEO::index_t cell_edge_nd(const GEO::index_t c, const GEO::index_t le, const GEO::index_t lv) const {
            assert(c < mesh_.cells.nb());
            return element_control_nodes_[c*CONTROL_POINTS_NB_PER_CELL_ + cell_edge_lnd(le, lv)];
        }

        /**
         * Get an internal control node index by local edge index and local vertex index in the edge.
         * @param[in] c cell index, 0,1,...,hex_mesh.cells.nb()-1
         * @param[in] le local edge index, 0,1,...,11
         * @param[in] lv local vertex index in the edge (ev0 -> ev1), 0,1,...,order-2
         * @return control node index
         */
        [[nodiscard]] GEO::index_t cell_edge_inner_nd(const GEO::index_t c, const GEO::index_t le, const GEO::index_t lv) const {
            assert(c < mesh_.cells.nb());
            return element_control_nodes_[c*CONTROL_POINTS_NB_PER_CELL_ + cell_edge_inner_lnd(le, lv)];
        }

        /**
         * Get a control node index by local facet index and two local vertex index in the facet.
         * @param[in] c cell index, 0,1,...,hex_mesh.cells.nb()-1
         * @param[in] lf local facet index, 0,1,...,5
         * @param[in] lv0 local vertex index in the facet (fv0 -> fv1), 0,1,...,order
         * @param[in] lv1 local vertex index in the facet (fv0 -> fv3), 0,1,...,order
         * @return control node index
         */
        [[nodiscard]] GEO::index_t cell_facet_nd(const GEO::index_t c, const GEO::index_t lf, const GEO::index_t lv0, const GEO::index_t lv1) const {
            assert(c < mesh_.cells.nb());
            return element_control_nodes_[c*CONTROL_POINTS_NB_PER_CELL_ + cell_facet_lnd(lf, lv0, lv1)];
        }

        /**
         * Get an internal control node index by local facet index and two local vertex indexes in the facet.
         * @param[in] c cell index, 0,1,...,hex_mesh.cells.nb()-1
         * @param[in] lf local facet index, 0,1,...,5
         * @param[in] lv0 local vertex index in the facet (fv0 -> fv1), 0,1,...,order-2
         * @param[in] lv1 local vertex index in the facet (fv0 -> fv3), 0,1,...,order-2
         * @return control node index
         */
        [[nodiscard]] GEO::index_t cell_facet_inner_nd(const GEO::index_t c, const GEO::index_t lf, const GEO::index_t lv0, const GEO::index_t lv1) const {
            assert(c < mesh_.cells.nb());
            return element_control_nodes_[c*CONTROL_POINTS_NB_PER_CELL_ + cell_facet_inner_lnd(lf, lv0, lv1)];
        }

        /**
         * Get a control node index by flattened local control-point index in one cell.
         * @param[in] c cell index, 0,1,...,hex_mesh.cells.nb()-1
         * @param[in] lv flattened local control-point index, 0,1,...,CONTROL_POINTS_NB_PER_CELL-1
         * @return control node index
         */
        [[nodiscard]] GEO::index_t cell_nd(const GEO::index_t c, const GEO::index_t lv) const {
            assert(c < mesh_.cells.nb());
            assert(lv < CONTROL_POINTS_NB_PER_CELL_);
            return element_control_nodes_[c*CONTROL_POINTS_NB_PER_CELL_ + lv];
        }

        /**
         * Get a control node index by three local vertex indexes in the cell
         * @param[in] c cell index, 0,1,...,hex_mesh.cells.nb()-1
         * @param[in] lv0 local vertex index in the facet (cv0 -> cv1), 0,1,...,order
         * @param[in] lv1 local vertex index in the facet (cv0 -> cv2), 0,1,...,order
         * @param[in] lv2 local vertex index in the facet (cv0 -> cv4), 0,1,...,order
         * @return control node index
         */
        [[nodiscard]] GEO::index_t cell_nd(const GEO::index_t c, const GEO::index_t lv0, const GEO::index_t lv1, const GEO::index_t lv2) const {
            assert(c < mesh_.cells.nb());
            return element_control_nodes_[c*CONTROL_POINTS_NB_PER_CELL_ + cell_lnd(lv0, lv1, lv2)];
        }

        /**
         * Get an internal control node index by three local vertex indexes in the cell
         * @param[in] c cell index, 0,1,...,hex_mesh.cells.nb()-1
         * @param[in] lv0 local vertex index in the facet (cv0 -> cv1), 0,1,...,order-2
         * @param[in] lv1 local vertex index in the facet (cv0 -> cv2), 0,1,...,order-2
         * @param[in] lv2 local vertex index in the facet (cv0 -> cv4), 0,1,...,order-2
         * @return control node index
         */
        [[nodiscard]] GEO::index_t cell_inner_nd(const GEO::index_t c, const GEO::index_t lv0, const GEO::index_t lv1, const GEO::index_t lv2) const {
            assert(c < mesh_.cells.nb());
            return element_control_nodes_[c*CONTROL_POINTS_NB_PER_CELL_ + cell_inner_lnd(lv0, lv1, lv2)];
        }
    };
}

#endif //HOSM_VOLUME_CONTROL_GRID_H
