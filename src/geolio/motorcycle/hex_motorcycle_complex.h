//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/5.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_HEX_MOTORCYCLE_COMPLEX_H
#define GEOLIO_HEX_MOTORCYCLE_COMPLEX_H
#include <queue>
#include <geogram/mesh/mesh.h>
#include "hex_motorcycle_block.h"

namespace geolio
{
    /**
     * Motorcycle complex for hexahedral meshes (mesh based).
     *
     * This class computes a block decomposition of a hexahedral mesh using the motorcycle
     * complex / base complex construction described in the referenced paper. The algorithm
     * propagates "motorcycles" from singular and boundary edges to build a structured
     * partition of the mesh.
     *
     * @ref Brückler H, Gupta O, Mandad M, et al. The 3D motorcycle complex for structured volume decomposition[C]
     *      Computer Graphics Forum. 2022, 41(2): 221-235. (Section 5.1)
     */
    class HexMotorCycleComplex {
    public:
        /**
         * Creates a motorcycle complex processor for the given hexahedral mesh.
         *
         * @param[in] mesh Input hexahedral mesh. All cells must be hexahedra.
         *              The mesh is stored by reference and must outlive this object.
         *
         * @pre M is a valid hexahedral mesh with at least one cell.
         * @post Internal data structures are ready for edge classification and propagation.
         */
        explicit HexMotorCycleComplex(const GEO::Mesh& mesh);

        /**
         * Destroys the motorcycle complex and releases any bound temporary attributes.
         *
         * The destructor clears the internally bound cell-facet tag attribute created during
         * initialization, if it is still active.
         */
        ~HexMotorCycleComplex();

        /**
         * Motorcycle complex variants.
         */
        enum MotorCycleType {
            BASE_COMPLEX,        ///< Compute the base complex.
            MOTORCYCLE_COMPLEX   ///< Compute the motorcycle complex.
        };

        /**
         * Computes the selected complex and assigns block identifiers to the cells.
         *
         * @param[in] complex_type Complex variant to compute. Different variants may use
         *                         different propagation or stopping criteria.
         *
         * @return Total number of blocks.
         *
         * @post Every cell receives a block index. The block indices form a contiguous sequence
         *       from 0 to return value - 1.
         */
        GEO::index_t compute(MotorCycleType complex_type = BASE_COMPLEX);

        /**
         * Labels every cell in the input mesh with its block index.
         *
         * The block indices correspond to the ordering returned by `compute()`. Each block is
         * assigned a contiguous integer in the range `[0, nb_blocks - 1]`.
         *
         * @param[out] mesh_c_block Output attribute to receive the block id for each cell.
         *                          The attribute must already be bound and sized to the mesh.
         *
         * @pre `compute()` has been called and `blocks_` is populated.
         * @post Each cell entry in `mesh_c_block` contains the block index of the cell.
         */
        void label_blocks(GEO::Attribute<GEO::index_t>& mesh_c_block) const;

        /**
         * Builds a coarse hexahedral mesh from the computed block decomposition.
         *
         * Each coarse hex corresponds to one block in `blocks_`. Geometry and connectivity are
         * assembled from the block corner vertices inferred during decomposition.
         *
         * @param[out] mesh_out Output coarse hexahedral mesh. Existing content may be overwritten.
         * @param[out] old_cf_to_new_cf Optional map from original cell-facet ids (`8*c + lf`) to
         *                              coarse mesh facet ids. Pass `nullptr` to skip this mapping.
         *
         * @pre `compute()` has been called and `blocks_` is valid.
         * @post `M_out` contains the generated coarse mesh.
         */
        void create_coarse_mesh(
            GEO::Mesh& mesh_out,
            std::vector<GEO::index_t>* old_cf_to_new_cf = nullptr) const;

        /**
         * Returns the computed block decomposition.
         */
        [[nodiscard]] const auto& blocks() const { return blocks_; }

    private:
        /**
         * Identifies singular edges and boundary edges in the mesh.
         *
         * Singular edges are detected from edge valence, and boundary edges are detected from
         * their incident facets. These edges act as ignition sources for motorcycle propagation.
         *
         * This method populates:
         * - `M_ce_singular_`: singular/regular status for each edge
         * - `M_ce_border_`: boundary/interior status for each edge
         *
         * @post `M_ce_singular_` and `M_ce_border_` contain the edge classification results.
         *       An edge may be both singular and on the boundary.
         */
        void find_all_singular_and_border_edges();

        /**
         * Active fire front used during propagation.
         */
        struct Fire {
            GEO::index_t d;
            GEO::index_t c;
            GEO::index_t le;
            GEO::index_t lf;

            bool operator<(const Fire& other) const {
                if (d != other.d) return d > other.d;
                if (c != other.c) return c > other.c;
                if (lf != other.lf) return lf > other.lf;
                return le > other.le;
            }
        };

        /**
         * Inserts initial motorcycle fronts for singular and boundary edges.
         *
         * For each singular or boundary edge, this method creates a `Fire` at distance 0 and
         * pushes it into the priority queue. The fire stores the incident cell, local edge, and
         * local facet that define the propagation source.
         *
         * The priority queue is ordered by distance so that smaller-distance fronts are processed
         * first.
         *
         * @param[in,out] queue Priority queue (min-heap by distance) that will be populated
         *                      with initial motorcycle fronts. Existing contents are preserved.
         *
         * @pre `find_all_singular_and_border_edges()` has populated `M_ce_singular_` and
         *      `M_ce_border_`.
         * @post `queue` contains distance-0 fires for all identified singular and boundary edges.
         */
        void ignite(std::priority_queue<Fire>& queue) const;

        /**
         * Decomposes the mesh into topologically regular blocks.
         *
         * The method groups distance-tagged cells by propagation distance and connectivity.
         * Cells with the same distance that belong to the same connected component are assigned
         * to the same block.
         *
         * @return Total number of blocks created.
         *
         * @pre `M_cf_tagged_` has been populated by motorcycle propagation (for example via
         *      `compute()`).
         * @post Block indices are assigned to the processed cells.
         */
        GEO::index_t decompose_into_blocks();

        const std::string attribute_id_;

        const GEO::Mesh& mesh_; // Input hexahedral mesh.
        GEO::Attribute<GEO::index_t> mesh_cf_tagged_; // [8*c+lf] -> distance tag or GEO::NO_INDEX
        std::vector<bool> mesh_ce_singular_; // [12*c+le] -> singular/regular status
        std::vector<bool> mesh_ce_border_; // [12*c+le] -> boundary/interior status

        std::vector<HexMotorCycleBlock> blocks_;
    };
}

#endif //GEOLIO_HEX_MOTORCYCLE_COMPLEX_H
