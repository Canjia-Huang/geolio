//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/5.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_QUAD_MOTORCYCLE_BLOCK_H
#define GEOLIO_QUAD_MOTORCYCLE_BLOCK_H
#include <array>
#include <geogram/mesh/mesh.h>

namespace geolio
{
    class QuadMotorCycleBlock {
    public:
        /**
         * Creates a block builder on a quadrilateral mesh.
         *
         * @param[in] mesh Input mesh.
         * @param[in] mesh_fc_tagged Face-corner tag map: `[4*f + lv]` is a tag value or
         *                           `GEO::NO_INDEX` for untagged corners.
         */
        QuadMotorCycleBlock(
           const GEO::Mesh& mesh,
           const GEO::Attribute<GEO::index_t>& mesh_fc_tagged);

        /**
         * Flood-fills connected faces from a start face and builds an ordered block layout.
         *
         * Tagged facets in `mesh_fc_tagged_` are treated as stopping boundaries for propagation.
         *
         * @param[in] start_f Start face index.
         *
         * @pre `start_f` is a valid face index in `mesh_`.
         * @post The connected block containing `start_f` is collected. When the collected faces
         *       form a dense cuboid they are ordered in `block_facets_`; otherwise the block
         *       wraps around (a loop) and the faces are left in flood-fill order, see
         *       `is_rectangular()`.
         */
        void flood_fill_facets(GEO::index_t start_f);

        /**
         * Checks whether the collected block was laid out as a dense cuboid.
         *
         * A block that wraps around on itself, e.g. a closed ring of quads without any singular
         * vertex, cannot be unrolled into a cuboid. Its faces are still collected, but the
         * ordered layout is not available and the block has to be cut open first.
         *
         * @return true when `block_facets()` forms a dense `len_x() x len_y()` layout.
         */
        [[nodiscard]] bool is_rectangular() const {
            return len_x_ != GEO::NO_INDEX &&
                   static_cast<std::size_t>(len_x_)*len_y_ == block_facets_.size();
        }

        /**
         * Gets the mesh vertices of the four block corners, in the local block order.
         *
         * @pre `is_rectangular()` is true.
         */
        [[nodiscard]] std::array<GEO::index_t, 4> corner_vertices() const;

        /**
         * Gets the ordered block faces.
         *
         * @return Read-only reference to the internal ordered faces.
         */
        [[nodiscard]] const auto& block_facets() const { return block_facets_; }

        /**
         * Gets the block width in faces along the local +X direction.
         */
        [[nodiscard]] auto len_x() const { return len_x_; }

        /**
         * Gets the block height in faces along the local +Y direction.
         */
        [[nodiscard]] auto len_y() const { return len_y_; }

        /**
         * Gets the mesh vertex index of a block corner in the current local face orientation.
         *
         * @param[in] lv Local corner id in `[0, 3]`.
         * @return Vertex index in `mesh_` corresponding to the queried corner.
         *
         * @pre `is_rectangular()` is true.
         */
        [[nodiscard]] GEO::index_t facet_corner_vertex(GEO::index_t lv) const;

    private:
        /**
         * Reorders collected faces into a dense `(x, y)` layout and updates block dimensions.
         *
         * @return true when the collected faces fill a dense cuboid. Otherwise (the block wraps
         *         around, i.e. it is a loop) false is returned and the layout is left untouched,
         *         so that the caller can cut the block open and try again.
         */
        bool rebuild_ordered_facets();

        const GEO::Mesh& mesh_;
        const GEO::Attribute<GEO::index_t>& mesh_fc_tagged_; // [4*f+lv] -> distance tag or GEO::NO_INDEX

        struct BlockFacet {
           GEO::index_t f{GEO::NO_INDEX};
           GEO::vec2i coord;
           GEO::index_t lv{GEO::NO_INDEX}; // same as the start_f's lv
        };

        /**
         *   +Y           3 --- 2
         *   |            |     |    (x, y) -> block_facets_[x + y*len_x]
         *   o --- +X     0 --- 1
         */
        std::vector<BlockFacet> block_facets_;
        GEO::index_t len_x_{GEO::NO_INDEX}, len_y_{GEO::NO_INDEX};
    };
}

#endif //GEOLIO_QUAD_MOTORCYCLE_BLOCK_H
