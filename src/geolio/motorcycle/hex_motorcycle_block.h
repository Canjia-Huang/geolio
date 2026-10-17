//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/5.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_HEX_MOTORCYCLE_BLOCK_H
#define GEOLIO_HEX_MOTORCYCLE_BLOCK_H
#include <geogram/mesh/mesh.h>
#include <array>

namespace geolio
{
    class HexMotorCycleBlock {
    public:
        /**
         * Creates a block builder on a hexahedral mesh.
         *
         * @param[in] mesh Input mesh.
         * where facet `(c, lf)` is paired with facet `(nc, nlf)`.
         * @param[in] mesh_cf_tagged Cell-facet tag map: `[8*c + lf]` is a tag value or `GEO::NO_INDEX`
         * for untagged facets.
         */
        HexMotorCycleBlock(
            const GEO::Mesh& mesh,
            const GEO::Attribute<GEO::index_t>& mesh_cf_tagged);

        /**
         * Flood-fills connected cells from a start cell and builds an ordered block layout.
         *
         * Tagged facets in `M_cf_tagged_` are treated as stopping boundaries for propagation.
         *
         * @param[in] start_c Start cell index.
         *
         * @post The connected block containing `start_c` is collected. When the collected cells
         *       form a dense cuboid they are ordered in `block_cells_`; otherwise the block wraps
         *       around (a loop) and the cells are left in flood-fill order, see `is_cuboid()`.
         */
        void flood_fill_cells(GEO::index_t start_c);

        /**
         * Checks whether the collected block was laid out as a dense cuboid.
         *
         * A block that wraps around, e.g. a ring of hexahedra without any singular edge, cannot
         * be unrolled into a cuboid. Its cells are still collected, but the ordered layout is not
         * available and the block has to be cut open first.
         *
         * @return true when `block_cells()` forms a dense `len_x() x len_y() x len_z()` layout.
         */
        [[nodiscard]] bool is_cuboid() const {
            return len_x_ != GEO::NO_INDEX &&
                   static_cast<std::size_t>(len_x_)*len_y_*len_z_ == block_cells_.size();
        }

        /**
         * Gets the mesh vertices of the eight block corners, in the local block order.
         *
         * @pre `is_cuboid()` is true.
         */
        [[nodiscard]] std::array<GEO::index_t, 8> corner_vertices() const;

        /**
         * Gets the ordered block cells.
         *
         * @return Read-only reference to internal ordered cells.
         */
        [[nodiscard]] const auto& block_cells() const { return block_cells_; }

        /**
         * Gets the mesh vertex index of a block corner in the current local cell orientation.
         *
         * @param[in] lv Local corner id in `[0, 7]`.
         * @return Vertex index in `M_` corresponding to the queried corner.
         *
         * @pre `is_cuboid()` is true.
         */
        [[nodiscard]] GEO::index_t cell_corner_vertex(GEO::index_t lv) const;

        /**
         * Gets the block width in faces along the local +X direction.
         */
        [[nodiscard]] auto len_x() const { return len_x_; }

        /**
         * Gets the block height in faces along the local +Y direction.
         */
        [[nodiscard]] auto len_y() const { return len_y_; }

        /**
         * Gets the block height in faces along the local +Z direction.
         */
        [[nodiscard]] auto len_z() const { return len_z_; }

    private:
        /**
         * Reorders collected cells into a dense `(x, y, z)` layout and updates block dimensions.
         *
         * @return true when the collected cells fill a dense cuboid. Otherwise (the block wraps
         *         around, i.e. it is a loop) false is returned and the layout is left untouched,
         *         so that the caller can cut the block open and try again.
         */
        bool rebuild_ordered_cells();

        const GEO::Mesh& mesh_;
        const GEO::Attribute<GEO::index_t>& mesh_cf_tagged_; // [8*c+lf] -> tagged (distance) or not (GEO::NO_INDEX)

        struct BlockCell {
            GEO::index_t c{GEO::NO_INDEX};
            GEO::vec3i coord;
            std::array<GEO::index_t, 6> lfs{
                GEO::NO_INDEX, GEO::NO_INDEX, GEO::NO_INDEX, GEO::NO_INDEX, GEO::NO_INDEX, GEO::NO_INDEX
                }; // same as the start_c's lf
        };

        /*
             +Z                4-------6
             |                /|      /|
             o --- +Y        5-------7 |
            /                | 0-----|-2    (x, y, z) -> block_cells_[x + y*len_x + z*len_x*len_y]
          +X                 |/      |/
                             1-------3
         */
        std::vector<BlockCell> block_cells_;
        GEO::index_t len_x_{GEO::NO_INDEX}, len_y_{GEO::NO_INDEX}, len_z_{GEO::NO_INDEX};
    };
}

#endif //GEOLIO_HEX_MOTORCYCLE_BLOCK_H
