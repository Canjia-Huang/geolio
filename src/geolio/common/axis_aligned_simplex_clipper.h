//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/17.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_AXIS_ALIGNED_SIMPLEX_CLIPPER_H
#define GEOLIO_AXIS_ALIGNED_SIMPLEX_CLIPPER_H
#include <geogram/basic/geometry.h>

namespace geolio
{
    /**
     * Computes the total tetrahedron volume and the portions on each side of an axis-aligned clipping plane.
     * @param[in] p0 First tetrahedron vertex in 3D space.
     * @param[in] p1 Second tetrahedron vertex in 3D space.
     * @param[in] p2 Third tetrahedron vertex in 3D space.
     * @param[in] p3 Fourth tetrahedron vertex in 3D space.
     * @param[in] dim Axis index of the clipping direction (0: x, 1: y, 2: z).
     * @param[in] t Plane offset/value along the selected axis.
     * @param[out] V Total signed tetrahedron volume.
     * @param[out] V0 Volume on the negative side of the clipping plane.
     * @param[out] V1 Volume on the positive side of the clipping plane.
     * @note The tetrahedron vertex order (p0, p1, p2, p3) should define a positive signed volume.
     */
    void axis_aligned_tet_clipped_volumes(
        const GEO::vec3& p0,
        const GEO::vec3& p1,
        const GEO::vec3& p2,
        const GEO::vec3& p3,
        GEO::index_t dim,
        double t,
        double& V,
        double& V0,
        double& V1);

    class AxisAlignedTetClipper {
    public:
        /**
         * Constructs a clipper from the four vertices of one tetrahedron.
         * @param[in] p0 First tetrahedron vertex in 3D space.
         * @param[in] p1 Second tetrahedron vertex in 3D space.
         * @param[in] p2 Third tetrahedron vertex in 3D space.
         * @param[in] p3 Fourth tetrahedron vertex in 3D space.
         * @note The vertex order (p0, p1, p2, p3) yield a positive signed tetrahedron volume.
         */
        AxisAlignedTetClipper(
            const GEO::vec3& p0,
            const GEO::vec3& p1,
            const GEO::vec3& p2,
            const GEO::vec3& p3);

        /**
         * Clips the current tetrahedral partition by an axis-aligned plane.
         * @param[in] dim Axis index of the clipping direction (0: x, 1: y, 2: z).
         * @param[in] t Plane offset/value along the selected axis.
         */
        void clip(
            GEO::index_t dim,
            double t);

        /**
         * Returns partition labels for each generated tetrahedron.
         * @note For the i-th cut plane, bit i is 0 if the cell is on the negative side, and 1 if it is on the positive side.
         * @note Size is the number of generated tetrahedra.
         */
        [[nodiscard]] const auto& partitions() const { return partitions_; }

        /**
         * Returns all generated tetrahedron vertex coordinates.
         * @note Size is 4 * number of generated tetrahedra.
         */
        [[nodiscard]] const auto& tet_coords() const { return tet_coords_; }

        /**
         * Returns barycentric coordinates corresponding to `tet_coords()`.
         * @note Size is 4 * number of generated tetrahedra.
         */
        [[nodiscard]] const auto& bary_coords() const { return bary_coords_; }

        /**
         * Returns cut plane indices for each generated tetrahedron facet.
         * @note Size is 4 * number of generated tetrahedra.
         * @note Element [4*c + i] corresponds to the cut plane index of cell c's facet opposite to its i-th vertex.
         */
        [[nodiscard]] const auto& tet_facet_cut_plane() const { return tet_facet_cut_plane_; }

    private:
        GEO::index_t cut_planes_nb_ = 0;
        std::vector<GEO::index_t> partitions_; /* size == tets_nb,
            partitions_[c] & (1<<i) == true -> in the positive side of the ith cut plane */

        std::vector<GEO::vec3> tet_coords_; // size == 4 * tets_nb
        std::vector<GEO::vec4> bary_coords_; // size == 4 * tets_nb
        std::vector<GEO::index_t> tet_facet_cut_plane_; /* size == 4 * tets_nb,
            [4*c+i] -> cut plane (of cell c's facet opposite to ith vertex), or GEO::NO_INDEX */
    };
}
#endif //GEOLIO_AXIS_ALIGNED_SIMPLEX_CLIPPER_H
