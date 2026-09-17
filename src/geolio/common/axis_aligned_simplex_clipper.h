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

    template <GEO::index_t DIM, GEO::index_t N>
    class AxisAlignedSimplexClipper {
    public:
        virtual ~AxisAlignedSimplexClipper() = default;

        /**
         * Clips the current tetrahedral partition by an axis-aligned plane.
         * @param[in] dim Axis index of the clipping direction (0: x, 1: y, 2: z).
         * @param[in] t Plane offset/value along the selected axis.
         */
        virtual void clip(GEO::index_t dim, double t) = 0;

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
        [[nodiscard]] const auto& tet_coords() const { return coords_; }

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
        [[nodiscard]] const auto& facet_cut_plane() const { return facet_cut_plane_; }

    protected:
        GEO::index_t cut_planes_nb_ = 0;
        std::vector<GEO::index_t> partitions_; /* size == simplex_nb,
            partitions_[c] & (1<<i) == true -> in the positive side of the ith cut plane */

        std::vector<GEO::vecng<DIM, double>> coords_; // size == N * simplex_nb
        std::vector<GEO::vecng<N, double>> bary_coords_; // size == N * simplex_nb
        std::vector<GEO::index_t> facet_cut_plane_; /* size == N * simplex_nb,
            [N*c+i] -> cut plane (of simplex c's border opposite to ith vertex), or GEO::NO_INDEX */
    };

    template <GEO::index_t DIM>
    class AxisAlignedTriClipper : public AxisAlignedSimplexClipper<DIM, 3> {
    public:
        /**
         * Constructs a clipper from the three vertices of one triangle.
         * @param[in] p0 First triangle vertex in 3D space.
         * @param[in] p1 Second triangle vertex in 3D space.
         * @param[in] p2 Third triangle vertex in 3D space.
         * @note The vertex order (p0, p1, p2) yield a positive signed triangle area.
         */
        AxisAlignedTriClipper(const GEO::vecng<DIM, double>& p0, const GEO::vecng<DIM, double>& p1, const GEO::vecng<DIM, double>& p2);

        /**
         * @see AxisAlignedSimplexClipper::clip(GEO::index_t, double)
         */
        void clip(GEO::index_t dim, double t) override;
    };

    extern template class AxisAlignedTriClipper<2>;
    extern template class AxisAlignedTriClipper<3>;

    class AxisAlignedTetClipper : public AxisAlignedSimplexClipper<3, 4> {
    public:
        /**
         * Constructs a clipper from the four vertices of one tetrahedron.
         * @param[in] p0 First tetrahedron vertex in 3D space.
         * @param[in] p1 Second tetrahedron vertex in 3D space.
         * @param[in] p2 Third tetrahedron vertex in 3D space.
         * @param[in] p3 Fourth tetrahedron vertex in 3D space.
         * @note The vertex order (p0, p1, p2, p3) yield a positive signed tetrahedron volume.
         */
        AxisAlignedTetClipper(const GEO::vec3& p0, const GEO::vec3& p1, const GEO::vec3& p2, const GEO::vec3& p3);

        /**
         * @see AxisAlignedSimplexClipper::clip(GEO::index_t, double)
         */
        void clip(GEO::index_t dim, double t) override;
    };
}
#endif //GEOLIO_AXIS_ALIGNED_SIMPLEX_CLIPPER_H
