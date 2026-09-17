//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/17.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_AXIS_ALIGNED_SIMPLEX_CLIPPER_H
#define GEOLIO_AXIS_ALIGNED_SIMPLEX_CLIPPER_H
#include <geogram/basic/geometry.h>
#include <cassert>
#include <geogram/basic/geometry_nd.h>
#include <array>

namespace geolio
{
    /**
     * Computes the total triangle area and the portions on each side of an axis-aligned clipping plane.
     * @param[in] p0 First triangle vertex in space.
     * @param[in] p1 Second triangle vertex in space.
     * @param[in] p2 Third triangle vertex in space.
     * @param[in] dim Axis index of the clipping direction.
     * @param[in] t Plane offset/value along the selected axis.
     * @param[out] S Total triangle area.
     * @param[out] S_neg Area on the negative side of the clipping plane.
     * @param[out] S_pos Area on the positive side of the clipping plane.
     * @note The triangle vertex order (p0, p1, p2) should define a positive signed area.
     */
    template<GEO::index_t DIM>
    void axis_aligned_tri_clipped_areas(
        const GEO::vecng<DIM, double>& p0, const GEO::vecng<DIM, double>& p1, const GEO::vecng<DIM, double>& p2,
        const GEO::index_t dim, const double t,
        double& S, double& S_neg, double& S_pos
        ) {
        assert(dim < DIM);

        S = GEO::Geom::triangle_area(p0.data(), p1.data(), p2.data(), DIM);
        const std::array<double, 3> ds = { p0[dim]-t, p1[dim]-t, p2[dim]-t }; // dist to t
        const std::array<bool, 3> signs = { ds[0]>0, ds[1]>0, ds[2]>0 }; // positive

        if (const GEO::index_t positive_nb = signs[0]+signs[1]+signs[2];
            positive_nb == 0
            ) { // all negative - J0
            S_neg = S; S_pos = 0;
            }
        else if (positive_nb == 3) { // all positive - J1
            S_neg = 0, S_pos = S;
        }
        else if (positive_nb == 1) {
            for (GEO::index_t lv = 0; lv < 3; ++lv) { /* Find the positive one */
                if (signs[lv]) {
                    const GEO::index_t lv0 = lv;
                    const GEO::index_t lv1 = (lv+1)%3;
                    const GEO::index_t lv2 = (lv+2)%3;
                    const double r1 = std::abs(ds[lv0]) / (std::abs(ds[lv0]) + std::abs(ds[lv1]));
                    const double r2 = std::abs(ds[lv0]) / (std::abs(ds[lv0]) + std::abs(ds[lv2]));
                    assert(r1*r2 <= 1);
                    S_pos = S*r1*r2;
                    S_neg = S-S_pos;
                    break;
                }
            }
        }
        else {
            assert(positive_nb == 2);

            for (GEO::index_t lv = 0; lv < 3; ++lv) { /* Find the negative one */
                if (!signs[lv]) {
                    const GEO::index_t lv0 = lv;
                    const GEO::index_t lv1 = (lv+1)%3;
                    const GEO::index_t lv2 = (lv+2)%3;
                    const double r1 = std::abs(ds[lv0]) / (std::abs(ds[lv0]) + std::abs(ds[lv1]));
                    const double r2 = std::abs(ds[lv0]) / (std::abs(ds[lv0]) + std::abs(ds[lv2]));
                    assert(r1*r2 <= 1);
                    S_neg = S*r1*r2;
                    S_pos = S-S_neg;
                    break;
                }
            }
        }
    }

    /**
     * Computes the total tetrahedron volume and the portions on each side of an axis-aligned clipping plane.
     * @param[in] p0 First tetrahedron vertex in 3D space.
     * @param[in] p1 Second tetrahedron vertex in 3D space.
     * @param[in] p2 Third tetrahedron vertex in 3D space.
     * @param[in] p3 Fourth tetrahedron vertex in 3D space.
     * @param[in] dim Axis index of the clipping direction (0: x, 1: y, 2: z).
     * @param[in] t Plane offset/value along the selected axis.
     * @param[out] V Total signed tetrahedron volume.
     * @param[out] V_neg Volume on the negative side of the clipping plane.
     * @param[out] V_pos Volume on the positive side of the clipping plane.
     * @note The tetrahedron vertex order (p0, p1, p2, p3) should define a positive signed volume.
     */
    inline void axis_aligned_tet_clipped_volumes(
        const GEO::vec3& p0, const GEO::vec3& p1, const GEO::vec3& p2, const GEO::vec3& p3,
        const GEO::index_t dim, const double t,
        double& V, double& V_neg, double& V_pos
        ) {
        assert(dim < 3);

        V = GEO::Geom::tetra_volume(p0, p1, p2, p3);
        const std::array<double, 4> ds = { p0[dim]-t, p1[dim]-t, p2[dim]-t, p3[dim]-t }; // dist to t
        const std::array<bool, 4> signs = { ds[0]>0, ds[1]>0, ds[2]>0, ds[3]>0 }; // positive

        if (const GEO::index_t positive_nb = signs[0]+signs[1]+signs[2]+signs[3];
            positive_nb == 0
            ) { // all negative - J0
            V_neg = V; V_pos = 0;
        }
        else if (positive_nb == 4) { // all positive - J1
            V_neg = 0, V_pos = V;
        }
        else if (positive_nb == 1) {
            for (GEO::index_t lv = 0; lv < 4; ++lv) { /* Find the positive one */
                if (signs[lv]) {
                    const GEO::index_t lv0 = lv;
                    const GEO::index_t lv1 = (lv+1)%4;
                    const GEO::index_t lv2 = (lv+2)%4;
                    const GEO::index_t lv3 = (lv+3)%4;
                    const double r1 = std::abs(ds[lv0]) / (std::abs(ds[lv0]) + std::abs(ds[lv1]));
                    const double r2 = std::abs(ds[lv0]) / (std::abs(ds[lv0]) + std::abs(ds[lv2]));
                    const double r3 = std::abs(ds[lv0]) / (std::abs(ds[lv0]) + std::abs(ds[lv3]));
                    assert(r1*r2*r3 <= 1);
                    V_pos = V*r1*r2*r3;
                    V_neg = V-V_pos;
                    break;
                }
            }
        }
        else if (positive_nb == 3) {
            for (GEO::index_t lv = 0; lv < 4; ++lv) { /* Find the negative one */
                if (!signs[lv]) {
                    const GEO::index_t lv0 = lv;
                    const GEO::index_t lv1 = (lv+1)%4;
                    const GEO::index_t lv2 = (lv+2)%4;
                    const GEO::index_t lv3 = (lv+3)%4;
                    const double r1 = std::abs(ds[lv0]) / (std::abs(ds[lv0]) + std::abs(ds[lv1]));
                    const double r2 = std::abs(ds[lv0]) / (std::abs(ds[lv0]) + std::abs(ds[lv2]));
                    const double r3 = std::abs(ds[lv0]) / (std::abs(ds[lv0]) + std::abs(ds[lv3]));
                    assert(r1*r2*r3 <= 1);
                    V_neg = V*r1*r2*r3;
                    V_pos = V-V_neg;
                    break;
                }
            }
        }
        else {
            assert(positive_nb == 2);
            GEO::index_t lv0 = GEO::NO_INDEX; // positive
            GEO::index_t lv1 = GEO::NO_INDEX; // positive
            GEO::index_t lv2 = GEO::NO_INDEX; // negative
            GEO::index_t lv3 = GEO::NO_INDEX; // negative
            for (GEO::index_t lv = 0; lv < 4; ++lv) {
                if (signs[lv]) {
                    if (lv0 == GEO::NO_INDEX) lv0 = lv;
                    else lv1 = lv;
                }
                else {
                    if (lv2 == GEO::NO_INDEX) lv2 = lv;
                    else lv3 = lv;
                }
            }
            assert(lv0 < 4); assert(lv1 < 4); assert(lv2 < 4); assert(lv3 < 4);

            const double r02 = std::abs(ds[lv0]) / (std::abs(ds[lv0]) + std::abs(ds[lv2]));
            const double r03 = std::abs(ds[lv0]) / (std::abs(ds[lv0]) + std::abs(ds[lv3]));
            const double r12 = std::abs(ds[lv1]) / (std::abs(ds[lv1]) + std::abs(ds[lv2]));
            const double r13 = std::abs(ds[lv1]) / (std::abs(ds[lv1]) + std::abs(ds[lv3]));
            assert(r02*r03 + r12*r13*(1-r02) + r02*r13*(1-r03) <= 1);
            V_pos = V*(r02*r03 + r12*r13*(1-r02) + r02*r13*(1-r03));
            V_neg = V-V_pos;
        }
    }

    template<GEO::index_t DIM, GEO::index_t N>
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
        [[nodiscard]] const auto& coords() const { return coords_; }

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
