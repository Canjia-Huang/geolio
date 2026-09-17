//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/17.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "axis_aligned_simplex_clipper.h"
#include <cassert>
#include "log.h"

namespace geolio
{
    void axis_aligned_tet_clipped_volumes(
        const GEO::vec3& p0,
        const GEO::vec3& p1,
        const GEO::vec3& p2,
        const GEO::vec3& p3,
        const GEO::index_t dim,
        const double t,
        double& V,
        double& V0,
        double& V1
        ) {
        assert(dim < 3);

        V = GEO::Geom::tetra_volume(p0, p1, p2, p3);
        const std::array<double, 4> ds = { p0[dim]-t, p1[dim]-t, p2[dim]-t, p3[dim]-t }; // dist to t
        const std::array<bool, 4> signs = { ds[0]>0, ds[1]>0, ds[2]>0, ds[3]>0 }; // positive

        if (const GEO::index_t positive_nb = signs[0]+signs[1]+signs[2]+signs[3];
            positive_nb == 0
            ) { // all negative - J0
            V0 = V; V1 = 0;
        }
        else if (positive_nb == 4) { // all positive - J1
            V0 = 0, V1 = V;
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
                    V1 = V*r1*r2*r3;
                    V0 = V-V1;
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
                    V0 = V*r1*r2*r3;
                    V1 = V-V0;
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
            V1 = V*(r02*r03 + r12*r13*(1-r02) + r02*r13*(1-r03));
            V0 = V-V1;
        }
    }

    AxisAlignedTetClipper::AxisAlignedTetClipper(
        const GEO::vec3& p0,
        const GEO::vec3& p1,
        const GEO::vec3& p2,
        const GEO::vec3& p3
        ) {
        partitions_.push_back(0);

        tet_coords_.reserve(4);
        tet_coords_.push_back(p0);
        tet_coords_.push_back(p1);
        tet_coords_.push_back(p2);
        tet_coords_.push_back(p3);

        bary_coords_.reserve(4);
        bary_coords_.emplace_back(1, 0, 0, 0);
        bary_coords_.emplace_back(0, 1, 0, 0);
        bary_coords_.emplace_back(0, 0, 1, 0);
        bary_coords_.emplace_back(0, 0, 0, 1);

        tet_facet_cut_plane_.reserve(4);
        tet_facet_cut_plane_.push_back(GEO::NO_INDEX);
        tet_facet_cut_plane_.push_back(GEO::NO_INDEX);
        tet_facet_cut_plane_.push_back(GEO::NO_INDEX);
        tet_facet_cut_plane_.push_back(GEO::NO_INDEX);
    }

    void AxisAlignedTetClipper::clip(
        const GEO::index_t dim,
        const double t
        ) {
        assert(dim < 3);
        assert(tet_coords_.size()%4 == 0);
        assert(bary_coords_.size()%4 == 0);

        const GEO::index_t PREV_TETS_NB = partitions_.size();
        assert(tet_coords_.size()/4 == PREV_TETS_NB);
        assert(bary_coords_.size()/4 == PREV_TETS_NB);

        /* Cut each tet */
        for (GEO::index_t c = 0; c < PREV_TETS_NB; ++c) {
            assert(4*c+3 < tet_coords_.size());

            const auto origin_partition = partitions_[c];

            const std::array<double, 4> dists = {
                tet_coords_[4*c][dim]-t, tet_coords_[4*c+1][dim]-t, tet_coords_[4*c+2][dim]-t, tet_coords_[4*c+3][dim]-t
            };
            const std::array<bool, 4> signs = {
                dists[0]>0, dists[1]>0, dists[2]>0, dists[3]>0
            };

            if (const GEO::index_t positive_nb = signs[0]+signs[1]+signs[2]+signs[3];
                positive_nb == 0)
                continue;
            else if (positive_nb == 4) {
                partitions_[c] |= (1<<cut_planes_nb_);
            }
            else if (positive_nb == 1) {
                for (GEO::index_t lv = 0; lv < 4; ++lv) {
                    if (!signs[lv])
                        continue;

                    const GEO::index_t lv0 = lv;
                    const GEO::index_t lv1 = (lv+1)%4;
                    GEO::index_t lv2 = (lv+2)%4;
                    GEO::index_t lv3 = (lv+3)%4;
                    if (lv0%2 == 1)
                        std::swap(lv2, lv3);

                    const double r01 = std::abs(dists[lv0]) / (std::abs(dists[lv0])+std::abs(dists[lv1]));
                    const double r02 = std::abs(dists[lv0]) / (std::abs(dists[lv0])+std::abs(dists[lv2]));
                    const double r03 = std::abs(dists[lv0]) / (std::abs(dists[lv0])+std::abs(dists[lv3]));
                    assert(r01 >= 0 && r01 <= 1);
                    assert(r02 >= 0 && r02 <= 1);
                    assert(r03 >= 0 && r03 <= 1);

                    const auto p0 = tet_coords_[4*c+lv0];
                    const auto p1 = tet_coords_[4*c+lv1];
                    const auto p2 = tet_coords_[4*c+lv2];
                    const auto p3 = tet_coords_[4*c+lv3];
                    const auto p01 = (1-r01)*p0 + r01*p1;
                    const auto p02 = (1-r02)*p0 + r02*p2;
                    const auto p03 = (1-r03)*p0 + r03*p3;

                    const auto bp0 = bary_coords_[4*c+lv0];
                    const auto bp1 = bary_coords_[4*c+lv1];
                    const auto bp2 = bary_coords_[4*c+lv2];
                    const auto bp3 = bary_coords_[4*c+lv3];
                    const auto bp01 = (1-r01)*bp0 + r01*bp1;
                    const auto bp02 = (1-r02)*bp0 + r02*bp2;
                    const auto bp03 = (1-r03)*bp0 + r03*bp3;

                    partitions_.reserve(partitions_.size()+3);
                    tet_coords_.reserve(tet_coords_.size()+12);
                    bary_coords_.reserve(bary_coords_.size()+12);
                    tet_facet_cut_plane_.reserve(tet_facet_cut_plane_.size()+12);

                    partitions_.push_back(origin_partition);
                    tet_coords_.push_back(p1);    bary_coords_.push_back(bp1);
                    tet_coords_.push_back(p3);    bary_coords_.push_back(bp3);
                    tet_coords_.push_back(p2);    bary_coords_.push_back(bp2);
                    tet_coords_.push_back(p01);   bary_coords_.push_back(bp01);
                    tet_facet_cut_plane_.push_back(GEO::NO_INDEX);
                    tet_facet_cut_plane_.push_back(tet_facet_cut_plane_[4*c+lv3]);
                    tet_facet_cut_plane_.push_back(tet_facet_cut_plane_[4*c+lv2]);
                    tet_facet_cut_plane_.push_back(tet_facet_cut_plane_[4*c+lv0]);

                    partitions_.push_back(origin_partition);
                    tet_coords_.push_back(p3);    bary_coords_.push_back(bp3);
                    tet_coords_.push_back(p2);    bary_coords_.push_back(bp2);
                    tet_coords_.push_back(p01);   bary_coords_.push_back(bp01);
                    tet_coords_.push_back(p02);   bary_coords_.push_back(bp02);
                    tet_facet_cut_plane_.push_back(tet_facet_cut_plane_[4*c+lv3]);
                    tet_facet_cut_plane_.push_back(GEO::NO_INDEX);
                    tet_facet_cut_plane_.push_back(tet_facet_cut_plane_[4*c+lv1]);
                    tet_facet_cut_plane_.push_back(GEO::NO_INDEX);

                    partitions_.push_back(origin_partition);
                    tet_coords_.push_back(p3);    bary_coords_.push_back(bp3);
                    tet_coords_.push_back(p02);   bary_coords_.push_back(bp02);
                    tet_coords_.push_back(p01);   bary_coords_.push_back(bp01);
                    tet_coords_.push_back(p03);   bary_coords_.push_back(bp03);
                    tet_facet_cut_plane_.push_back(cut_planes_nb_);
                    tet_facet_cut_plane_.push_back(tet_facet_cut_plane_[4*c+lv2]);
                    tet_facet_cut_plane_.push_back(tet_facet_cut_plane_[4*c+lv1]);
                    tet_facet_cut_plane_.push_back(GEO::NO_INDEX);

                    partitions_[c] |= (1<<cut_planes_nb_);
                    tet_coords_[4*c+lv1] = p01;   bary_coords_[4*c+lv1] = bp01;
                    tet_coords_[4*c+lv2] = p02;   bary_coords_[4*c+lv2] = bp02;
                    tet_coords_[4*c+lv3] = p03;   bary_coords_[4*c+lv3] = bp03;
                    tet_facet_cut_plane_[4*c+lv0] = cut_planes_nb_;

                    break;
                }
            }
            else if (positive_nb == 3) {
                for (GEO::index_t lv = 0; lv < 4; ++lv) {
                    if (signs[lv])
                        continue;

                    const GEO::index_t lv0 = lv;
                    const GEO::index_t lv1 = (lv+1)%4;
                    GEO::index_t lv2 = (lv+2)%4;
                    GEO::index_t lv3 = (lv+3)%4;
                    if (lv0%2 == 1)
                        std::swap(lv2, lv3);

                    const double r01 = std::abs(dists[lv0]) / (std::abs(dists[lv0])+std::abs(dists[lv1]));
                    const double r02 = std::abs(dists[lv0]) / (std::abs(dists[lv0])+std::abs(dists[lv2]));
                    const double r03 = std::abs(dists[lv0]) / (std::abs(dists[lv0])+std::abs(dists[lv3]));
                    assert(r01 >= 0 && r01 <= 1);
                    assert(r02 >= 0 && r02 <= 1);
                    assert(r03 >= 0 && r03 <= 1);

                    const auto p0 = tet_coords_[4*c+lv0];
                    const auto p1 = tet_coords_[4*c+lv1];
                    const auto p2 = tet_coords_[4*c+lv2];
                    const auto p3 = tet_coords_[4*c+lv3];
                    const auto p01 = (1-r01)*p0 + r01*p1;
                    const auto p02 = (1-r02)*p0 + r02*p2;
                    const auto p03 = (1-r03)*p0 + r03*p3;

                    const auto bp0 = bary_coords_[4*c+lv0];
                    const auto bp1 = bary_coords_[4*c+lv1];
                    const auto bp2 = bary_coords_[4*c+lv2];
                    const auto bp3 = bary_coords_[4*c+lv3];
                    const auto bp01 = (1-r01)*bp0 + r01*bp1;
                    const auto bp02 = (1-r02)*bp0 + r02*bp2;
                    const auto bp03 = (1-r03)*bp0 + r03*bp3;

                    partitions_.reserve(partitions_.size()+3);
                    tet_coords_.reserve(tet_coords_.size()+12);
                    bary_coords_.reserve(bary_coords_.size()+12);
                    tet_facet_cut_plane_.reserve(tet_facet_cut_plane_.size()+12);

                    partitions_.push_back(origin_partition | (1<<cut_planes_nb_));
                    tet_coords_.push_back(p1);    bary_coords_.push_back(bp1);
                    tet_coords_.push_back(p3);    bary_coords_.push_back(bp3);
                    tet_coords_.push_back(p2);    bary_coords_.push_back(bp2);
                    tet_coords_.push_back(p01);   bary_coords_.push_back(bp01);
                    tet_facet_cut_plane_.push_back(GEO::NO_INDEX);
                    tet_facet_cut_plane_.push_back(tet_facet_cut_plane_[4*c+lv3]);
                    tet_facet_cut_plane_.push_back(tet_facet_cut_plane_[4*c+lv2]);
                    tet_facet_cut_plane_.push_back(tet_facet_cut_plane_[4*c+lv0]);

                    partitions_.push_back(origin_partition | (1<<cut_planes_nb_));
                    tet_coords_.push_back(p3);    bary_coords_.push_back(bp3);
                    tet_coords_.push_back(p2);    bary_coords_.push_back(bp2);
                    tet_coords_.push_back(p01);   bary_coords_.push_back(bp01);
                    tet_coords_.push_back(p02);   bary_coords_.push_back(bp02);
                    tet_facet_cut_plane_.push_back(tet_facet_cut_plane_[4*c+lv3]);
                    tet_facet_cut_plane_.push_back(GEO::NO_INDEX);
                    tet_facet_cut_plane_.push_back(tet_facet_cut_plane_[4*c+lv1]);
                    tet_facet_cut_plane_.push_back(GEO::NO_INDEX);

                    partitions_.push_back(origin_partition | (1<<cut_planes_nb_));
                    tet_coords_.push_back(p3);    bary_coords_.push_back(bp3);
                    tet_coords_.push_back(p02);   bary_coords_.push_back(bp02);
                    tet_coords_.push_back(p01);   bary_coords_.push_back(bp01);
                    tet_coords_.push_back(p03);   bary_coords_.push_back(bp03);
                    tet_facet_cut_plane_.push_back(cut_planes_nb_);
                    tet_facet_cut_plane_.push_back(tet_facet_cut_plane_[4*c+lv2]);
                    tet_facet_cut_plane_.push_back(tet_facet_cut_plane_[4*c+lv1]);
                    tet_facet_cut_plane_.push_back(GEO::NO_INDEX);

                    // partitions_[c] = origin_partition;
                    tet_coords_[4*c+lv1] = p01;   bary_coords_[4*c+lv1] = bp01;
                    tet_coords_[4*c+lv2] = p02;   bary_coords_[4*c+lv2] = bp02;
                    tet_coords_[4*c+lv3] = p03;   bary_coords_[4*c+lv3] = bp03;
                    tet_facet_cut_plane_[4*c+lv0] = cut_planes_nb_;

                    break;
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
                        if (lv0 == GEO::NO_INDEX)
                            lv0 = lv;
                        else
                            lv1 = lv;
                    }
                    else {
                        if (lv2 == GEO::NO_INDEX)
                            lv2 = lv;
                        else
                            lv3 = lv;
                    }
                }
                assert(lv0 != GEO::NO_INDEX); assert(lv1 != GEO::NO_INDEX); assert(lv2 != GEO::NO_INDEX); assert(lv3 != GEO::NO_INDEX);

                const double r02 = std::abs(dists[lv0]) / (std::abs(dists[lv0])+std::abs(dists[lv2]));
                const double r03 = std::abs(dists[lv0]) / (std::abs(dists[lv0])+std::abs(dists[lv3]));
                const double r12 = std::abs(dists[lv1]) / (std::abs(dists[lv1])+std::abs(dists[lv2]));
                const double r13 = std::abs(dists[lv1]) / (std::abs(dists[lv1])+std::abs(dists[lv3]));

                const auto p0 = tet_coords_[4*c+lv0];
                const auto p1 = tet_coords_[4*c+lv1];
                const auto p2 = tet_coords_[4*c+lv2];
                const auto p3 = tet_coords_[4*c+lv3];
                const auto p02 = (1-r02)*p0 + r02*p2;
                const auto p03 = (1-r03)*p0 + r03*p3;
                const auto p12 = (1-r12)*p1 + r12*p2;
                const auto p13 = (1-r13)*p1 + r13*p3;

                const auto bp0 = bary_coords_[4*c+lv0];
                const auto bp1 = bary_coords_[4*c+lv1];
                const auto bp2 = bary_coords_[4*c+lv2];
                const auto bp3 = bary_coords_[4*c+lv3];
                const auto bp02 = (1-r02)*bp0 + r02*bp2;
                const auto bp03 = (1-r03)*bp0 + r03*bp3;
                const auto bp12 = (1-r12)*bp1 + r12*bp2;
                const auto bp13 = (1-r13)*bp1 + r13*bp3;

                partitions_.reserve(partitions_.size()+5);
                tet_coords_.reserve(tet_coords_.size()+20);
                bary_coords_.reserve(bary_coords_.size()+20);
                tet_facet_cut_plane_.reserve(tet_facet_cut_plane_.size()+20);
                if (GEO::Geom::tetra_signed_volume(p02, p03, p12, p0) > 0) {
                    partitions_.push_back(origin_partition | (1<<cut_planes_nb_));
                    tet_coords_.push_back(p02);   bary_coords_.push_back(bp02);
                    tet_coords_.push_back(p03);   bary_coords_.push_back(bp03);
                    tet_coords_.push_back(p12);   bary_coords_.push_back(bp12);
                    tet_coords_.push_back(p0);    bary_coords_.push_back(bp0);
                    tet_facet_cut_plane_.push_back(GEO::NO_INDEX);
                    tet_facet_cut_plane_.push_back(tet_facet_cut_plane_[4*c+lv3]);
                    tet_facet_cut_plane_.push_back(tet_facet_cut_plane_[4*c+lv1]);
                    tet_facet_cut_plane_.push_back(cut_planes_nb_);

                    partitions_.push_back(origin_partition | (1<<cut_planes_nb_));
                    tet_coords_.push_back(p13);   bary_coords_.push_back(bp13);
                    tet_coords_.push_back(p12);   bary_coords_.push_back(bp12);
                    tet_coords_.push_back(p03);   bary_coords_.push_back(bp03);
                    tet_coords_.push_back(p1);    bary_coords_.push_back(bp1);
                    tet_facet_cut_plane_.push_back(GEO::NO_INDEX);
                    tet_facet_cut_plane_.push_back(tet_facet_cut_plane_[4*c+lv2]);
                    tet_facet_cut_plane_.push_back(tet_facet_cut_plane_[4*c+lv0]);
                    tet_facet_cut_plane_.push_back(cut_planes_nb_);

                    partitions_.push_back(origin_partition | (1<<cut_planes_nb_));
                    tet_coords_.push_back(p0);    bary_coords_.push_back(bp0);
                    tet_coords_.push_back(p03);   bary_coords_.push_back(bp03);
                    tet_coords_.push_back(p12);   bary_coords_.push_back(bp12);
                    tet_coords_.push_back(p1);    bary_coords_.push_back(bp1);
                    tet_facet_cut_plane_.push_back(GEO::NO_INDEX);
                    tet_facet_cut_plane_.push_back(tet_facet_cut_plane_[4*c+lv3]);
                    tet_facet_cut_plane_.push_back(tet_facet_cut_plane_[4*c+lv2]);
                    tet_facet_cut_plane_.push_back(GEO::NO_INDEX);

                    partitions_.push_back(origin_partition);
                    tet_coords_.push_back(p03);   bary_coords_.push_back(bp03);
                    tet_coords_.push_back(p12);   bary_coords_.push_back(bp12);
                    tet_coords_.push_back(p13);   bary_coords_.push_back(bp13);
                    tet_coords_.push_back(p3);    bary_coords_.push_back(bp3);
                    tet_facet_cut_plane_.push_back(tet_facet_cut_plane_[4*c+lv0]);
                    tet_facet_cut_plane_.push_back(tet_facet_cut_plane_[4*c+lv2]);
                    tet_facet_cut_plane_.push_back(GEO::NO_INDEX);
                    tet_facet_cut_plane_.push_back(cut_planes_nb_);

                    partitions_.push_back(origin_partition);
                    tet_coords_.push_back(p12);   bary_coords_.push_back(bp12);
                    tet_coords_.push_back(p03);   bary_coords_.push_back(bp03);
                    tet_coords_.push_back(p02);   bary_coords_.push_back(bp02);
                    tet_coords_.push_back(p2);    bary_coords_.push_back(bp2);
                    tet_facet_cut_plane_.push_back(tet_facet_cut_plane_[4*c+lv1]);
                    tet_facet_cut_plane_.push_back(tet_facet_cut_plane_[4*c+lv3]);
                    tet_facet_cut_plane_.push_back(GEO::NO_INDEX);
                    tet_facet_cut_plane_.push_back(cut_planes_nb_);

                    // partitions_[c] = origin_partition;
                    tet_coords_[4*c+lv0] = p03;   bary_coords_[4*c+lv0] = bp03;
                    tet_coords_[4*c+lv1] = p12;   bary_coords_[4*c+lv1] = bp12;
                    tet_facet_cut_plane_[4*c+lv2] = GEO::NO_INDEX;
                    tet_facet_cut_plane_[4*c+lv3] = GEO::NO_INDEX;
                }
                else {
                    partitions_.push_back(origin_partition | (1<<cut_planes_nb_));
                    tet_coords_.push_back(p03);   bary_coords_.push_back(bp03);
                    tet_coords_.push_back(p02);   bary_coords_.push_back(bp02);
                    tet_coords_.push_back(p13);   bary_coords_.push_back(bp13);
                    tet_coords_.push_back(p0);    bary_coords_.push_back(bp0);
                    tet_facet_cut_plane_.push_back(GEO::NO_INDEX);
                    tet_facet_cut_plane_.push_back(tet_facet_cut_plane_[4*c+lv2]);
                    tet_facet_cut_plane_.push_back(tet_facet_cut_plane_[4*c+lv1]);
                    tet_facet_cut_plane_.push_back(cut_planes_nb_);

                    partitions_.push_back(origin_partition | (1<<cut_planes_nb_));
                    tet_coords_.push_back(p12);   bary_coords_.push_back(bp12);
                    tet_coords_.push_back(p13);   bary_coords_.push_back(bp13);
                    tet_coords_.push_back(p02);   bary_coords_.push_back(bp02);
                    tet_coords_.push_back(p1);    bary_coords_.push_back(bp1);
                    tet_facet_cut_plane_.push_back(GEO::NO_INDEX);
                    tet_facet_cut_plane_.push_back(tet_facet_cut_plane_[4*c+lv3]);
                    tet_facet_cut_plane_.push_back(tet_facet_cut_plane_[4*c+lv0]);
                    tet_facet_cut_plane_.push_back(cut_planes_nb_);

                    partitions_.push_back(origin_partition | (1<<cut_planes_nb_));
                    tet_coords_.push_back(p0);    bary_coords_.push_back(bp0);
                    tet_coords_.push_back(p02);   bary_coords_.push_back(bp02);
                    tet_coords_.push_back(p13);   bary_coords_.push_back(bp13);
                    tet_coords_.push_back(p1);    bary_coords_.push_back(bp1);
                    tet_facet_cut_plane_.push_back(GEO::NO_INDEX);
                    tet_facet_cut_plane_.push_back(tet_facet_cut_plane_[4*c+lv2]);
                    tet_facet_cut_plane_.push_back(tet_facet_cut_plane_[4*c+lv3]);
                    tet_facet_cut_plane_.push_back(GEO::NO_INDEX);

                    partitions_.push_back(origin_partition);
                    tet_coords_.push_back(p02);   bary_coords_.push_back(bp02);
                    tet_coords_.push_back(p13);   bary_coords_.push_back(bp13);
                    tet_coords_.push_back(p12);   bary_coords_.push_back(bp12);
                    tet_coords_.push_back(p2);    bary_coords_.push_back(bp2);
                    tet_facet_cut_plane_.push_back(tet_facet_cut_plane_[4*c+lv0]);
                    tet_facet_cut_plane_.push_back(tet_facet_cut_plane_[4*c+lv3]);
                    tet_facet_cut_plane_.push_back(GEO::NO_INDEX);
                    tet_facet_cut_plane_.push_back(cut_planes_nb_);

                    partitions_.push_back(origin_partition);
                    tet_coords_.push_back(p13);   bary_coords_.push_back(bp13);
                    tet_coords_.push_back(p02);   bary_coords_.push_back(bp02);
                    tet_coords_.push_back(p03);   bary_coords_.push_back(bp03);
                    tet_coords_.push_back(p3);    bary_coords_.push_back(bp3);
                    tet_facet_cut_plane_.push_back(tet_facet_cut_plane_[4*c+lv1]);
                    tet_facet_cut_plane_.push_back(tet_facet_cut_plane_[4*c+lv2]);
                    tet_facet_cut_plane_.push_back(GEO::NO_INDEX);
                    tet_facet_cut_plane_.push_back(cut_planes_nb_);

                    // partitions_[c] = origin_partition;
                    tet_coords_[4*c+lv0] = p02;   bary_coords_[4*c+lv0] = bp02;
                    tet_coords_[4*c+lv1] = p13;   bary_coords_[4*c+lv1] = bp13;
                    tet_facet_cut_plane_[4*c+lv2] = GEO::NO_INDEX;
                    tet_facet_cut_plane_[4*c+lv3] = GEO::NO_INDEX;
                }
            }
        }

        ++cut_planes_nb_;
        if (cut_planes_nb_ > 32)
            LOG::WARN("Cut planes nb == {} > 32, partitions may out of uint32_t!", cut_planes_nb_);
    }
}
