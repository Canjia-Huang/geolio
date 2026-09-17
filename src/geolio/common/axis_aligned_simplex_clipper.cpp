//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/17.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "axis_aligned_simplex_clipper.h"
#include <cassert>
#include "log.h"

namespace geolio
{
    template <GEO::index_t DIM>
    AxisAlignedTriClipper<DIM>::AxisAlignedTriClipper(
        const GEO::vecng<DIM, double>& p0,
        const GEO::vecng<DIM, double>& p1,
        const GEO::vecng<DIM, double>& p2
        ) {
        this->partitions_.push_back(0);

        this->coords_.reserve(3);
        this->coords_.push_back(p0);
        this->coords_.push_back(p1);
        this->coords_.push_back(p2);

        this->bary_coords_.reserve(3);
        this->bary_coords_.emplace_back(1, 0, 0);
        this->bary_coords_.emplace_back(0, 1, 0);
        this->bary_coords_.emplace_back(0, 0, 1);

        this->facet_cut_plane_.reserve(3);
        this->facet_cut_plane_.push_back(GEO::NO_INDEX);
        this->facet_cut_plane_.push_back(GEO::NO_INDEX);
        this->facet_cut_plane_.push_back(GEO::NO_INDEX);
    }

    template <GEO::index_t DIM>
    void AxisAlignedTriClipper<DIM>::clip(
        const GEO::index_t dim,
        const double t
        ) {
        assert(dim < DIM);
        assert(this->coords_.size()%3 == 0);
        assert(this->bary_coords_.size()%3 == 0);

        const GEO::index_t PREV_TRIS_NB = this->partitions_.size();
        assert(this->coords_.size()/3 == PREV_TRIS_NB);
        assert(this->bary_coords_.size()/3 == PREV_TRIS_NB);

        /* Cut each triangle */
        std::array<double, 3> dists{};
        std::array<bool, 3> signs{};
        for (GEO::index_t f = 0; f < PREV_TRIS_NB; ++f) {
            const auto origin_partition = this->partitions_[f];

            dists[0] = this->coords_[3*f][dim]-t;
            dists[1] = this->coords_[3*f+1][dim]-t;
            dists[2] = this->coords_[3*f+2][dim]-t;
            signs[0] = dists[0]>0;
            signs[1] = dists[1]>0;
            signs[2] = dists[2]>0;

            if (const GEO::index_t positive_nb = signs[0]+signs[1]+signs[2];
                positive_nb == 0)
                continue;
            else if (positive_nb == 3)
                this->partitions_[f] |= (1<<this->cut_planes_nb_);
            else if (positive_nb == 1) {
                for (GEO::index_t lv = 0; lv < 3; ++lv) {
                    if (!signs[lv])
                        continue;

                    const GEO::index_t lv0 = lv;
                    const GEO::index_t lv1 = (lv+1)%3;
                    const GEO::index_t lv2 = (lv+2)%3;

                    const double r01 = std::abs(dists[lv0]) /  (std::abs(dists[lv0])+std::abs(dists[lv1]));
                    const double r02 = std::abs(dists[lv0]) /  (std::abs(dists[lv0])+std::abs(dists[lv2]));
                    assert(r01 >= 0 && r01 <=1);
                    assert(r02 >= 0 && r02 <=1);

                    const auto p0 = this->coords_[3*f+lv0];
                    const auto p1 = this->coords_[3*f+lv1];
                    const auto p2 = this->coords_[3*f+lv2];
                    const auto p01 = (1-r01)*p0 + r01*p1;
                    const auto p02 = (1-r02)*p0 + r02*p2;

                    const auto bp0 = this->bary_coords_[3*f+lv0];
                    const auto bp1 = this->bary_coords_[3*f+lv1];
                    const auto bp2 = this->bary_coords_[3*f+lv2];
                    const auto bp01 = (1-r01)*bp0 + r01*bp1;
                    const auto bp02 = (1-r02)*bp0 + r02*bp2;

                    this->partitions_.reserve(this->partitions_.size()+2);
                    this->coords_.reserve(this->coords_.size()+6);
                    this->bary_coords_.reserve(this->bary_coords_.size()+6);
                    this->facet_cut_plane_.reserve(this->facet_cut_plane_.size()+6);

                    this->partitions_.push_back(origin_partition);
                    this->coords_.push_back(p1);    this->bary_coords_.push_back(bp1);
                    this->coords_.push_back(p02);   this->bary_coords_.push_back(bp02);
                    this->coords_.push_back(p01);   this->bary_coords_.push_back(bp01);
                    this->facet_cut_plane_.push_back(GEO::NO_INDEX);
                    this->facet_cut_plane_.push_back(this->cut_planes_nb_);
                    this->facet_cut_plane_.push_back(this->facet_cut_plane_[3*f+lv0]);

                    this->partitions_.push_back(origin_partition);
                    this->coords_.push_back(p1);    this->bary_coords_.push_back(bp1);
                    this->coords_.push_back(p2);    this->bary_coords_.push_back(bp2);
                    this->coords_.push_back(p02);   this->bary_coords_.push_back(bp02);
                    this->facet_cut_plane_.push_back(this->facet_cut_plane_[3*f+lv1]);
                    this->facet_cut_plane_.push_back(this->facet_cut_plane_[3*f+lv2]);
                    this->facet_cut_plane_.push_back(GEO::NO_INDEX);

                    this->partitions_[f] |= (1<<this->cut_planes_nb_);
                    this->coords_[3*f+lv1] = p01;   this->bary_coords_[3*f+lv1] = bp01;
                    this->coords_[3*f+lv2] = p02;   this->bary_coords_[3*f+lv2] = bp02;
                    this->facet_cut_plane_[3*f+lv1] = this->cut_planes_nb_;

                    break;
                }
            }
            else {
                assert(positive_nb == 2);

                for (GEO::index_t lv = 0; lv < 3; ++lv) {
                    if (signs[lv])
                        continue;

                    const GEO::index_t lv0 = lv;
                    const GEO::index_t lv1 = (lv+1)%3;
                    const GEO::index_t lv2 = (lv+2)%3;

                    const double r01 = std::abs(dists[lv0]) /  (std::abs(dists[lv0])+std::abs(dists[lv1]));
                    const double r02 = std::abs(dists[lv0]) /  (std::abs(dists[lv0])+std::abs(dists[lv2]));
                    assert(r01 >= 0 && r01 <=1);
                    assert(r02 >= 0 && r02 <=1);

                    const auto p0 = this->coords_[3*f+lv0];
                    const auto p1 = this->coords_[3*f+lv1];
                    const auto p2 = this->coords_[3*f+lv2];
                    const auto p01 = (1-r01)*p0 + r01*p1;
                    const auto p02 = (1-r02)*p0 + r02*p2;

                    const auto bp0 = this->bary_coords_[3*f+lv0];
                    const auto bp1 = this->bary_coords_[3*f+lv1];
                    const auto bp2 = this->bary_coords_[3*f+lv2];
                    const auto bp01 = (1-r01)*bp0 + r01*bp1;
                    const auto bp02 = (1-r02)*bp0 + r02*bp2;

                    this->partitions_.reserve(this->partitions_.size()+2);
                    this->coords_.reserve(this->coords_.size()+6);
                    this->bary_coords_.reserve(this->bary_coords_.size()+6);
                    this->facet_cut_plane_.reserve(this->facet_cut_plane_.size()+6);

                    this->partitions_.push_back(origin_partition | (1<<this->cut_planes_nb_));
                    this->coords_.push_back(p1);    this->bary_coords_.push_back(bp1);
                    this->coords_.push_back(p02);   this->bary_coords_.push_back(bp02);
                    this->coords_.push_back(p01);   this->bary_coords_.push_back(bp01);
                    this->facet_cut_plane_.push_back(GEO::NO_INDEX);
                    this->facet_cut_plane_.push_back(this->cut_planes_nb_);
                    this->facet_cut_plane_.push_back(this->facet_cut_plane_[3*f+lv0]);

                    this->partitions_.push_back(origin_partition | (1<<this->cut_planes_nb_));
                    this->coords_.push_back(p1);    this->bary_coords_.push_back(bp1);
                    this->coords_.push_back(p2);    this->bary_coords_.push_back(bp2);
                    this->coords_.push_back(p02);   this->bary_coords_.push_back(bp02);
                    this->facet_cut_plane_.push_back(this->facet_cut_plane_[3*f+lv1]);
                    this->facet_cut_plane_.push_back(this->facet_cut_plane_[3*f+lv2]);
                    this->facet_cut_plane_.push_back(GEO::NO_INDEX);

                    // this->partitions_[f] = origin_partition;
                    this->coords_[3*f+lv1] = p01;   this->bary_coords_[3*f+lv1] = bp01;
                    this->coords_[3*f+lv2] = p02;   this->bary_coords_[3*f+lv2] = bp02;
                    this->facet_cut_plane_[3*f+lv1] = this->cut_planes_nb_;

                    break;
                }
            }
        }

        ++this->cut_planes_nb_;
        if (this->cut_planes_nb_ > 32)
            LOG::WARN("Cut planes nb == {} > 32, partitions out of uint32_t!", this->cut_planes_nb_);
    }

    template class AxisAlignedTriClipper<2>;
    template class AxisAlignedTriClipper<3>;

    AxisAlignedTetClipper::AxisAlignedTetClipper(
        const GEO::vec3& p0,
        const GEO::vec3& p1,
        const GEO::vec3& p2,
        const GEO::vec3& p3
        ) {
        partitions_.push_back(0);

        coords_.reserve(4);
        coords_.push_back(p0);
        coords_.push_back(p1);
        coords_.push_back(p2);
        coords_.push_back(p3);

        bary_coords_.reserve(4);
        bary_coords_.emplace_back(1, 0, 0, 0);
        bary_coords_.emplace_back(0, 1, 0, 0);
        bary_coords_.emplace_back(0, 0, 1, 0);
        bary_coords_.emplace_back(0, 0, 0, 1);

        facet_cut_plane_.reserve(4);
        facet_cut_plane_.push_back(GEO::NO_INDEX);
        facet_cut_plane_.push_back(GEO::NO_INDEX);
        facet_cut_plane_.push_back(GEO::NO_INDEX);
        facet_cut_plane_.push_back(GEO::NO_INDEX);
    }

    void AxisAlignedTetClipper::clip(
        const GEO::index_t dim,
        const double t
        ) {
        assert(dim < 3);
        assert(coords_.size()%4 == 0);
        assert(bary_coords_.size()%4 == 0);

        const GEO::index_t PREV_TETS_NB = partitions_.size();
        assert(coords_.size()/4 == PREV_TETS_NB);
        assert(bary_coords_.size()/4 == PREV_TETS_NB);

        /* Cut each tetrahedron */
        std::array<double, 4> dists{};
        std::array<bool, 4> signs{};
        for (GEO::index_t c = 0; c < PREV_TETS_NB; ++c) {
            const auto origin_partition = partitions_[c];

            dists[0] = coords_[4*c][dim]-t;
            dists[1] = coords_[4*c+1][dim]-t;
            dists[2] = coords_[4*c+2][dim]-t;
            dists[3] = coords_[4*c+3][dim]-t;
            signs[0] = dists[0]>0;
            signs[1] = dists[1]>0;
            signs[2] = dists[2]>0;
            signs[3] = dists[3]>0;

            if (const GEO::index_t positive_nb = signs[0]+signs[1]+signs[2]+signs[3];
                positive_nb == 0)
                continue;
            else if (positive_nb == 4)
                partitions_[c] |= (1<<cut_planes_nb_);
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

                    const auto p0 = coords_[4*c+lv0];
                    const auto p1 = coords_[4*c+lv1];
                    const auto p2 = coords_[4*c+lv2];
                    const auto p3 = coords_[4*c+lv3];
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
                    coords_.reserve(coords_.size()+12);
                    bary_coords_.reserve(bary_coords_.size()+12);
                    facet_cut_plane_.reserve(facet_cut_plane_.size()+12);

                    partitions_.push_back(origin_partition);
                    coords_.push_back(p1);    bary_coords_.push_back(bp1);
                    coords_.push_back(p3);    bary_coords_.push_back(bp3);
                    coords_.push_back(p2);    bary_coords_.push_back(bp2);
                    coords_.push_back(p01);   bary_coords_.push_back(bp01);
                    facet_cut_plane_.push_back(GEO::NO_INDEX);
                    facet_cut_plane_.push_back(facet_cut_plane_[4*c+lv3]);
                    facet_cut_plane_.push_back(facet_cut_plane_[4*c+lv2]);
                    facet_cut_plane_.push_back(facet_cut_plane_[4*c+lv0]);

                    partitions_.push_back(origin_partition);
                    coords_.push_back(p3);    bary_coords_.push_back(bp3);
                    coords_.push_back(p2);    bary_coords_.push_back(bp2);
                    coords_.push_back(p01);   bary_coords_.push_back(bp01);
                    coords_.push_back(p02);   bary_coords_.push_back(bp02);
                    facet_cut_plane_.push_back(facet_cut_plane_[4*c+lv3]);
                    facet_cut_plane_.push_back(GEO::NO_INDEX);
                    facet_cut_plane_.push_back(facet_cut_plane_[4*c+lv1]);
                    facet_cut_plane_.push_back(GEO::NO_INDEX);

                    partitions_.push_back(origin_partition);
                    coords_.push_back(p3);    bary_coords_.push_back(bp3);
                    coords_.push_back(p02);   bary_coords_.push_back(bp02);
                    coords_.push_back(p01);   bary_coords_.push_back(bp01);
                    coords_.push_back(p03);   bary_coords_.push_back(bp03);
                    facet_cut_plane_.push_back(cut_planes_nb_);
                    facet_cut_plane_.push_back(facet_cut_plane_[4*c+lv2]);
                    facet_cut_plane_.push_back(facet_cut_plane_[4*c+lv1]);
                    facet_cut_plane_.push_back(GEO::NO_INDEX);

                    partitions_[c] |= (1<<cut_planes_nb_);
                    coords_[4*c+lv1] = p01;   bary_coords_[4*c+lv1] = bp01;
                    coords_[4*c+lv2] = p02;   bary_coords_[4*c+lv2] = bp02;
                    coords_[4*c+lv3] = p03;   bary_coords_[4*c+lv3] = bp03;
                    facet_cut_plane_[4*c+lv0] = cut_planes_nb_;

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

                    const auto p0 = coords_[4*c+lv0];
                    const auto p1 = coords_[4*c+lv1];
                    const auto p2 = coords_[4*c+lv2];
                    const auto p3 = coords_[4*c+lv3];
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
                    coords_.reserve(coords_.size()+12);
                    bary_coords_.reserve(bary_coords_.size()+12);
                    facet_cut_plane_.reserve(facet_cut_plane_.size()+12);

                    partitions_.push_back(origin_partition | (1<<cut_planes_nb_));
                    coords_.push_back(p1);    bary_coords_.push_back(bp1);
                    coords_.push_back(p3);    bary_coords_.push_back(bp3);
                    coords_.push_back(p2);    bary_coords_.push_back(bp2);
                    coords_.push_back(p01);   bary_coords_.push_back(bp01);
                    facet_cut_plane_.push_back(GEO::NO_INDEX);
                    facet_cut_plane_.push_back(facet_cut_plane_[4*c+lv3]);
                    facet_cut_plane_.push_back(facet_cut_plane_[4*c+lv2]);
                    facet_cut_plane_.push_back(facet_cut_plane_[4*c+lv0]);

                    partitions_.push_back(origin_partition | (1<<cut_planes_nb_));
                    coords_.push_back(p3);    bary_coords_.push_back(bp3);
                    coords_.push_back(p2);    bary_coords_.push_back(bp2);
                    coords_.push_back(p01);   bary_coords_.push_back(bp01);
                    coords_.push_back(p02);   bary_coords_.push_back(bp02);
                    facet_cut_plane_.push_back(facet_cut_plane_[4*c+lv3]);
                    facet_cut_plane_.push_back(GEO::NO_INDEX);
                    facet_cut_plane_.push_back(facet_cut_plane_[4*c+lv1]);
                    facet_cut_plane_.push_back(GEO::NO_INDEX);

                    partitions_.push_back(origin_partition | (1<<cut_planes_nb_));
                    coords_.push_back(p3);    bary_coords_.push_back(bp3);
                    coords_.push_back(p02);   bary_coords_.push_back(bp02);
                    coords_.push_back(p01);   bary_coords_.push_back(bp01);
                    coords_.push_back(p03);   bary_coords_.push_back(bp03);
                    facet_cut_plane_.push_back(cut_planes_nb_);
                    facet_cut_plane_.push_back(facet_cut_plane_[4*c+lv2]);
                    facet_cut_plane_.push_back(facet_cut_plane_[4*c+lv1]);
                    facet_cut_plane_.push_back(GEO::NO_INDEX);

                    // partitions_[c] = origin_partition;
                    coords_[4*c+lv1] = p01;   bary_coords_[4*c+lv1] = bp01;
                    coords_[4*c+lv2] = p02;   bary_coords_[4*c+lv2] = bp02;
                    coords_[4*c+lv3] = p03;   bary_coords_[4*c+lv3] = bp03;
                    facet_cut_plane_[4*c+lv0] = cut_planes_nb_;

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

                const auto p0 = coords_[4*c+lv0];
                const auto p1 = coords_[4*c+lv1];
                const auto p2 = coords_[4*c+lv2];
                const auto p3 = coords_[4*c+lv3];
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
                coords_.reserve(coords_.size()+20);
                bary_coords_.reserve(bary_coords_.size()+20);
                facet_cut_plane_.reserve(facet_cut_plane_.size()+20);
                if (GEO::Geom::tetra_signed_volume(p02, p03, p12, p0) > 0) {
                    partitions_.push_back(origin_partition | (1<<cut_planes_nb_));
                    coords_.push_back(p02);   bary_coords_.push_back(bp02);
                    coords_.push_back(p03);   bary_coords_.push_back(bp03);
                    coords_.push_back(p12);   bary_coords_.push_back(bp12);
                    coords_.push_back(p0);    bary_coords_.push_back(bp0);
                    facet_cut_plane_.push_back(GEO::NO_INDEX);
                    facet_cut_plane_.push_back(facet_cut_plane_[4*c+lv3]);
                    facet_cut_plane_.push_back(facet_cut_plane_[4*c+lv1]);
                    facet_cut_plane_.push_back(cut_planes_nb_);

                    partitions_.push_back(origin_partition | (1<<cut_planes_nb_));
                    coords_.push_back(p13);   bary_coords_.push_back(bp13);
                    coords_.push_back(p12);   bary_coords_.push_back(bp12);
                    coords_.push_back(p03);   bary_coords_.push_back(bp03);
                    coords_.push_back(p1);    bary_coords_.push_back(bp1);
                    facet_cut_plane_.push_back(GEO::NO_INDEX);
                    facet_cut_plane_.push_back(facet_cut_plane_[4*c+lv2]);
                    facet_cut_plane_.push_back(facet_cut_plane_[4*c+lv0]);
                    facet_cut_plane_.push_back(cut_planes_nb_);

                    partitions_.push_back(origin_partition | (1<<cut_planes_nb_));
                    coords_.push_back(p0);    bary_coords_.push_back(bp0);
                    coords_.push_back(p03);   bary_coords_.push_back(bp03);
                    coords_.push_back(p12);   bary_coords_.push_back(bp12);
                    coords_.push_back(p1);    bary_coords_.push_back(bp1);
                    facet_cut_plane_.push_back(GEO::NO_INDEX);
                    facet_cut_plane_.push_back(facet_cut_plane_[4*c+lv3]);
                    facet_cut_plane_.push_back(facet_cut_plane_[4*c+lv2]);
                    facet_cut_plane_.push_back(GEO::NO_INDEX);

                    partitions_.push_back(origin_partition);
                    coords_.push_back(p03);   bary_coords_.push_back(bp03);
                    coords_.push_back(p12);   bary_coords_.push_back(bp12);
                    coords_.push_back(p13);   bary_coords_.push_back(bp13);
                    coords_.push_back(p3);    bary_coords_.push_back(bp3);
                    facet_cut_plane_.push_back(facet_cut_plane_[4*c+lv0]);
                    facet_cut_plane_.push_back(facet_cut_plane_[4*c+lv2]);
                    facet_cut_plane_.push_back(GEO::NO_INDEX);
                    facet_cut_plane_.push_back(cut_planes_nb_);

                    partitions_.push_back(origin_partition);
                    coords_.push_back(p12);   bary_coords_.push_back(bp12);
                    coords_.push_back(p03);   bary_coords_.push_back(bp03);
                    coords_.push_back(p02);   bary_coords_.push_back(bp02);
                    coords_.push_back(p2);    bary_coords_.push_back(bp2);
                    facet_cut_plane_.push_back(facet_cut_plane_[4*c+lv1]);
                    facet_cut_plane_.push_back(facet_cut_plane_[4*c+lv3]);
                    facet_cut_plane_.push_back(GEO::NO_INDEX);
                    facet_cut_plane_.push_back(cut_planes_nb_);

                    // partitions_[c] = origin_partition;
                    coords_[4*c+lv0] = p03;   bary_coords_[4*c+lv0] = bp03;
                    coords_[4*c+lv1] = p12;   bary_coords_[4*c+lv1] = bp12;
                    facet_cut_plane_[4*c+lv2] = GEO::NO_INDEX;
                    facet_cut_plane_[4*c+lv3] = GEO::NO_INDEX;
                }
                else {
                    partitions_.push_back(origin_partition | (1<<cut_planes_nb_));
                    coords_.push_back(p03);   bary_coords_.push_back(bp03);
                    coords_.push_back(p02);   bary_coords_.push_back(bp02);
                    coords_.push_back(p13);   bary_coords_.push_back(bp13);
                    coords_.push_back(p0);    bary_coords_.push_back(bp0);
                    facet_cut_plane_.push_back(GEO::NO_INDEX);
                    facet_cut_plane_.push_back(facet_cut_plane_[4*c+lv2]);
                    facet_cut_plane_.push_back(facet_cut_plane_[4*c+lv1]);
                    facet_cut_plane_.push_back(cut_planes_nb_);

                    partitions_.push_back(origin_partition | (1<<cut_planes_nb_));
                    coords_.push_back(p12);   bary_coords_.push_back(bp12);
                    coords_.push_back(p13);   bary_coords_.push_back(bp13);
                    coords_.push_back(p02);   bary_coords_.push_back(bp02);
                    coords_.push_back(p1);    bary_coords_.push_back(bp1);
                    facet_cut_plane_.push_back(GEO::NO_INDEX);
                    facet_cut_plane_.push_back(facet_cut_plane_[4*c+lv3]);
                    facet_cut_plane_.push_back(facet_cut_plane_[4*c+lv0]);
                    facet_cut_plane_.push_back(cut_planes_nb_);

                    partitions_.push_back(origin_partition | (1<<cut_planes_nb_));
                    coords_.push_back(p0);    bary_coords_.push_back(bp0);
                    coords_.push_back(p02);   bary_coords_.push_back(bp02);
                    coords_.push_back(p13);   bary_coords_.push_back(bp13);
                    coords_.push_back(p1);    bary_coords_.push_back(bp1);
                    facet_cut_plane_.push_back(GEO::NO_INDEX);
                    facet_cut_plane_.push_back(facet_cut_plane_[4*c+lv2]);
                    facet_cut_plane_.push_back(facet_cut_plane_[4*c+lv3]);
                    facet_cut_plane_.push_back(GEO::NO_INDEX);

                    partitions_.push_back(origin_partition);
                    coords_.push_back(p02);   bary_coords_.push_back(bp02);
                    coords_.push_back(p13);   bary_coords_.push_back(bp13);
                    coords_.push_back(p12);   bary_coords_.push_back(bp12);
                    coords_.push_back(p2);    bary_coords_.push_back(bp2);
                    facet_cut_plane_.push_back(facet_cut_plane_[4*c+lv0]);
                    facet_cut_plane_.push_back(facet_cut_plane_[4*c+lv3]);
                    facet_cut_plane_.push_back(GEO::NO_INDEX);
                    facet_cut_plane_.push_back(cut_planes_nb_);

                    partitions_.push_back(origin_partition);
                    coords_.push_back(p13);   bary_coords_.push_back(bp13);
                    coords_.push_back(p02);   bary_coords_.push_back(bp02);
                    coords_.push_back(p03);   bary_coords_.push_back(bp03);
                    coords_.push_back(p3);    bary_coords_.push_back(bp3);
                    facet_cut_plane_.push_back(facet_cut_plane_[4*c+lv1]);
                    facet_cut_plane_.push_back(facet_cut_plane_[4*c+lv2]);
                    facet_cut_plane_.push_back(GEO::NO_INDEX);
                    facet_cut_plane_.push_back(cut_planes_nb_);

                    // partitions_[c] = origin_partition;
                    coords_[4*c+lv0] = p02;   bary_coords_[4*c+lv0] = bp02;
                    coords_[4*c+lv1] = p13;   bary_coords_[4*c+lv1] = bp13;
                    facet_cut_plane_[4*c+lv2] = GEO::NO_INDEX;
                    facet_cut_plane_[4*c+lv3] = GEO::NO_INDEX;
                }
            }
        }

        ++cut_planes_nb_;
        if (cut_planes_nb_ > 32)
            LOG::WARN("Cut planes nb == {} > 32, partitions out of uint32_t!", cut_planes_nb_);
    }
}
