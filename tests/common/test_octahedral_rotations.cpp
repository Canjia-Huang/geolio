//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/14.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <gtest/gtest.h>
#include <geolio/common/octahedral_rotations.h>
#include "../utils.h"

namespace
{
    template <GEO::index_t DIM>
    bool equal(
        const GEO::Matrix<DIM, double>& A,
        const GEO::Matrix<DIM, double>& B
        ) {
        for (GEO::index_t i = 0; i < DIM; ++i) {
            for (GEO::index_t j = 0; j < DIM; ++j) {
                if (std::fabs(A(i,j)-B(i,j)) > 1e-10)
                    return false;
            }
        }
        return true;
    }
}

namespace geolio::test
{
    template <GEO::index_t DIM>
    class OctahedralRotationsTest : public ::testing::Test {
        static_assert(DIM == 2 || DIM == 3);
    protected:
        void SetUp(
            ) override {
            if constexpr (DIM == 2) {
                for (const auto& R : OCTAHEDRAL_ROTATIONS_2D)
                    rotations_.push_back(R);
            }
            else {
                for (const auto& R : OCTAHEDRAL_ROTATIONS_3D)
                    rotations_.push_back(R);
            }
        }

        void test_determinant(
            ) const {
            for (const auto& R : rotations_)
                EXPECT_EQ(GEO::det(R), 1);
        }

        void test_orthogonal(
            ) const {
            for (const auto& R : rotations_) {
                const auto I = R * R.transpose();
                for (GEO::index_t i = 0; i < DIM; ++i) {
                    for (GEO::index_t j = 0; j < DIM; ++j) {
                        if (i == j)
                            EXPECT_EQ(I(i, j), 1);
                        else
                            EXPECT_EQ(I(i, j), 0);
                    }
                }
            }
        }

        void test_uniqueness(
            ) const {
            for (GEO::index_t i = 0, i_end = rotations_.size(); i < i_end; ++i) {
                const auto& Ri = rotations_[i];
                for (GEO::index_t j = i+1; j < i_end; ++j) {
                    const auto& Rj = rotations_[j];
                    EXPECT_FALSE(equal(Ri, Rj));
                }
            }
        }

        void test_closure(
            ) const {
            for (const auto& R0 : rotations_) {
                for (const auto& R1 : rotations_) {
                    const auto R = R0*R1;

                    bool found = false;
                    for (const auto& R2 : rotations_) {
                        if (equal(R, R2)) {
                            found = true;
                            break;
                        }
                    }
                    EXPECT_TRUE(found);
                }
            }
        }

        void test_contains_identity(
            ) const {
            GEO::Matrix<DIM, double> I;
            I.load_identity();
            for (const auto& R : rotations_) {
                if (equal(R, I))
                    return;
            }
            FAIL();
        }

        std::vector<GEO::Matrix<DIM, double>> rotations_;
    };

    template <typename DimType>
    class OctahedralRotationsDimTest : public OctahedralRotationsTest<DimType::value> {};

    TYPED_TEST_SUITE(OctahedralRotationsDimTest, DimTypes);

    TYPED_TEST(OctahedralRotationsDimTest, validity) {
        this->test_determinant();
        this->test_orthogonal();
        this->test_uniqueness();
        this->test_closure();
        this->test_contains_identity();
    }

    TYPED_TEST(OctahedralRotationsDimTest, compute_transition_function) {
        constexpr GEO::index_t DIM = TypeParam::value;
        constexpr GEO::index_t N = 100;

        for (GEO::index_t i = 0; i < N; ++i) {
            const auto gt_ri = static_cast<GEO::index_t>(std::round((this->rotations_.size()-1) * GEO::Numeric::random_float32()));
            const auto& R = this->rotations_[gt_ri];
            GEO::vecng<DIM, double> gt_t;
            for (GEO::index_t d = 0; d < DIM; ++d)
                gt_t[d] = GEO::Numeric::random_int32();

            GEO::vecng<DIM, double> p0;
            GEO::vecng<DIM, double> p1;
            GEO::vecng<DIM, double> p2;
            for (GEO::index_t d = 0; d < DIM; ++d) {
                p0[d] = GEO::Numeric::random_float32();
                p1[d] = GEO::Numeric::random_float32();
                p2[d] = GEO::Numeric::random_float32();
            }
            const auto q0 = R*p0 + gt_t;
            const auto q1 = R*p1 + gt_t;
            const auto q2 = R*p2 + gt_t;

            GEO::index_t ri;
            GEO::vecng<DIM, double> t;
            if constexpr (DIM == 2)
                compute_transition_function(p0, p1, q0, q1, ri, t);
            else
                compute_transition_function(p0, p1, p2, q0, q1, q2, ri, t);
            EXPECT_EQ(ri, gt_ri);
            EXPECT_NEAR(t.x, gt_t.x, 1e-10);
            EXPECT_NEAR(t.y, gt_t.y, 1e-10);
            if constexpr (DIM == 3)
                EXPECT_NEAR(t.z, gt_t.z, 1e-10);
        }
    }
}