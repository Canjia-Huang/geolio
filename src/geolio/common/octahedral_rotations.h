//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/14.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_OCTAHEDRAL_ROTATIONS_H
#define GEOLIO_OCTAHEDRAL_ROTATIONS_H
#include <geogram/basic/geometry.h>
#include <cassert>
#include <array>

namespace geolio
{
    /**
     * The 4 proper rotation matrices of the octahedral (cube) symmetry group.
     * @details Each 2x2 matrix is a right-handed axis permutation/sign-flip (det=1),
     * used to enumerate orientation candidates for frame/element alignment.
     */
    const std::array<GEO::mat2, 4> OCTAHEDRAL_ROTATIONS_2D = {
        {
            {{1, 0}, {0, 1}},
            {{0, -1}, {1, 0}},
            {{-1, 0}, {0, -1}},
            {{0, 1}, {-1, 0}}
        }
    };

    /**
     * The 24 proper rotation matrices of the octahedral (cube) symmetry group.
     * @details Each 3x3 matrix is a right-handed axis permutation/sign-flip (det=1),
     * used to enumerate orientation candidates for frame/element alignment.
     */
    const std::array<GEO::mat3, 24> OCTAHEDRAL_ROTATIONS_3D = {
        {
            {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
            {{1, 0, 0}, {0, -1, 0}, {0, 0, -1}},
            {{-1, 0, 0}, {0, 1, 0}, {0, 0, -1}},
            {{-1, 0, 0}, {0, -1, 0}, {0, 0, 1}},
            {{1, 0, 0}, {0, 0, 1}, {0, -1, 0}},
            {{1, 0, 0}, {0, 0, -1}, {0, 1, 0}},
            {{-1, 0, 0}, {0, 0, 1}, {0, 1, 0}},
            {{-1, 0, 0}, {0, 0, -1}, {0, -1, 0}},
            {{0, 1, 0}, {1, 0, 0}, {0, 0, -1}},
            {{0, 1, 0}, {-1, 0, 0}, {0, 0, 1}},
            {{0, -1, 0}, {1, 0, 0}, {0, 0, 1}},
            {{0, -1, 0}, {-1, 0, 0}, {0, 0, -1}},
            {{0, 1, 0}, {0, 0, 1}, {1, 0, 0}},
            {{0, 1, 0}, {0, 0, -1}, {-1, 0, 0}},
            {{0, -1, 0}, {0, 0, 1}, {-1, 0, 0}},
            {{0, -1, 0}, {0, 0, -1}, {1, 0, 0}},
            {{0, 0, 1}, {1, 0, 0}, {0, 1, 0}},
            {{0, 0, 1}, {-1, 0, 0}, {0, -1, 0}},
            {{0, 0, -1}, {1, 0, 0}, {0, -1, 0}},
            {{0, 0, -1}, {-1, 0, 0}, {0, 1, 0}},
            {{0, 0, 1}, {0, 1, 0}, {-1, 0, 0}},
            {{0, 0, 1}, {0, -1, 0}, {1, 0, 0}},
            {{0, 0, -1}, {0, 1, 0}, {1, 0, 0}},
            {{0, 0, -1}, {0, -1, 0}, {-1, 0, 0}}
        }
    };

    /**
     * Compute the transition function between two parameterized tetrahedra.
     *
     * This routine searches the octahedral rotations to find the best
     * alignment between the two local bases on the triangle adjacent to
     * two tetrahedra, then computes the integer translation that maps the
     * source origin to the target origin.
     *
     * @param[in] ui1  First parameter point of the shared triangle on the first tetrahedron.
     * @param[in] ui2  Second parameter point of the shared triangle on the first tetrahedron.
     * @param[in] ui3  Third parameter point of the shared triangle on the first tetrahedron.
     * @param[in] uj1  First parameter point of the shared triangle on the second tetrahedron.
     * @param[in] uj2  Second parameter point of the shared triangle on the second tetrahedron.
     * @param[in] uj3  Third parameter point of the shared triangle on the second tetrahedron.
     * @param[out] ri  Index of the best matching octahedral rotation in
     *                 `OCTAHEDRAL_ROTATIONS`.
     * @param[out] t   Integer translation vector from the source basis to the
     *                 target basis.
     *
     * @note `ri` must be valid after the search, and `t` is rounded to integer
     *       coordinates component-wise.
     * @see HexHex - Highspeed Extraction of Hexahedral Meshes, Section 5.1.1
     */
    template<GEO::index_t DIM>
    void compute_transition_function(
        const GEO::vecng<DIM, double>& ui1, const GEO::vecng<DIM, double>& ui2, const GEO::vecng<DIM, double>& ui3,
        const GEO::vecng<DIM, double>& uj1, const GEO::vecng<DIM, double>& uj2, const GEO::vecng<DIM, double>& uj3,
        GEO::index_t& ri,
        GEO::vecng<DIM, double>& t
        ) {
        static_assert(DIM == 2 || DIM == 3);
        const auto ui2_vec = ui2-ui1;
        const auto ui3_vec = ui3-ui1;
        const auto uj2_vec = uj2-uj1;
        const auto uj3_vec = uj3-uj1;

        ri = GEO::NO_INDEX;
        double min_value = std::numeric_limits<double>::max();
        if constexpr (DIM == 2) {
            for (GEO::index_t i = 0; i < 4; ++i) {
                const auto& R = OCTAHEDRAL_ROTATIONS_2D[i];
                if (const double value = GEO::length2(uj2_vec - R*ui2_vec) + GEO::length2(uj3_vec - R*ui3_vec);
                    value < min_value
                    ) {
                    ri = i;
                    min_value = value;
                }
            }
        }
        else {
            for (GEO::index_t i = 0; i < 24; ++i) {
                const auto& R = OCTAHEDRAL_ROTATIONS_3D[i];
                if (const double value = GEO::length2(uj2_vec - R*ui2_vec) + GEO::length2(uj3_vec - R*ui3_vec);
                    value < min_value
                    ) {
                    ri = i;
                    min_value = value;
                }
            }
        }
        assert(ri != GEO::NO_INDEX);

        if constexpr (DIM == 2) {
            t = uj1 - OCTAHEDRAL_ROTATIONS_2D[ri]*ui1;
            t.x = std::round(t.x);
            t.y = std::round(t.y);
        }
        else {
            t = uj1 - OCTAHEDRAL_ROTATIONS_3D[ri]*ui1;
            t.x = std::round(t.x);
            t.y = std::round(t.y);
            t.z = std::round(t.z);
        }
    }
}
#endif //GEOLIO_OCTAHEDRAL_ROTATIONS_H
