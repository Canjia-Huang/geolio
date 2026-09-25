//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/14.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_OCTAHEDRAL_ROTATIONS_H
#define GEOLIO_OCTAHEDRAL_ROTATIONS_H
#include <geogram/basic/geometry.h>
#include <cassert>
#include <array>
#include <cmath>
#include <limits>

namespace geolio
{
    /**
     * The 4 orientation-preserving symmetry matrices of the square: the cyclic
     * group C4 of rotations by multiples of 90 degrees, which is the 2D analogue
     * of the octahedral rotation group (there is no octahedral group in 2D).
     * @details Each 2x2 matrix is a signed axis permutation with det=+1, used to
     * enumerate orientation candidates for frame/element alignment. The
     * reflections of the full square symmetry group D4 (det=-1) are deliberately
     * excluded, because a chart transition must preserve orientation.
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
     * Compute the 2D transition function between two parameterized triangles
     * that share an edge.
     *
     * @details The transition function is the rigid change of frame between the
     * two charts across their shared edge, i.e.
     * `uj = R * ui + t`, with a rotation `R` of `OCTAHEDRAL_ROTATIONS_2D` and an
     * integer translation `t`. `R` is recovered as the minimizer of the squared
     * distance between the source edge vector and its rotated image,
     * `|(uj2 - uj1) - R*(ui2 - ui1)|^2`, which is well defined and unique as soon
     * as the shared edge has non-zero length.
     *
     * @param[in] ui1 First endpoint of the shared edge, in the parameter domain
     *                 of the source chart.
     * @param[in] ui2 Second endpoint of the shared edge, in the parameter domain
     *                 of the source chart.
     * @param[in] uj1 Image of `ui1` in the parameter domain of the target chart.
     * @param[in] uj2 Image of `ui2` in the parameter domain of the target chart.
     * @param[out] ri Index in `OCTAHEDRAL_ROTATIONS_2D` of the selected rotation `R`.
     * @param[out] t Integer translation `t = uj1 - R*ui1`, rounded component-wise.
     *
     * @pre `ui1 != ui2` and `uj1 != uj2`: a zero-length shared edge leaves `R`
     *      under-determined and the result is arbitrary.
     * @pre `uj1` and `uj2` are the images of `ui1` and `ui2` respectively (same
     *      vertex correspondence on both sides). A permuted correspondence is not
     *      detected: the search then silently returns the closest, wrong pair.
     * @pre Both parameterizations live on the integer lattice, so that the exact
     *      `uj1 - R*ui1` is integral and rounding is lossless.
     *
     * @note The routine always reports the best candidate found and has no failure
     *       return value; a caller that cannot guarantee the preconditions above
     *       should validate the transition by checking `uj = R*ui + t` itself.
     * @note Ties are resolved in favor of the lowest rotation index.
     * @see compute_transition_function(const GEO::vec3&, const GEO::vec3&, const GEO::vec3&,
     *      const GEO::vec3&, const GEO::vec3&, const GEO::vec3&, GEO::index_t&, GEO::vec3&)
     *      for the tetrahedron-based 3D variant.
     */
    inline void compute_transition_function(
        const GEO::vec2& ui1, const GEO::vec2& ui2,
        const GEO::vec2& uj1, const GEO::vec2& uj2,
        GEO::index_t& ri,
        GEO::vec2& t
        ) {
        const auto ui2_vec = ui2-ui1;
        const auto uj2_vec = uj2-uj1;

        ri = GEO::NO_INDEX;
        double min_value = std::numeric_limits<double>::max();
        for (GEO::index_t i = 0; i < 4; ++i) {
            const auto& R = OCTAHEDRAL_ROTATIONS_2D[i];
            if (const double value = GEO::length2(uj2_vec - R*ui2_vec);
                value < min_value
                ) {
                ri = i;
                min_value = value;
            }
        }
        assert(ri != GEO::NO_INDEX);

        t = uj1 - OCTAHEDRAL_ROTATIONS_2D[ri]*ui1;
        t.x = std::round(t.x);
        t.y = std::round(t.y);
    }

    /**
     * The 24 proper rotation matrices of the octahedral (cube) symmetry group:
     * all signed axis permutations with det=+1, i.e. the full orientation-preserving
     * automorphism group of the integer lattice Z^3.
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
     * Compute the transition function between two parameterized tetrahedra that
     * share a face.
     *
     * @details The transition function is the rigid change of frame between the
     * two charts across their shared triangle, i.e.
     * `uj = R * ui + t`, with a rotation `R` of `OCTAHEDRAL_ROTATIONS_3D` and an
     * integer translation `t`. `R` is recovered as the minimizer of the summed
     * squared distance between the two source edge vectors and their rotated
     * images, `|(uj2 - uj1) - R*(ui2 - ui1)|^2 + |(uj3 - uj1) - R*(ui3 - ui1)|^2`.
     * Because two edges of a non-degenerate triangle span the plane they define
     * and separate the 24 rotations, the minimizer is unique on valid input.
     *
     * @param[in] ui1 First vertex of the shared triangle, in the parameter domain
     *                 of the source chart.
     * @param[in] ui2 Second vertex of the shared triangle, in the parameter domain
     *                 of the source chart.
     * @param[in] ui3 Third vertex of the shared triangle, in the parameter domain
     *                 of the source chart.
     * @param[in] uj1 Image of `ui1` in the parameter domain of the target chart.
     * @param[in] uj2 Image of `ui2` in the parameter domain of the target chart.
     * @param[in] uj3 Image of `ui3` in the parameter domain of the target chart.
     * @param[out] ri Index in `OCTAHEDRAL_ROTATIONS_3D` of the selected rotation `R`.
     * @param[out] t Integer translation `t = uj1 - R*ui1`, rounded component-wise.
     *
     * @pre The shared triangle is non-degenerate (`ui2-ui1` and `ui3-ui1` are
     *      linearly independent); a degenerate triangle leaves `R`
     *      under-determined and the result is arbitrary.
     * @pre `uj1`, `uj2`, `uj3` are the images of `ui1`, `ui2`, `ui3` respectively
     *      (same vertex correspondence on both sides, same orientation). A
     *      permuted correspondence is not detected: the search then silently
     *      returns the closest, wrong pair.
     * @pre Both parameterizations live on the integer lattice, so that the exact
     *      `uj1 - R*ui1` is integral and rounding is lossless.
     *
     * @note The routine always reports the best candidate found and has no failure
     *       return value; a caller that cannot guarantee the preconditions above
     *       should validate the transition by checking `uj = R*ui + t` itself.
     * @note Ties are resolved in favor of the lowest rotation index.
     * @see compute_transition_function(const GEO::vec2&, const GEO::vec2&,
     *      const GEO::vec2&, const GEO::vec2&, GEO::index_t&, GEO::vec2&)
     *      for the edge-based 2D variant.
     * @see HexHex: Highspeed Extraction of Hexahedral Meshes, Section 5.1.1.
     */
    inline void compute_transition_function(
        const GEO::vec3& ui1, const GEO::vec3& ui2, const GEO::vec3& ui3,
        const GEO::vec3& uj1, const GEO::vec3& uj2, const GEO::vec3& uj3,
        GEO::index_t& ri,
        GEO::vec3& t
        ) {
        const auto ui2_vec = ui2-ui1;
        const auto ui3_vec = ui3-ui1;
        const auto uj2_vec = uj2-uj1;
        const auto uj3_vec = uj3-uj1;

        ri = GEO::NO_INDEX;
        double min_value = std::numeric_limits<double>::max();
            for (GEO::index_t i = 0; i < 24; ++i) {
                const auto& R = OCTAHEDRAL_ROTATIONS_3D[i];
                if (const double value = GEO::length2(uj2_vec - R*ui2_vec) + GEO::length2(uj3_vec - R*ui3_vec);
                    value < min_value
                    ) {
                    ri = i;
                    min_value = value;
                }
            }
        assert(ri != GEO::NO_INDEX);

        t = uj1 - OCTAHEDRAL_ROTATIONS_3D[ri]*ui1;
        t.x = std::round(t.x);
        t.y = std::round(t.y);
        t.z = std::round(t.z);
    }
}
#endif //GEOLIO_OCTAHEDRAL_ROTATIONS_H
