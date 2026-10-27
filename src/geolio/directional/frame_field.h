//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/3.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_FRAME_FIELD_H
#define GEOLIO_FRAME_FIELD_H
#include <geogram/mesh/mesh_frame_field.h>

namespace geolio
{
    class FrameField : public GEO::FrameField {
    public:
        /**
         * @brief Creates a frame field aligned with the principal curvature directions,
         *        optionally keeping their magnitudes as the vector sizes.
         *
         * @param[in] mesh the input mesh, triangulated
         * \param[in] sharp_angle_threshold angles smaller than this threshold
         *                 (in degrees) are considered to be sharp features
         * @param[in] autoscale if positive, normalize the sizes so that the largest one is
         *                 exactly @p autoscale; if 0, store the raw magnitudes. A negative or
         *                 non-finite value is rejected, as is a mesh without facets, both with
         *                 an error or warning message and without touching the object.
         * @warning With @p preserve_size, flat facets get zero-length vectors; see the class
         *          documentation.
         */
        void create_curvature_directions(const GEO::Mesh& mesh, double sharp_angle_threshold = 45.0, double autoscale = 0.0);

        /**
         * @brief Computes one anisotropy matrix for each facet from the current frame field.
         *
         * The matrices are symmetric positive-definite 3x3 metrics. Their principal
         * directions follow the frame field, while their anisotropy is controlled by
         * the relative magnitudes of the two curvature directions. The matrices are
         * written in facet order and stored consecutively in row-major order.
         *
         * The metrics measure lengths, so a large weight means short cells: the larger
         * weight is the one along the maximum curvature direction, i.e. cells are made
         * short across a ridge and long along it. The weight along the facet normal is 1,
         * and the two in-plane weights multiply to 1, so that a facet is never made
         * globally smaller or larger than another: the anisotropy cannot turn into a
         * density bias. Their ratio is at most @p max_aspect, which it reaches where the
         * smallest curvature vanishes, i.e. on a developable region. An isotropic facet
         * (flat, or umbilic) therefore gets the identity.
         *
         * @param[in] mesh the mesh whose facets correspond to the current frame field
         * @param[in] max_aspect the maximum ratio of the two in-plane weights, i.e. the
         *                       square of the largest aspect ratio the cells can reach;
         *                       values below 1 are treated as 1
         * @param[out] matrices output vector, replaced with exactly
         *                       ``9 * M.facets.nb()`` coefficients. The matrix for
         *                       facet @p f starts at ``matrices[9 * f]`` and its
         *                       coefficient at row @p i and column @p j is stored at
         *                       ``matrices[9 * f + 3 * i + j]``
         *
         * @throws std::logic_error if the current frame field does not contain
         *         exactly one frame for each facet of @p M
         */
        void compute_anisotropy_matrices(const GEO::Mesh& mesh, double max_aspect, std::vector<double>& matrices) const;

    private:
        /// The estimated @f$|k_{\max}|@f$ per facet, in facet order.
        std::vector<double> certainty_max_;

        /// The estimated @f$|k_{\min}|@f$ per facet, in facet order.
        std::vector<double> certainty_min_;
    };
}

#endif //GEOLIO_FRAME_FIELD_H
