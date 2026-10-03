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
         * @param[in] M the input mesh, triangulated
         * \param[in] sharp_angle_threshold angles smaller than this threshold
         *                 (in degrees) are considered to be sharp features
         * @param[in] autoscale if positive, normalize the sizes so that the largest one is
         *                 exactly @p autoscale; if 0, store the raw magnitudes. A negative or
         *                 non-finite value is rejected, as is a mesh without facets, both with
         *                 an error or warning message and without touching the object.
         * @warning With @p preserve_size, flat facets get zero-length vectors; see the class
         *          documentation.
         */
        void create_curvature_directions(const GEO::Mesh& M, double sharp_angle_threshold = 45.0, double autoscale = 0.0);

    private:
        /// The estimated @f$|k_{\max}|@f$ per facet, in facet order.
        std::vector<double> certainty_max_;

        /// The estimated @f$|k_{\min}|@f$ per facet, in facet order.
        std::vector<double> certainty_min_;
    };
}

#endif //GEOLIO_FRAME_FIELD_H
