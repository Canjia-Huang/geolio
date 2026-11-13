//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/11/13.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "clipping.h"

#include <cmath>

namespace geolio::geobox
{
    GEO::vec3 clip_plane_normal(
        const ClipPlane& clip
        ) {
        const double length = std::sqrt(
            clip.normal[0] * clip.normal[0] +
            clip.normal[1] * clip.normal[1] +
            clip.normal[2] * clip.normal[2]
        );
        if (length <= 0.0) {
            // A normal the user can drag to nothing: the plane faces the viewer,
            // the direction it starts on.
            return GEO::vec3(0.0, 0.0, 1.0);
        }
        return GEO::vec3(
            clip.normal[0] / length,
            clip.normal[1] / length,
            clip.normal[2] / length
        );
    }

    GEO::vec4 clip_plane_equation(
        const ClipPlane& clip,
        const GEO::vec3& center,
        const double bias
        ) {
        // The plane stands at the shift away from the center of the scene, along
        // its normal, and the side the normal points to is kept: the shaders keep a
        // vertex when the dot product of its coordinates with the equation is
        // positive.
        const GEO::vec3 normal = clip_plane_normal(clip);
        const double offset =
            double(clip.shift) +
            center[0] * normal[0] +
            center[1] * normal[1] +
            center[2] * normal[2];
        GEO::vec4 equation(
            normal[0], normal[1], normal[2], -offset
        );

        // The plane keeps the side its normal points to; inverting the clipping
        // keeps the other side, which is the opposite half-space.
        if (clip.invert)
            equation = -1.0 * equation;

        // The offset comes last, so that it moves the plane the same way whether
        // the clip is inverted or not.
        equation[3] += bias;

        return equation;
    }
}
