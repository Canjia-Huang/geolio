//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/11/13.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "scene_light.h"

#include <algorithm>
#include <cmath>

namespace geolio::geobox
{
    namespace
    {
        constexpr double PI = 3.14159265358979323846;
        constexpr double DEGREES_PER_RADIAN = 180.0 / PI;
        constexpr double RADIANS_PER_DEGREE = PI / 180.0;

        /** Below this, the cross product of two unit vectors does not say
         *  around which axis they are rotated (they are parallel). */
        constexpr double PARALLEL_EPSILON = 1e-9;
    }

    GEO::vec3 scene_light_reference_direction(
        ) {
        return GEO::normalize(GEO::vec3(1.0, 1.0, 1.0));
    }

    GEO::vec3 scene_light_direction(
        const GEO::mat4& rotation
        ) {
        // The application rotates the reference direction the way
        // SimpleApplication::draw_scene_begin() does, as a row vector times the
        // matrix (an arc ball matrix multiplies the vector on its left, i.e.
        // this is the transposed rotation applied to the vector).
        const GEO::vec3 reference = scene_light_reference_direction();
        const GEO::vec4 rotated = GEO::vec4(
            reference.x, reference.y, reference.z, 0.0) * rotation;
        return GEO::normalize(GEO::vec3(rotated.x, rotated.y, rotated.z));
    }

    GEO::mat4 scene_light_rotation(
        const GEO::vec3& direction
        ) {
        const GEO::vec3 from = scene_light_reference_direction();
        const GEO::vec3 to = GEO::normalize(direction);

        // Rotation that takes `from` to `to` (axis and angle of Rodrigues): a
        // half turn around any axis orthogonal to `from` when the two are
        // opposite, no rotation at all when they are the same.
        const GEO::vec3 cross = GEO::cross(from, to);
        const double sine = GEO::length(cross);
        const double cosine = GEO::dot(from, to);

        GEO::vec3 axis(0.0, 0.0, 0.0);
        double unit_sine = 0.0;
        double unit_cosine = 1.0;
        if (sine < PARALLEL_EPSILON) {
            if (cosine < 0.0) {
                // Opposite directions: any axis orthogonal to `from` gives the
                // half turn that swaps them.
                const GEO::vec3 helper = std::abs(from.x) < 0.9 ?
                    GEO::vec3(1.0, 0.0, 0.0) : GEO::vec3(0.0, 1.0, 0.0);
                axis = GEO::normalize(GEO::cross(from, helper));
                unit_cosine = -1.0;
            }
            // Same direction: the identity, unit_axis is not used.
        }
        else {
            axis = cross / sine;
            unit_sine = sine;
            unit_cosine = cosine;
        }

        // Rodrigues' formula, for the convention where the matrix multiplies the
        // vector on its left: rotating v gives R v, with
        //  R = I cos + sin [a]x + (1 - cos) a a^T.
        const double x = axis.x, y = axis.y, z = axis.z;
        const double t = 1.0 - unit_cosine;
        const double r[3][3] = {
            {
                t * x * x + unit_cosine,
                t * x * y - unit_sine * z,
                t * x * z + unit_sine * y
            },
            {
                t * x * y + unit_sine * z,
                t * y * y + unit_cosine,
                t * y * z - unit_sine * x
            },
            {
                t * x * z - unit_sine * y,
                t * y * z + unit_sine * x,
                t * z * z + unit_cosine
            }
        };

        // The application applies the matrix to the reference direction as a row
        // vector (light = reference * M), which is M transposed applied to the
        // direction. Storing R transposed therefore makes the light point along
        // `to` (see scene_light_direction()).
        GEO::mat4 rotation;
        rotation.load_identity();
        for (GEO::index_t i = 0; i < 3; ++i) {
            for (GEO::index_t j = 0; j < 3; ++j)
                rotation(i, j) = r[j][i];
        }
        return rotation;
    }

    GEO::vec2 scene_light_angles(
        const GEO::vec3& direction
        ) {
        const GEO::vec3 unit = GEO::normalize(direction);
        return {
            std::atan2(unit.y, unit.x) * DEGREES_PER_RADIAN,
            std::asin(std::clamp(unit.z, -1.0, 1.0)) * DEGREES_PER_RADIAN
        };
    }

    GEO::vec3 scene_light_direction_from_angles(
        const GEO::vec2& angles
        ) {
        const double azimuth = angles.x * RADIANS_PER_DEGREE;
        const double elevation = angles.y * RADIANS_PER_DEGREE;
        const double horizontal = std::cos(elevation);
        return {
            horizontal * std::cos(azimuth),
            horizontal * std::sin(azimuth),
            std::sin(elevation)
        };
    }

    SceneLight& scene_light(
        ) {
        static SceneLight light;
        return light;
    }
}
