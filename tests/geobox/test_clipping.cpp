//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/11/13.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <cmath>
#include <gtest/gtest.h>
#include <geolio/geobox/clipping.h>

namespace geolio::test
{
    /**
     * @brief The clipping of the viewer starts inactive, with its plane across the
     *        Z axis of the scene, through the origin, keeping the side of the plane
     *        it keeps when it is turned on.
     * @details The plane starts facing the Z axis of the scene, attached to the view,
     *          and cutting the geometry rather than the cells: a clip is a plane
     *          first, and the cell modes are the ones to ask for.
     */
    TEST(ClipPlaneTest, StartsOnTheDefaultsOfGraphite) {
        const geobox::ClipPlane clip;
        EXPECT_FALSE(clip.active);
        EXPECT_EQ(clip.anchor, geobox::ClipAnchor::VIEW);
        EXPECT_DOUBLE_EQ(clip.normal[0], 0.0);
        EXPECT_DOUBLE_EQ(clip.normal[1], 0.0);
        EXPECT_DOUBLE_EQ(clip.normal[2], 1.0);
        EXPECT_FLOAT_EQ(clip.shift, 0.0f);
        EXPECT_FALSE(clip.invert);
        EXPECT_EQ(clip.mode, GLUP_CLIP_STANDARD);
    }

    /**
     * @brief The plane is perpendicular to the normal of the clip, whatever the
     *        direction and the length of that normal.
     * @details A vertex is kept when the dot product of its coordinates with the
     *          equation is positive, so the plane faces the way its normal does, and
     *          the opposite way once the clip is inverted.
     */
    TEST(ClipPlaneTest, PlaneIsPerpendicularToTheNormalOfTheClip) {
        geobox::ClipPlane clip;

        clip.normal = GEO::vec3(1.0, 0.0, 0.0);
        GEO::vec4 equation = geobox::clip_plane_equation(clip, GEO::vec3(0.0, 0.0, 0.0));
        EXPECT_DOUBLE_EQ(equation[0], 1.0);
        EXPECT_DOUBLE_EQ(equation[1], 0.0);
        EXPECT_DOUBLE_EQ(equation[2], 0.0);

        clip.normal = GEO::vec3(0.0, 0.0, 1.0);
        equation = geobox::clip_plane_equation(clip, GEO::vec3(0.0, 0.0, 0.0));
        EXPECT_DOUBLE_EQ(equation[2], 1.0);
        EXPECT_DOUBLE_EQ(equation[0], 0.0);
        EXPECT_DOUBLE_EQ(equation[1], 0.0);

        // A direction that is not an axis of the scene: the plane faces it, and
        // the coefficients are the ones of a unit vector, whatever the length the
        // normal was given with.
        clip.normal = GEO::vec3(1.0, 1.0, 0.0);
        equation = geobox::clip_plane_equation(clip, GEO::vec3(0.0, 0.0, 0.0));
        const double s = 1.0 / std::sqrt(2.0);
        EXPECT_DOUBLE_EQ(equation[0], s);
        EXPECT_DOUBLE_EQ(equation[1], s);
        EXPECT_DOUBLE_EQ(equation[2], 0.0);

        clip.normal = GEO::vec3(0.0, 0.0, 3.0);
        equation = geobox::clip_plane_equation(clip, GEO::vec3(0.0, 0.0, 0.0));
        EXPECT_DOUBLE_EQ(equation[2], 1.0);
        EXPECT_DOUBLE_EQ(geobox::clip_plane_normal(clip)[2], 1.0);

        // A normal dragged to nothing faces the viewer, as the default one does.
        clip.normal = GEO::vec3(0.0, 0.0, 0.0);
        equation = geobox::clip_plane_equation(clip, GEO::vec3(0.0, 0.0, 0.0));
        EXPECT_DOUBLE_EQ(equation[2], 1.0);
    }

    /**
     * @brief The shift is the coordinate the plane stands at along the axis, in the
     *        units of the scene, and it has no bound.
     * @details The side of the smaller coordinates is kept: the plane of a shift of
     *          2 along Z keeps what is below z = 2, and the one of a shift of -2
     *          keeps what is below z = -2.
     */
    TEST(ClipPlaneTest, ShiftIsThePositionOfThePlaneAlongTheNormal) {
        geobox::ClipPlane clip;
        clip.normal = GEO::vec3(0.0, 0.0, 1.0);

        // Shift 0: the plane goes through the origin, and the side the normal
        // points to is kept, i.e. z >= 0.
        EXPECT_DOUBLE_EQ(geobox::clip_plane_equation(clip, GEO::vec3(0.0, 0.0, 0.0))[3], 0.0);

        // A shift of 2 moves the plane to z = 2; a shift of -2 to z = -2; there is
        // no bound on it.
        clip.shift = 2.0f;
        EXPECT_DOUBLE_EQ(geobox::clip_plane_equation(clip, GEO::vec3(0.0, 0.0, 0.0))[3], -2.0);
        clip.shift = -2.0f;
        EXPECT_DOUBLE_EQ(geobox::clip_plane_equation(clip, GEO::vec3(0.0, 0.0, 0.0))[3], 2.0);
        clip.shift = 1000.0f;
        EXPECT_DOUBLE_EQ(geobox::clip_plane_equation(clip, GEO::vec3(0.0, 0.0, 0.0))[3], -1000.0);
    }

    /**
     * @brief The plane is offset by the bias along its normal, which is what keeps
     *        the vertices lying on it from falling on one side or the other
     *        depending on the frame.
     */
    TEST(ClipPlaneTest, BiasOffsetsThePlaneAlongItsNormal) {
        geobox::ClipPlane clip;
        clip.normal = GEO::vec3(0.0, 0.0, 1.0);
        clip.shift = 1.0f;

        const GEO::vec4 without = geobox::clip_plane_equation(clip, GEO::vec3(0.0, 0.0, 0.0));
        const GEO::vec4 with = geobox::clip_plane_equation(clip, GEO::vec3(0.0, 0.0, 0.0), 1e-3);
        // The offset is added to a coefficient that is close to 1 here, so the
        // difference carries the rounding of that addition.
        EXPECT_NEAR(with[3] - without[3], 1e-3, 1e-12);
        // The coefficients of the plane do not change, only its offset does.
        EXPECT_DOUBLE_EQ(with[0], without[0]);
        EXPECT_DOUBLE_EQ(with[1], without[1]);
        EXPECT_DOUBLE_EQ(with[2], without[2]);
    }

    /**
     * @brief The shift of a clip is a distance from the center of the scene along
     *        the normal, so that the plane of a null shift cuts the middle of the
     *        scene whatever the scene is centered on.
     */
    TEST(ClipPlaneTest, ShiftIsMeasuredFromTheCenterOfTheScene) {
        geobox::ClipPlane clip;
        clip.normal = GEO::vec3(0.0, 0.0, 1.0);

        // A scene centered on the origin: the plane of a shift of 1 stands at
        // z = 1.
        clip.shift = 1.0f;
        GEO::vec4 equation =
            geobox::clip_plane_equation(clip, GEO::vec3(0.0, 0.0, 0.0));
        EXPECT_NEAR(equation[3], -1.0, 1e-12);

        // A scene whose center is at z = 4: a shift of 0 puts the plane through
        // the center, i.e. at z = 4, and the shift then moves it from there.
        clip.shift = 0.0f;
        equation = geobox::clip_plane_equation(clip, GEO::vec3(1.0, 2.0, 4.0));
        EXPECT_NEAR(equation[3], -4.0, 1e-12);
        clip.shift = 2.0f;
        equation = geobox::clip_plane_equation(clip, GEO::vec3(1.0, 2.0, 4.0));
        EXPECT_NEAR(equation[3], -6.0, 1e-12);

        // Only the projection of the center on the normal counts.
        clip.normal = GEO::vec3(1.0, 0.0, 0.0);
        clip.shift = 0.0f;
        equation = geobox::clip_plane_equation(clip, GEO::vec3(3.0, 9.0, 9.0));
        EXPECT_NEAR(equation[3], -3.0, 1e-12);
    }

    /**
     * @brief Inverting the clip keeps the other side of the plane, i.e. the other
     *        half-space.
     */
    TEST(ClipPlaneTest, InvertKeepsTheOtherSideOfThePlane) {
        geobox::ClipPlane clip;
        clip.normal = GEO::vec3(1.0, 0.0, 0.0);
        clip.shift = 1.5f;

        const GEO::vec4 kept = geobox::clip_plane_equation(clip, GEO::vec3(0.0, 0.0, 0.0));
        clip.invert = true;
        const GEO::vec4 flipped = geobox::clip_plane_equation(clip, GEO::vec3(0.0, 0.0, 0.0));
        for (int i = 0; i < 4; ++i)
            EXPECT_DOUBLE_EQ(flipped[i], -kept[i]);

        // The kept side of the plane is the one its normal points to, i.e.
        // x >= 1.5, which its equation (1, 0, 0, -1.5) says.
        EXPECT_DOUBLE_EQ(kept[0], 1.0);
        EXPECT_DOUBLE_EQ(kept[3], -1.5);
    }
}
