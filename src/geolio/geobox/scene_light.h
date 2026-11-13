//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/11/13.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_SCENE_LIGHT_H
#define GEOLIO_SCENE_LIGHT_H

#include <geogram/basic/geometry.h>

namespace geolio::geobox
{
    /**
     * @brief Direction the light of the scene has before it is rotated.
     * @details geogram's GLUP draws with a single directional light, and the
     *          application does not store its direction: it rotates this fixed
     *          direction with the arc ball of the light and hands the result to
     *          GLUP (see SimpleApplication::draw_scene_begin()), i.e. the
     *          direction of the light is R * scene_light_reference_direction().
     *          The functions below invert and re-express exactly that.
     * @return The reference direction, a unit vector along (1,1,1).
     */
    [[nodiscard]] GEO::vec3 scene_light_reference_direction();

    /**
     * @brief Direction the light of the scene comes from, from its rotation.
     * @param[in] rotation Rotation of the light, as stored by the application
     *            (ArcBall::get_value()).
     * @return The unit direction the light comes from.
     */
    [[nodiscard]] GEO::vec3 scene_light_direction(const GEO::mat4& rotation);

    /**
     * @brief Rotation that makes the light of the scene come from a direction.
     * @details The rotation is the smallest one that takes
     *          scene_light_reference_direction() to @p direction: the light has
     *          no roll, only a direction, so any rotation that points it the
     *          right way describes the same light.
     * @param[in] direction Direction the light should come from; it does not
     *            need to be normalized.
     * @return The rotation to give to the light (ArcBall::set_value()).
     */
    [[nodiscard]] GEO::mat4 scene_light_rotation(const GEO::vec3& direction);

    /**
     * @brief Azimuth and elevation of a direction, in degrees.
     * @details The two angles that place a unit vector on the sphere: the
     *          azimuth turns around the Z axis, from the X axis, and the
     *          elevation is the angle above the XY plane. They are what the
     *          light controls of the viewer panel edit.
     * @param[in] direction Direction to express as angles (normalized here).
     * @return The azimuth in [-180, 180] and the elevation in [-90, 90].
     */
    [[nodiscard]] GEO::vec2 scene_light_angles(const GEO::vec3& direction);

    /**
     * @brief Direction that has the given azimuth and elevation.
     * @param[in] angles Azimuth and elevation, in degrees (see
     *            scene_light_angles()).
     * @return The unit direction the light comes from.
     */
    [[nodiscard]] GEO::vec3 scene_light_direction_from_angles(
        const GEO::vec2& angles
        );

    /**
     * @brief The parameters of the light of the scene, other than its direction.
     * @details GLUP shades with a single directional light, and it bakes the
     *          color, the intensity, the ambient level and the exponent of the
     *          specular highlight of that light into its shaders. GeoBox draws the
     *          surfaces with its own program (see LightShader) to make them
     *          parameters of the viewer; they describe the light of the scene as a
     *          whole, so they live here rather than in the objects, which is also
     *          where GLUP keeps its light.
     *          The defaults are the constants of GLUP's shader, so that a scene
     *          drawn with them looks the way it looked before the light could be
     *          set.
     */
    struct SceneLight {
        /** Color of the light: it tints what the light reaches. */
        GEO::vec3f color = GEO::vec3f(1.0f, 1.0f, 1.0f);
        /** Intensity of the light: 1 is the light GLUP shades with. */
        float intensity = 1.0f;
        /** Level added to every lit fragment, in place of GLUP's 0.2. */
        float ambient = 0.2f;
        /** Exponent of the specular highlight, in place of GLUP's 30. */
        float specular_exponent = 30.0f;
    };

    /**
     * @brief The light of the scene (see SceneLight).
     * @details There is one light, as in GLUP: the viewer panel edits this one,
     *          and every object shades with it.
     * @return The light, to read or to edit.
     */
    [[nodiscard]] SceneLight& scene_light();
}

#endif //GEOLIO_SCENE_LIGHT_H
