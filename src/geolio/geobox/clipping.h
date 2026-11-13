//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/11/13.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_CLIPPING_H
#define GEOLIO_CLIPPING_H

#include <geogram/basic/geometry.h>
#include <geogram_gfx/GLUP/GLUP.h>

namespace geolio::geobox
{
    /**
     * @brief What the clipping plane of the viewer is attached to.
     * @details The plane can be attached to the scene or to the view of it, which is
     *          what GeoGram's clipping and Graphite's camera clipping do: the plane
     *          then stays where it is on the screen, and the scene turns under it.
     */
    enum class ClipAnchor {
        /** @brief The plane is attached to the view: turning the scene sweeps the
         *  plane across it. */
        VIEW,
        /** @brief The plane is attached to the scene: it cuts the same place
         *  whatever the view is. */
        SCENE
    };

    /**
     * @brief The clipping plane of the viewer.
     * @details The clipping of the viewer gathers what comes from two places, which
     *          hold here one and the same plane: Graphite's camera clipping, a plane
     *          laid on an axis of the scene and shifted along it (ported from
     *          OGF::ClippingConfig, the property the "Camera" module of Graphite
     *          exposes with `clipping`), and what GeoGram's clipping lets one set of
     *          it, the GLUP mode the plane discards the elements with and the side
     *          of the plane that is kept.
     *          - `active` turns the clipping on;
     *          - `anchor` is what the plane is attached to, the view or the scene
     *            (see ClipAnchor);
     *          - `normal` is the direction the plane faces: the plane is
     *            perpendicular to it, and nothing ties it to an axis of the scene
     *            (Graphite's own clipping turns its plane with a rotation, which its
     *            panel does not expose);
     *          - `shift` moves the plane along its normal, in the units of the
     *            scene and without a bound: the plane stands at that distance from
     *            the center of the scene along the normal, and the side of the plane
     *            that is kept is the one its normal points to;
     *          - `invert` keeps the other side of the plane;
     *          - `mode` is the GLUP clipping mode, the one GeoGram's clipping also
     *            uses (see GLUPclipMode).
     *          The plane is applied in the space the scene is drawn in, which
     *          decides what turning the view does: with the plane attached to the
     *          view it sweeps across the scene, with the plane attached to the scene
     *          it is the scene that turns around it.
     */
    struct ClipPlane {
        /** @brief Whether the clipping is on. */
        bool active = false;

        /** @brief What the plane is attached to (see ClipAnchor). */
        ClipAnchor anchor = ClipAnchor::VIEW;

        /** @brief Direction the plane faces, in the axes of the scene: the plane is
         *  perpendicular to it. Only its direction matters, its length does not. */
        GEO::vec3 normal = GEO::vec3(0.0, 0.0, 1.0);

        /** @brief Where the plane stands along its normal, in the units of the
         *  scene. */
        float shift = 0.0f;

        /** @brief Whether the side of the plane that is kept is the other one. */
        bool invert = false;

        /** @brief Mode the clipping discards the elements with: the plane cuts the
         *  geometry unless whole cells are asked for (see GLUPclipMode). */
        GLUPclipMode mode = GLUP_CLIP_STANDARD;
    };

    /**
     * @brief Direction the plane of a clip faces, normalized.
     * @details The plane is laid perpendicular to the normal of the clip, and its
     *          shift is a distance along that direction: the normal only gives its
     *          direction, whatever its length.
     * @param[in] clip Clipping to get the normal of.
     * @return The unit normal of the plane, which is the Z axis of the scene when
     *         the normal of the clip is null.
     */
    [[nodiscard]] GEO::vec3 clip_plane_normal(const ClipPlane& clip);

    /**
     * @brief Equation of the plane of a clip.
     * @details The plane is perpendicular to the normal of the clip, at the shift
     *          away from the center of the scene along that direction. The coefficients are the ones the GLUP shaders
     *          test the vertices with, i.e. a vertex is kept when the dot product of
     *          its coordinates with them is positive: the side of the plane that is
     *          kept is therefore the one its normal points to, unless the clip is
     *          inverted.
     *          The plane is offset by `bias` along its normal: a vertex that lies
     *          exactly on the plane, as the vertices of a regular grid do, is then
     *          always on the same side of it. That matters because GLUP classifies
     *          the vertices of the cells on the CPU, against a plane it recomputes
     *          in single precision from the modelview of the frame: a vertex on the
     *          plane would otherwise fall on one side or the other depending on the
     *          rounding of the frame, and its cells would blink as the view moves.
     * @param[in] clip Clipping to compute the plane of.
     * @param[in] center Center of the scene, in the units of the scene: the shift
     *            of the clip is a distance from it.
     * @param[in] bias Offset of the plane along its normal, in the units of the
     *            space it is applied in.
     * @return The coefficients of the plane, in the space the scene is drawn in.
     */
    [[nodiscard]] GEO::vec4 clip_plane_equation(
        const ClipPlane& clip,
        const GEO::vec3& center,
        double bias = 0.0);
}

#endif //GEOLIO_CLIPPING_H
