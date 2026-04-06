//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/4/6.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOGRAMMESHUTILS_TET_DESCRIPTOR_H
#define GEOGRAMMESHUTILS_TET_DESCRIPTOR_H

#include <array>
#include <geogram/basic/numeric.h>

namespace GEO::MeshUtils
{
    /**
     * Local-vertex to adjacent-local-vertex table for a tetrahedron.
     *
     * Each local vertex (LV, 0-3) has exactly three adjacent vertices.
     * `TET_LV_ADJACENT_LV[lv]` returns those three adjacent LV indices.
     */
    constexpr std::array<std::array<GEO::index_t, 3>, 4> TET_LV_ADJACENT_LV = {
        {
            {1, 3, 2},
            {0, 2, 3},
            {3, 1, 0},
            {0, 1, 2}
        }
    };

    /**
     * Local-vertex to incident-local-edge table for a tetrahedron.
     *
     * `TET_LV_INCIDENT_LE[lv]` lists the three local edges (LE, 0-5)
     * incident to local vertex `lv`.
     * The order follows the inward orientation convention.
     */
    constexpr std::array<std::array<GEO::index_t, 3>, 4> TET_LV_INCIDENT_LE = {
        {
            {3, 4, 5},
            {0, 3, 2},
            {0, 1, 4},
            {1, 2, 5}
        }
    };

    /**
     * Local-vertex to incident-local-face table for a tetrahedron.
     *
     * `TET_LV_INCIDENT_LF[lv]` lists the three local faces (LF, 0-3)
     * incident to local vertex `lv`.
     * The order follows the inward orientation convention.
     */
    constexpr std::array<std::array<GEO::index_t, 3>, 4> TET_LV_INCIDENT_LF = {
        {
            {1, 2, 3},
            {0, 3, 2},
            {0, 1, 3},
            {0, 2, 1}
        }
    };

    /**
     * Local-edge to endpoint-local-vertex table for a tetrahedron.
     *
     * `TET_LE_INCIDENT_LV[le]` returns the two endpoint local vertices
     * of local edge `le`.
     */
    constexpr std::array<std::array<GEO::index_t, 2>, 6> TET_LE_INCIDENT_LV = {
        {
            {1, 2}, {2, 3}, {3, 1}, {0, 1}, {0, 2}, {0, 3}
        }
    };

    /**
     * Local-edge to incident-local-face table for a tetrahedron.
     *
     * `TET_LE_INCIDENT_LF[le]` returns the two local faces (LF, 0-3)
     * sharing local edge `le`.
     */
    constexpr std::array<std::array<GEO::index_t, 2>, 6> TET_LE_INCIDENT_LF = {
        {
            {0, 3}, {0, 1}, {0, 2}, {2, 3}, {3, 1}, {1, 2}
        }
    };

    /**
     * Local-face to corner-local-vertex table for a tetrahedron.
     *
     * `TET_LF_INCIDENT_LV[lf]` returns the three corner local vertices
     * on local face `lf`, in the face ordering used by this project.
     */
    constexpr std::array<std::array<GEO::index_t, 3>, 4> TET_LF_INCIDENT_LV = {
        {
            {1, 3, 2},
            {0, 2, 3},
            {3, 1, 0},
            {0, 1, 2}
        }
    };

    /**
     * Local-face to boundary-local-edge table for a tetrahedron.
     *
     * `TET_LF_INCIDENT_LE[lf]` returns the three local edges that bound
     * local face `lf`, in the face-edge ordering used by this project.
     */
    constexpr std::array<std::array<GEO::index_t, 3>, 4> TET_LF_INCIDENT_LE = {
        {
            {2, 1, 0},
            {4, 1, 5},
            {2, 3, 5},
            {3, 0, 4}
        }
    };
}

#endif //GEOGRAMMESHUTILS_TET_DESCRIPTOR_H