//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/4/5.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOGRAMMESHUTILS_HEX_DESCRIPTOR_H
#define GEOGRAMMESHUTILS_HEX_DESCRIPTOR_H

namespace GEO::MeshUtils
{
    /**
     * Local-vertex to adjacent-local-vertex table for a hexahedron.
     *
     * Each local vertex (LV, 0-7) has exactly three edge-neighbor vertices.
     * `HEX_LV_ADJACENT_LV[lv]` returns those three adjacent LV indices.
     */
    constexpr std::array<std::array<GEO::index_t, 3>, 8> HEX_LV_ADJACENT_LV = {
        {
            {1, 2, 4},
            {0, 3, 5},
            {0, 3, 6},
            {1, 2, 7},
            {0, 5, 6},
            {1, 4, 7},
            {2, 4, 7},
            {3, 5, 6}
        }
    };

    /**
     * Local-vertex to incident-local-edge table for a hexahedron.
     *
     * `HEX_LV_INCIDENT_LE[lv]` lists the three local edges (LE, 0-11)
     * incident to the local vertex `lv`.
     */
    constexpr std::array<std::array<GEO::index_t, 3>, 8> HEX_LV_INCIDENT_LE = {
        {
            {0, 3, 8},
            {0, 1, 9},
            {2, 3, 11},
            {1, 2, 10},
            {4, 7, 8},
            {4, 5, 9},
            {6, 7, 11},
            {5, 6, 10},
        }
    };

    /**
     * Local-vertex to incident-local-face table for a hexahedron.
     *
     * `HEX_LV_INCIDENT_LF[lv]` lists the three local faces (LF, 0-5)
     * incident to local vertex `lv`. The order follows the inward orientation
     * convention (i.e., ordered toward the interior of the hex).
     */
    constexpr std::array<std::array<GEO::index_t, 3>, 8> HEX_LV_INCIDENT_LF = {
        {
            {0, 2, 4},
            {1, 4, 2},
            {0, 4, 3},
            {1, 3, 4},
            {0, 5, 2},
            {1, 2, 5},
            {0, 3, 5},
            {1, 5, 3}
        }
    };

    /**
     * Local-edge to endpoint-local-vertex table for a hexahedron.
     *
     * `HEX_LE_INCIDENT_LV[le]` returns the two endpoint local vertices
     * of local edge `le`.
     */
    constexpr std::array<std::array<GEO::index_t, 2>, 12> HEX_LE_INCIDENT_LV = {
        {
            {0, 1}, {1, 3}, {3, 2}, {2, 0}, {4, 5}, {5, 7}, {7, 6}, {6, 4}, {0, 4}, {1, 5}, {3, 7}, {2, 6}
        }
    };

    /**
     * Local-face to corner-local-vertex table for a hexahedron.
     *
     * `HEX_LF_INCIDENT_LV[lf]` returns the four corner local vertices
     * on local face `lf`, in the face ordering used by this project.
     */
    constexpr std::array<std::array<GEO::index_t, 4>, 6> HEX_LF_INCIDENT_LV = {
        {
            {0, 2, 6, 4},
            {3, 1, 5, 7},
            {1, 0, 4, 5},
            {2, 3, 7, 6},
            {1, 3, 2, 0},
            {4, 6, 7, 5}
        }
    };

    /**
     * Local-face to boundary-local-edge table for a hexahedron.
     *
     * `HEX_LF_INCIDENT_LE[lf]` returns the four local edges that bound
     * local face `lf`, in the face-edge ordering used by this project.
     */
    constexpr std::array<std::array<GEO::index_t, 4>, 6> HEX_LF_INCIDENT_LE = {
        {
            {3, 11, 7, 8},
            {1, 9, 5, 10},
            {0, 8, 4, 9},
            {2, 10, 6, 11},
            {1, 2, 3, 0},
            {7, 6, 5, 4}
        }
    };

    /**
     * Opposite-local-face lookup table.
     *
     * `HEX_LF_OPPOSITE_LF[lf]` gives the local face index opposite to `lf`.
     */
    constexpr std::array<GEO::index_t, 6> HEX_LF_OPPOSITE_LF = {
        {1, 0, 3, 2, 5, 4}
    };
}

#endif //GEOGRAMMESHUTILS_HEX_DESCRIPTOR_H