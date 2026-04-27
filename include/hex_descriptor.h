//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/4/5.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOGRAMMESHUTILS_HEX_DESCRIPTOR_H
#define GEOGRAMMESHUTILS_HEX_DESCRIPTOR_H

#include <array>
#include <geogram/basic/numeric.h>

namespace GEO::MeshUtils
{
    /**
     * Local-vertex to adjacent-local-vertex table for a hexahedron.
     *
     * Each local vertex (LV, 0-7) has exactly three edge-neighbor vertices.
     * `HEX_LV_ADJACENT_LV[lv]` returns those three adjacent LV indices.
     *
     * @note Orientation points toward the hex interior, i.e.
     * `cross(v0 - v, v1 - v) == (v2 - v)`.
     */
    constexpr std::array<std::array<GEO::index_t, 3>, 8> HEX_LV_ADJACENT_LV = {
        {
            {1, 2, 4},
            {0, 5, 3},
            {0, 3, 6},
            {1, 7, 2},
            {0, 6, 5},
            {1, 4, 7},
            {2, 7, 4},
            {3, 5, 6}
        }
    };

    /**
     * Local-vertex to incident-local-edge table for a hexahedron.
     *
     * `HEX_LV_INCIDENT_LE[lv]` lists the three local edges (LE, 0-11)
     * incident to the local vertex `lv`.
     *
     * @note Orientation points toward the hex interior, i.e.
     * `cross(e0, e1) == e2`, with all edges treated as outgoing from `v`.
     */
    constexpr std::array<std::array<GEO::index_t, 3>, 8> HEX_LV_INCIDENT_LE = {
        {
            {0, 3, 8},
            {0, 9, 1},
            {2, 11, 3},
            {1, 10, 2},
            {4, 8, 7},
            {4, 5, 9},
            {6, 7, 11},
            {5, 6, 10},
        }
    };

    /**
     * Local-vertex to incident-local-face table for a hexahedron.
     *
     * `HEX_LV_INCIDENT_LF[lv]` lists the three local faces (LF, 0-5)
     * incident to local vertex `lv`.
     * @note The order follows the inward orientation convention
     * (i.e., ordered toward the interior of the hex, cross(f0's outward normal, f1's outward normal) == -f2's outward normal ).
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
     * Local-edge to incident-local-face table for a hexahedron.
     *
     * `HEX_LE_INCIDENT_LF[le]` returns the two local faces (LF, 0-5)
     * sharing local edge `le`.
     *
     * @note Orientation is opposite to the edge direction, i.e.
     * `cross(f0 normal, f1 normal) == -edge`.
     */
    constexpr std::array<std::array<GEO::index_t, 2>, 12> HEX_LE_INCIDENT_LF = {
        {
            {4, 2}, {4, 1}, {4, 3}, {4, 0}, {2, 5}, {1, 5}, {3, 5}, {0, 5}, {2, 0}, {1, 2}, {3, 1}, {0, 3}
        }
    };

    /**
     * Bitmask encoding for each local edge.
     *
     * `HEX_ENCODED_LE[le]` stores a 2-bit vertex mask of edge `le`:
     * `(1 << lv_a) | (1 << lv_b)`, where `lv_a` and `lv_b` are its endpoints.
     * This supports order-independent edge lookup from two local vertices.
     */
    constexpr std::array<GEO::index_t, 12> HEX_ENCODED_LE = {
        {
            (1<<HEX_LE_INCIDENT_LV[0][0]) | (1<<HEX_LE_INCIDENT_LV[0][1]),
            (1<<HEX_LE_INCIDENT_LV[1][0]) | (1<<HEX_LE_INCIDENT_LV[1][1]),
            (1<<HEX_LE_INCIDENT_LV[2][0]) | (1<<HEX_LE_INCIDENT_LV[2][1]),
            (1<<HEX_LE_INCIDENT_LV[3][0]) | (1<<HEX_LE_INCIDENT_LV[3][1]),
            (1<<HEX_LE_INCIDENT_LV[4][0]) | (1<<HEX_LE_INCIDENT_LV[4][1]),
            (1<<HEX_LE_INCIDENT_LV[5][0]) | (1<<HEX_LE_INCIDENT_LV[5][1]),
            (1<<HEX_LE_INCIDENT_LV[6][0]) | (1<<HEX_LE_INCIDENT_LV[6][1]),
            (1<<HEX_LE_INCIDENT_LV[7][0]) | (1<<HEX_LE_INCIDENT_LV[7][1]),
            (1<<HEX_LE_INCIDENT_LV[8][0]) | (1<<HEX_LE_INCIDENT_LV[8][1]),
            (1<<HEX_LE_INCIDENT_LV[9][0]) | (1<<HEX_LE_INCIDENT_LV[9][1]),
            (1<<HEX_LE_INCIDENT_LV[10][0]) | (1<<HEX_LE_INCIDENT_LV[10][1]),
            (1<<HEX_LE_INCIDENT_LV[11][0]) | (1<<HEX_LE_INCIDENT_LV[11][1]),
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
     *
     * @note Ordered consistently with the local vertex order of each face.
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

    /**
     * Bitmask encoding for each local face.
     *
     * `HEX_ENCODED_LF[lf]` stores a 4-bit vertex mask of face `lf`:
     * `(1 << lv0) | (1 << lv1) | (1 << lv2) | (1 << lv3)`.
     * This enables fast face lookup from either 3 or 4 local vertices.
     */
    constexpr std::array<GEO::index_t, 6> HEX_ENCODED_LF = {
        {
            (1<<HEX_LF_INCIDENT_LV[0][0]) | (1<<HEX_LF_INCIDENT_LV[0][1]) | (1<<HEX_LF_INCIDENT_LV[0][2]) | (1<<HEX_LF_INCIDENT_LV[0][3]),
            (1<<HEX_LF_INCIDENT_LV[1][0]) | (1<<HEX_LF_INCIDENT_LV[1][1]) | (1<<HEX_LF_INCIDENT_LV[1][2]) | (1<<HEX_LF_INCIDENT_LV[1][3]),
            (1<<HEX_LF_INCIDENT_LV[2][0]) | (1<<HEX_LF_INCIDENT_LV[2][1]) | (1<<HEX_LF_INCIDENT_LV[2][2]) | (1<<HEX_LF_INCIDENT_LV[2][3]),
            (1<<HEX_LF_INCIDENT_LV[3][0]) | (1<<HEX_LF_INCIDENT_LV[3][1]) | (1<<HEX_LF_INCIDENT_LV[3][2]) | (1<<HEX_LF_INCIDENT_LV[3][3]),
            (1<<HEX_LF_INCIDENT_LV[4][0]) | (1<<HEX_LF_INCIDENT_LV[4][1]) | (1<<HEX_LF_INCIDENT_LV[4][2]) | (1<<HEX_LF_INCIDENT_LV[4][3]),
            (1<<HEX_LF_INCIDENT_LV[5][0]) | (1<<HEX_LF_INCIDENT_LV[5][1]) | (1<<HEX_LF_INCIDENT_LV[5][2]) | (1<<HEX_LF_INCIDENT_LV[5][3])
        }
    };
}

#endif //GEOGRAMMESHUTILS_HEX_DESCRIPTOR_H