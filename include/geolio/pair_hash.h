//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/4/20.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOGRAMMESHUTILS_PAIR_HASH_H
#define GEOGRAMMESHUTILS_PAIR_HASH_H

#include <cstddef>
#include <iostream>

namespace geolio
{
    /**
     * @brief Hash functor for std::pair<uint32_t, uint32_t>.
     *
     * Combines the two 32-bit integers into a single 64-bit key by shifting
     * the first element into the high 32 bits and OR'ing the second into the
     * low 32 bits, then applies std::hash<uint64_t> to produce a size_t hash.
     * This allows using std::pair<uint32_t, uint32_t> as a key in unordered
     * containers (e.g., std::unordered_map or std::unordered_set).
     */
    struct PairHash {
        std::size_t operator () (std::pair<uint32_t, uint32_t> const& pair
            ) const {
            const uint64_t key = (static_cast<uint64_t>(pair.first) << 32) | pair.second;
            return std::hash<uint64_t>{}(key);
        }
    };
}

#endif //GEOGRAMMESHUTILS_PAIR_HASH_H