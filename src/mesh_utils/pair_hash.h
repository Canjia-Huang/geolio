//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/4/20.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOGRAMMESHUTILS_PAIR_HASH_H
#define GEOGRAMMESHUTILS_PAIR_HASH_H

#include <cstddef>
#include <iostream>

namespace GEO::MeshUtils
{
    struct PairHash {
        std::size_t operator () (std::pair<uint, uint> const& pair
            ) const {
            const uint64_t key = (static_cast<uint64_t>(pair.first) << 32) | pair.second;
            return std::hash<uint64_t>{}(key);
        }
    };
}

#endif //GEOGRAMMESHUTILS_PAIR_HASH_H