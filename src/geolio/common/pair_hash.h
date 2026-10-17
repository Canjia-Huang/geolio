//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/4/20.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_PAIR_HASH_H
#define GEOLIO_PAIR_HASH_H
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iostream>
#include <utility>

namespace geolio
{
    /**
     * @brief Computes a hash value for an ordered pair of 32-bit unsigned integers.
     *
     * Packs @p a into the high 32 bits and @p b into the low 32 bits of a
     * 64-bit key, then applies Fibonacci hashing to improve bit distribution.
     * The order of the arguments is significant: swapping them may produce a
     * different hash value.
     *
     * @param[in] a The first value in the pair.
     * @param[in] b The second value in the pair.
     * @return A 64-bit hash value derived from the ordered pair (@p a, @p b).
     */
    inline uint64_t compute_pair_hash(const uint32_t& a, const uint32_t& b) {
        // Fibonacci hashing of the packed key: the low bits of v1 alone are too regular
        // for linear probing on a structured grid mesh.
        const auto key = (static_cast<uint64_t>(a) << 32) | static_cast<uint64_t>(b);
        return static_cast<uint64_t>((key * 0x9E3779B97F4A7C15ull) >> 32);
    }

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
        /**
         * @brief Computes the hash value of a pair.
         * @details Packs the two 32-bit integers into a single 64-bit key by
         *          shifting the first element into the high 32 bits and OR-ing
         *          the second into the low 32 bits, then hashes that key with
         *          std::hash<uint64_t>.
         * @param[in] pair The pair to hash.
         * @return The computed hash value.
         */
        uint64_t operator () (std::pair<uint32_t, uint32_t> const& pair) const {
            return compute_pair_hash(pair.first, pair.second);
        }
    };
}

#endif //GEOLIO_PAIR_HASH_H