//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/29.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_UTILS_H
#define GEOLIO_UTILS_H
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <random>
#include <string>
#include <thread>

namespace geolio
{
    static std::atomic<std::uint64_t> seed_counter{0};

    static thread_local std::mt19937 generator = [] {
        std::seed_seq seq{
            static_cast<std::uint64_t>(std::random_device{}()),
            static_cast<std::uint64_t>(std::hash<std::thread::id>{}(std::this_thread::get_id())),
            seed_counter.fetch_add(1, std::memory_order_relaxed)
        };
        return std::mt19937(seq);
    }();

    /**
     * @brief Generates a random alphanumeric string of a given length.
     * @details Uses a function-local static std::mt19937 generator seeded from
     *          std::random_device. Each character is drawn uniformly at random
     *          from the alphanumeric character set using a uniform integer
     *          distribution and appended to the result.
     * @param[in] length The number of characters in the generated string.
     * @return The generated random string.
     */
    inline std::string generate_random_string(
        const std::size_t length
        ) {
        static const std::string CHARACTERS = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";

        std::uniform_int_distribution<std::size_t> distribution(0, CHARACTERS.size() - 1);

        std::string random_string;
        random_string.reserve(length);
        for (std::size_t i = 0; i < length; ++i)
            random_string += CHARACTERS[distribution(generator)];

        return random_string;
    }
}

#endif //GEOLIO_UTILS_H
