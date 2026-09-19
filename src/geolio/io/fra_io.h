//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/19.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_FRA_IO_H
#define GEOLIO_FRA_IO_H
#include <geogram/basic/geometry.h>

namespace geolio
{
    bool fra_load(
        const std::string& filepath,
        GEO::index_t& vectors_nb_per_element,
        std::vector<double>& frames);

    bool fra_save(
        const std::string& filepath,
        GEO::index_t vectors_nb_per_element,
        const std::vector<double>& frames,
        bool save_header = false,
        bool save_infos = false);
}

#endif //GEOLIO_FRA_IO_H
