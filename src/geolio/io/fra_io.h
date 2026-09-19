//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/19.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_FRA_IO_H
#define GEOLIO_FRA_IO_H
#include <geogram/basic/geometry.h>

namespace geolio
{
    /**
     * @brief Loads a cross-field frame set from a FRA file.
     * @details Reads the optional FRA header and frame vectors from the specified file.
     *          Each frame vector is represented by three consecutive double values.
     * @param[in] filepath Path to the FRA file to read.
     * @param[out] vectors_nb_per_element Number of frame vectors stored for each element.
     * @param[out] frames Loaded frame vectors, flattened into a sequence of XYZ components.
     * @return `true` if the file is opened and parsed successfully; `false` otherwise.
     */
    bool fra_load(
        const std::string& filepath,
        GEO::index_t& vectors_nb_per_element,
        std::vector<double>& frames);

    /**
     * @brief Saves a cross-field frame set to a FRA file.
     * @details Writes the frame vectors as groups of three XYZ components. When requested,
     *          the FRA format header and element information line are written before the data.
     * @param[in] filepath Path to the output FRA file.
     * @param[in] vectors_nb_per_element Number of frame vectors stored for each element.
     * @param[in] frames Frame vectors to write, flattened into XYZ components.
     * @param[in] save_header Whether to write the `FRA 1` format header.
     * @param[in] save_infos Whether to write element count, vector count, and symmetry information.
     * @return `true` if the file is written successfully; `false` if it cannot be opened.
     */
    bool fra_save(
        const std::string& filepath,
        GEO::index_t vectors_nb_per_element,
        const std::vector<double>& frames,
        bool save_header = false,
        bool save_infos = false);
}

#endif //GEOLIO_FRA_IO_H
