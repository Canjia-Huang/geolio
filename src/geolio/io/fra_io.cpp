//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/19.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "fra_io.h"
#include <fstream>
#include "line_stream.h"
#include <geolio/common/log.h>
#include <geolio/common/parse_filepath.h>

namespace geolio
{
    bool fra_load(
        const std::string& filepath,
        GEO::index_t& vectors_nb_per_element,
        std::vector<double>& frames
        ) {
        vectors_nb_per_element = GEO::NO_INDEX;
        frames.clear();

        LineInput in(filepath);
        if (!in.OK()) {
            LOG::ERROR("Cannot load file `{}`!", filepath);
            return false;
        }

        /* Header */
        while(!in.eof()) {
            if (!in.get_line())
                break;
            in.get_fields();
            if (in.nb_fields() == 0) {
                LOG::ERROR("Line {} :Expect non-empty line!", in.line_number());
                return false;
            }

            if (const std::string kw = in.field(0);
                kw == "FRA") // skip this line
                continue;

            if (in.nb_fields() == 3) // elements_nb, axes_nb, n_symmetry, skip this line
                continue;

            if (in.nb_fields() % 3 != 0) {
                LOG::ERROR("Line {} :Invalid number of axes, expected 3*n!", in.line_number());
                return false;
            }

            if (vectors_nb_per_element == GEO::NO_INDEX)
                vectors_nb_per_element = in.nb_fields()/3;
            else {
                if (vectors_nb_per_element != in.nb_fields()/3) {
                    LOG::ERROR("Line {} :Invalid number of vectors, expected {}!", in.line_number(), vectors_nb_per_element);
                    return false;
                }
            }

            for (GEO::index_t i = 0, i_end = in.nb_fields(); i < i_end; ++i)
                frames.push_back(in.field_as_double(i));
        }

        return true;
    }

    bool fra_save(
        const std::string& filepath,
        const GEO::index_t vectors_nb_per_element,
        const std::vector<double>& frames,
        const bool save_header,
        const bool save_infos
        ) {
        if (const auto ext = get_extension(filepath);
            ext != "fra")
            LOG::WARN("Currently, only fra format output is supported, but the specified file extension `{}` is not. Is this a mistake?", ext);

        std::ofstream out(filepath);
        if (!out.good()) {
            LOG::ERROR("Could not open file `{}` for writing!", filepath);
            return false;
        }

        if (save_header)
            out << "FRA 1" << "\n";
        if (save_infos) {
            const GEO::index_t elements_nb = frames.size() / (3*vectors_nb_per_element);
            out << elements_nb << " " << vectors_nb_per_element << " " << "4" << "\n"; // 4-NoSy
        }

        for (GEO::index_t i = 0, i_end = frames.size(); i < i_end;) {
            for (GEO::index_t j = 0; j < vectors_nb_per_element; ++j) {
                out << frames[i] << " " << frames[i+1] << " " << frames[i+2] << " ";
                i += 3;
            }
            out << "\n";
        }

        out.close();
        return true;
    }
}
