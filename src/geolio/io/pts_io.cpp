//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/2.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "pts_io.h"
#include "line_stream.h"
#include <geolio/common/log.h>

namespace geolio
{
    bool PTS_IOHandler::load(
        const std::string& filename,
        GEO::Mesh& mesh,
        const GEO::MeshIOFlags& ioflags
        ) {
        mesh.clear();

        LineInput in(filename);
        if(!in.OK()) {
            LOG::ERROR("Cannot load file `{}`!", filename);
            return false;
        }

        GEO::index_t dim = GEO::NO_INDEX;
        std::vector<double> pts;
        while(!in.eof()) {
            if (!in.get_line())
                break; // back empty line
            in.get_fields();
            if (in.nb_fields() == 0) {
                LOG::ERROR("Line {} :Invalid keyword format!", in.line_number());
                return false;
            }

            if (const std::string kw = in.field(0);
                kw == "v"
                ) {
                if (in.nb_fields() == 3) { // 2D points
                    if (dim == GEO::NO_INDEX)
                        dim = 2;
                    else if (dim != 2) {
                        LOG::ERROR("Line {} :Inconsistent vertex dimension!", in.line_number());
                        return false;
                    }

                    pts.push_back(in.field_as_double(1));
                    pts.push_back(in.field_as_double(2));
                }
                else if (in.nb_fields() == 4) {
                    if (dim == GEO::NO_INDEX)
                        dim = 3;
                    else if (dim != 3) {
                        LOG::ERROR("Line {} :Inconsistent vertex dimension!", in.line_number());
                        return false;
                    }

                    pts.push_back(in.field_as_double(1));
                    pts.push_back(in.field_as_double(2));
                    pts.push_back(in.field_as_double(3));
                }
                else {
                    LOG::ERROR("Line {} :Invalid number of vertex position {}!", in.line_number(), in.nb_fields()-1);
                    return false;
                }
            }
        }

        if (dim == 2 || dim == 3) {
            mesh.vertices.set_dimension(dim);
            mesh.vertices.assign_points(pts.data(), dim, pts.size()/dim);
        }
        else {
            LOG::ERROR("Error vertex dimension {}!", dim);
            return false;
        }

        return true;
    }

    bool PTS_IOHandler::save(
        const GEO::Mesh& mesh,
        const std::string& filename,
        const GEO::MeshIOFlags& ioflags
        ) {
        std::ofstream out(filename.c_str());
        if(!out) {
            LOG::ERROR("Cannot save file `{}`!", filename);
            return false;
        }

        for (const auto& v : mesh.vertices) {
            out << "v" << " ";
            for (GEO::index_t d = 0, d_end = mesh.vertices.dimension(); d < d_end; d++)
                out << mesh.vertices.point_ptr(v)[d] << " ";
            out << "\n";
        }

        out.close();

        return true;
    }
}