//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/2.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_PTS_IO_H
#define GEOLIO_PTS_IO_H
#include <geogram/mesh/mesh_io.h>

namespace geolio
{
    class PTS_IOHandler : public GEO::MeshIOHandler {
    public:
        bool load(
            const std::string& filename,
            GEO::Mesh& mesh,
            const GEO::MeshIOFlags& ioflags) override;

        bool save(
            const GEO::Mesh& mesh,
            const std::string& filename,
            const GEO::MeshIOFlags& ioflags) override;
    };
}

#endif //GEOLIO_PTS_IO_H
