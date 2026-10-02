//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/26.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_IO_H
#define GEOLIO_IO_H
#include <geogram/mesh/mesh_io.h>
#include "aedtplt_io.h"
#include "hexex_io.h"
#include "off_io.h"
#include "ovm_io.h"
#include "pts_io.h"

namespace geolio
{
    inline void register_additional_MeshIOHandlers(
        ) {
        geo_register_MeshIOHandler_creator(AEDTPLT_IOHandler, "aedtplt");
        geo_register_MeshIOHandler_creator(HEXEX_IOHandler, "hexex");
        geo_register_MeshIOHandler_creator(OFF_IOHandler, "off");
        geo_register_MeshIOHandler_creator(OVM_IOHandler, "ovm");
        geo_register_MeshIOHandler_creator(PTS_IOHandler, "pts");
    }
}

#endif //GEOLIO_IO_H
