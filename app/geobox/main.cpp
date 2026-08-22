//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/18.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//

#include <geolio/common/log.h>
#include <geogram/basic/command_line_args.h>
#include <geolio/geobox/application.h>

int main(const int argc, char** argv) {
    spdlog::set_level(spdlog::level::trace);

    GEO::initialize(GEO::GEOGRAM_INSTALL_ALL);
    GEO::CmdLine::import_arg_group("standard");
    GEO::CmdLine::import_arg_group("algo");

    geolio::geobox::GeoBoxApplication app;
    app.start(argc, argv);
    return 0;
}
