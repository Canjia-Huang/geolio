//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/19.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_CONFIG_H
#define GEOLIO_CONFIG_H

namespace geolio
{
    inline std::string get_configuration_description() {
#ifdef NDEBUG
        return "RELEASE";
#else
        return "DEBUG";
#endif
    }

    static const std::string CMDLINE_FOOTER =   "SUPPORT:\n"
                                                " - Developed by huangcanjia\n"
                                                " - For bug reports or requirements, please contact: Canjia Huang <huangcanjia0214@gmail.com>";

}

#endif //GEOLIO_CONFIG_H
