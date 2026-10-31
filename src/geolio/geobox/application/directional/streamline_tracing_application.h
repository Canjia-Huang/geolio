//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/31.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_STREAMLINE_TRACING_APPLICATION_H
#define GEOLIO_STREAMLINE_TRACING_APPLICATION_H
#include <geolio/geobox/application/base_application.h>
#include <geolio/geobox/object/mesh_object.h>
#include <geolio/common/log.h>

namespace geolio::geobox
{
    class StreamlineTracingApplication : public BaseApplication {
    public:
        explicit StreamlineTracingApplication(ApplicationObjects& objects)
            : BaseApplication("streamline tracing", objects)
        {
            menu_tooltip_ = "Tracing directional field.";
        }

    protected:
        void draw_window_contents() override {

        }


    };
}

#endif //GEOLIO_STREAMLINE_TRACING_APPLICATION_H
