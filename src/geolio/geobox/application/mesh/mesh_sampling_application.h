//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_MESH_SAMPLING_APPLICATION_H
#define GEOLIO_MESH_SAMPLING_APPLICATION_H
#include "mesh_application.h"
#include <geolio/geobox/object/mesh_object.h>
#include <geolio/common/log.h>

namespace geolio::geobox
{
    class MeshSamplingApplication : public MeshApplication {
    public:
        explicit MeshSamplingApplication(std::vector<std::shared_ptr<BaseObject>>& objects)
            : MeshApplication("sampling", objects)
        {
            menu_tooltip_ = "Sampling on mesh.";
        }

    protected:
        bool is_mesh_valid(const GEO::Mesh& mesh) override {
            return mesh.facets.nb() > 0 || mesh.cells.nb() > 0;
        }
    };
}

#endif //GEOLIO_MESH_SAMPLING_APPLICATION_H
