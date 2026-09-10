//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/23.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_LOCAL_OPERATION_OPTIMIZATION_APPLICATION_H
#define GEOLIO_LOCAL_OPERATION_OPTIMIZATION_APPLICATION_H

#include "base_application.h"
#include <geolio/geobox/object/base_object.h>
#include <geogram/mesh/mesh.h>

#include "geolio/local_operation_optimization/split_operation.h"
#include "geolio/local_operation_optimization/swap_operation.h"
#include "geolio/local_operation_optimization/tri_local_operation_optimization.h"

namespace geolio::geobox
{
    class LocalOperationOptimizationApplication : public BaseApplication {
    public:
        LocalOperationOptimizationApplication(
            std::string application_name,
            const std::vector<std::shared_ptr<BaseObject>>& objects);

    protected:
        void draw_window_contents() override;

        void init_target_edge_length();

        template <GEO::index_t DIM>
        void perform(GEO::Mesh& mesh);

        const std::vector<std::shared_ptr<BaseObject>>& objects_;

        /** The mesh object currently selected in the combo box. */
        std::weak_ptr<BaseObject> selected_mesh_object_;

        /* Parameters */
        GEO::index_t rounds_nb_ = 5;
        double target_edge_length_ = 1;
        bool fix_boundary_elements_ = true;
        bool fix_sharp_elements_ = true;
        double sharp_angle_ = 135;
        bool allow_split_fixed_edges_ = true;
        bool allow_collapse_fixed_edges_ = true;
        bool allow_smooth_fixed_edges_vertices_ = true;
    };
}

#endif //GEOLIO_LOCAL_OPERATION_OPTIMIZATION_APPLICATION_H
