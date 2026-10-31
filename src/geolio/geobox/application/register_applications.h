//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/29.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_APPLICATIONS_H
#define GEOLIO_APPLICATIONS_H
#include "application_registry.h"
#include "mesh/local_operation_optimization_application.h"
#include "mesh/mesh_connect_application.h"
#include "mesh/mesh_random_sampling_application.h"
#include "mesh/mesh_repair_colocate_application.h"
#include "mesh/mesh_repair_triangulate_application.h"
#include "mesh/mesh_compute_border_application.h"
#include "points/centroidal_voronoi_tesselation_application.h"
#include "attributes/local_feature_size_application.h"
#include "points/restricted_voronoi_diagram_application.h"

namespace geolio::geobox
{
    /**
     * @brief Registers every built-in sub-application (BaseApplication
     *        subclasses) into the application registry.
     * @details Mirrors register_additional_MeshIOHandlers() from
     *          geolio/io/io.h: each concrete application is declared here
     *          with its sub-menu path, and GeoBoxApplication instantiates all
     *          of them by iterating the registry. The display name and the
     *          instance id are decided by the application class itself (see
     *          BaseApplication), so they are not repeated here.
     *
     *          This file is the single place to edit when adding a
     *          sub-application or a sub-menu:
     *            - the trailing arguments of
     *              geolio_register_BaseApplication_creator() are the labels of
     *              the parent sub-menus, outermost first, so
     *              ("Mesh", "Optimization") means
     *              Applications > Mesh > Optimization > entry;
     *            - leaving them out puts the entry in the "ungrouped"
     *              sub-menu;
     *            - applications sharing a path automatically share the
     *              sub-menu (created once, in registration order), and a
     *              sub-menu with no application in it never appears.
     *          The top menu bar (GeoBoxApplication::draw_application_menus())
     *          only walks the resulting tree, so it never needs to change.
     */
    inline void register_additional_Applications() {
        geolio_register_BaseApplication_creator(LocalFeatureSizeApplication, "attributes");
        geolio_register_BaseApplication_creator(CentroidalVoronoiTesselationApplication, "points");
        geolio_register_BaseApplication_creator(RestrictedVoronoiDiagramApplication, "points");
        geolio_register_BaseApplication_creator(MeshComputeBorderApplication, "mesh");
        geolio_register_BaseApplication_creator(MeshConnectApplication, "mesh");
        geolio_register_BaseApplication_creator(LocalOperationOptimizationApplication, "mesh", "optimization");
        geolio_register_BaseApplication_creator(MeshRepairColocateApplication, "mesh", "repair");
        geolio_register_BaseApplication_creator(MeshRepairTriangulateApplication, "mesh", "repair");
        geolio_register_BaseApplication_creator(MeshRandomSamplingApplication, "mesh");
    }
}

#endif //GEOLIO_APPLICATIONS_H
