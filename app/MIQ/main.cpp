//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/19.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <string>
#include <CLI/CLI.hpp>
#include <geogram/basic/command_line_args.h>
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_repair.h>
#include <geolio/common/config.h>
#include <geolio/common/log.h>
#include <geolio/common/parse_filepath.h>
#include <geolio/io/fra_io.h>
#include <geolio/MIQ/miq_interface.h>

using namespace geolio;

const std::string APP_NAME = "MIQ";

int main(int argc, char* argv[]) {
    /*== Initialize ================================================================================================ */

    /* Init spdlog */
    spdlog::set_level(spdlog::level::trace);

    /* Init Geogram */
    GEO::initialize(GEO::GEOGRAM_INSTALL_ALL);
    GEO::CmdLine::import_arg_group("standard");
    GEO::CmdLine::import_arg_group("algo");

    std::string in_mesh_filepath;
    std::string in_cross_filepath;
    std::string out_mesh_filepath;
    std::string out_cross_filepath;
    MIQParameters params;
    double snap_sharp_feature = 180;

    /*== App ======================================================================================================= */
    CLI::App app{APP_NAME};
    argv = app.ensure_utf8(argv);

    app.description(APP_NAME + " [" + get_configuration_description() + "]");
    app.footer(CMDLINE_FOOTER);

    app.add_option(
        "--in-mesh,--i-mesh",
        in_mesh_filepath,
        "Input mesh file."
        )->check(CLI::ExistingFile)->required();

    app.add_option(
        "--in-cross,--i-cross",
        in_cross_filepath,
        "(Optional) Input cross field file in FRA format. "
        "If not specified, the nRosy field will be automatically computed (based on the libigl library)."
        )->check(CLI::ExistingFile);

    app.add_option(
        "--out-mesh,--o-mesh",
        out_mesh_filepath,
        "Output mesh file; the format may need to support UV coordinates."
        );

    app.add_option(
        "--out-cross,--o-cross",
        out_cross_filepath,
        "Output cross field file in FRA format."
        );

    app.add_option(
        "--scale,-s",
        params.scale,
        "Global scaling factor for the gradient; controls quad resolution."
        )->default_val(params.scale);

    app.add_flag(
        "--snap-bdy,!--relax-bdy",
        params.snap_boundary,
        "Specify whether the integer grid parameterization is aligned with the open boundary (if any)."
        )->default_val(params.snap_boundary);

    app.add_option(
        "--snap-feature",
        snap_sharp_feature,
        "Specify the recognition angle for identifying the sharp features to which the parameterization is to be aligned. "
        "(set to 180 if sharp features are not to be recognized, only for 3D surface mesh)"
        )->default_val(snap_sharp_feature);

    CLI11_PARSE(app, argc, argv);

    /* == Parse == */
    const std::string model_name = get_filename(in_mesh_filepath);

    if (!out_mesh_filepath.empty()) {
        if (std::filesystem::is_directory(out_mesh_filepath))
            out_mesh_filepath += "/" + model_name + "_miq.obj";
    }
    if (!out_cross_filepath.empty()) {
        if (std::filesystem::is_directory(out_cross_filepath))
            out_cross_filepath += "/" + model_name + "_miq.fra";

        params.output_cross = true;
    }

    LOG::DEBUG("===== Parse parameters =====");
    LOG::DEBUG("in_mesh_filepath: {}", in_mesh_filepath);
    LOG::DEBUG("in_cross_filepath: {}", in_cross_filepath);
    LOG::DEBUG("out_mesh_filepath: {}", out_mesh_filepath);
    LOG::DEBUG("out_cross_filepath: {}", out_cross_filepath);
    LOG::DEBUG("scale: {}", params.scale);
    LOG::DEBUG("snap_to_boundary: {}", params.snap_boundary);
    LOG::DEBUG("snap_sharp_feature: {}", snap_sharp_feature);
    LOG::DEBUG("============================");

    /*== Let's go! ================================================================================================= */
    LOG::TRACE("Start {}!", APP_NAME);

    /* Load mesh */
    GEO::Mesh mesh;
    if (!mesh.load(in_mesh_filepath)) {
        LOG::ERROR("Could not load mesh file: {}!", in_mesh_filepath);
        return EXIT_FAILURE;
    }
    if (mesh.facets.nb() == 0) {
        LOG::ERROR("There are no facets in the input mesh!");
        return EXIT_FAILURE;
    }
    if (!mesh.facets.are_simplices()) {
        LOG::WARN("The input mesh is not entirely a triangular mesh, so additional triangulation is required.");
        GEO::mesh_repair(mesh, GEO::MESH_REPAIR_TRIANGULATE);

        if (!mesh.facets.are_simplices()) {
            LOG::ERROR("Triangulation failed!");
            return EXIT_FAILURE;
        }
    }
    LOG::INFO("Loaded mesh: #V: {}, #E: {}, #F: {}, #C: {}.", mesh.vertices.nb(), mesh.edges.nb(), mesh.facets.nb(), mesh.cells.nb());

    /* Load cross */
    GEO::index_t vectors_nb_per_facet = GEO::NO_INDEX;
    std::vector<double> cross;
    if (!in_cross_filepath.empty()) {
        if (!fra_load(in_cross_filepath, vectors_nb_per_facet, cross)) {
            LOG::ERROR("Could not load fra file: {}!", in_cross_filepath);
            return EXIT_FAILURE;
        }

        LOG::INFO("Loaded cross: #F: {}, #N: {}.", cross.size()/(3*vectors_nb_per_facet), vectors_nb_per_facet);

        params.cross = cross.data();
        params.vectors_nb_per_facet = vectors_nb_per_facet;
    }

    /* Recognition sharp features */
    if (snap_sharp_feature < 180) {
        if (mesh.vertices.dimension() > 2) {
            const double limit_angle = M_PI - snap_sharp_feature*M_PI/180.0;

            std::vector<GEO::vec3> mesh_f_normal(mesh.facets.nb());
            for (const auto& f : mesh.facets)
                mesh_f_normal[f] = GEO::Geom::triangle_normal(
                    mesh.facets.point(f, 0),
                    mesh.facets.point(f, 1),
                    mesh.facets.point(f, 2));

            std::vector<bool> added_fc(mesh.facet_corners.nb(), false);
            for (const auto& f : mesh.facets) {
                for (GEO::index_t lv = 0; lv < 3; ++lv) {
                    if (added_fc[mesh.facets.corner(f, lv)])
                        continue;

                    if (const auto& nf = mesh.facets.adjacent(f, lv);
                        nf != GEO::NO_FACET
                        ) {
                        const auto& f_normal = mesh_f_normal[f];
                        const auto& nf_normal = mesh_f_normal[nf];
                        if (const auto angle = GEO::Geom::angle(f_normal, nf_normal);
                            angle > limit_angle
                            ) { // sharp
                            params.hard_features.push_back({static_cast<int>(f), static_cast<int>(lv)});

                            added_fc[mesh.facets.corner(f, lv)] = true;
                            const auto nlv = mesh.facets.find_vertex(nf, mesh.facets.vertex(f, (lv+1)%3));
                            assert(nlv != GEO::NO_INDEX);
                            added_fc[mesh.facets.corner(nf, nlv)] = true;
                        }
                    }
                }
            }

            LOG::INFO("Recognition {} sharp feature edges.", params.hard_features.size());
        }
        else
            LOG::WARN("The snap feature only supports 3D meshes; the current mesh is 2D, so it is skipped.");
    }

    /* MIQ */
    GEO::Attribute<double> mesh_fc_uv;
    { // bind uv attribute
        mesh_fc_uv.bind_if_is_defined(mesh.facet_corners.attributes(), "tex_coord");
        if (mesh_fc_uv.is_bound()) {
            if (mesh_fc_uv.dimension() != 2)
                mesh_fc_uv.destroy();
        }
        if (!mesh_fc_uv.is_bound())
            mesh_fc_uv.create_vector_attribute(mesh.facet_corners.attributes(), "tex_coord", 2);
    }

    if (mesh.vertices.dimension() == 2)
        miq<2>(mesh, mesh_fc_uv, params);
    else
        miq<3>(mesh, mesh_fc_uv, params);

    /* Output */
    if (!out_mesh_filepath.empty()) {
        if (!mesh.save(out_mesh_filepath))
            LOG::ERROR("Could not save mesh file: {}", out_mesh_filepath);
        else
            LOG::INFO("Saved mesh file: {}", out_mesh_filepath);
    }
    if (!out_cross_filepath.empty()) {
        if (!fra_save(out_cross_filepath, params.out_vectors_nb_per_facet, params.out_cross))
            LOG::ERROR("Could not save fra file: {}", out_cross_filepath);
        else
            LOG::INFO("Saved fra file: {}", out_cross_filepath);
    }

    LOG::TRACE("{} done!", APP_NAME);
}