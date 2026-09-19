//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/19.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <CLI/CLI.hpp>
#include <geolio/miq/miq_interface.h>
#include <geolio/common/config.h>
#include <geogram/mesh/mesh.h>
#include <string>
#include <geolio/common/parse_filepath.h>
#include <geolio/common/log.h>
#include <geogram/basic/command_line_args.h>
#include <geogram/basic/command_line.h>
#include <geogram/mesh/mesh_repair.h>
#include <geolio/io/fra_io.h>

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
    double scale = 30.0;
    MIQParameters params;

    /*== App ======================================================================================================= */
    CLI::App app{APP_NAME};
    argv = app.ensure_utf8(argv);

    app.description(APP_NAME + " [" + get_configuration_description() + "]");
    app.footer(CMDLINE_FOOTER);

    app.add_option(
        "--in-mesh,--i-mesh",
        in_mesh_filepath,
        ""
        )->check(CLI::ExistingFile)->required();

    app.add_option(
        "--in-cross,--i-cross",
        in_cross_filepath,
        ""
        )->check(CLI::ExistingFile);

    app.add_option(
        "--out-mesh,--o-mesh",
        out_mesh_filepath,
        "format need to support uv");

    app.add_option(
        "--out-cross,--o-cross",
        out_cross_filepath,
        "");

    app.add_option(
        "--scale,-s",
        scale,
        "");

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

    LOG::DEBUG("in_mesh_filepath: {}", in_mesh_filepath);
    LOG::DEBUG("in_cross_filepath: {}", in_cross_filepath);
    LOG::DEBUG("out_mesh_filepath: {}", out_mesh_filepath);
    LOG::DEBUG("out_cross_filepath: {}", out_cross_filepath);
    LOG::DEBUG("scale: {}", scale);

    /*== Let's go! ================================================================================================= */
    LOG::TRACE("Start {}!", APP_NAME);

    /* Load mesh */
    GEO::Mesh mesh;
    if (!mesh.load(in_mesh_filepath)) {
        LOG::ERROR("Could not load mesh file: {}", in_mesh_filepath);
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
    LOG::INFO("Loaded mesh: #V: {}, #E: {}, #F: {}, #C: {}", mesh.vertices.nb(), mesh.edges.nb(), mesh.facets.nb(), mesh.cells.nb());

    /* Load cross */
    GEO::index_t vectors_nb_per_facet = GEO::NO_INDEX;
    std::vector<double> cross;
    if (!in_cross_filepath.empty()) {
        if (!fra_load(in_cross_filepath, vectors_nb_per_facet, cross)) {
            LOG::ERROR("Could not load fra file: {}", in_cross_filepath);
            return EXIT_FAILURE;
        }

        LOG::INFO("Loaded cross: #F: {}, #N: {}", cross.size()/(3*vectors_nb_per_facet), vectors_nb_per_facet);

        params.cross = cross.data();
        params.vectors_nb_per_facet = vectors_nb_per_facet;
    }

    /* MIQ */
    GEO::Attribute<double> mesh_fc_uv;
    mesh_fc_uv.create_vector_attribute(mesh.facet_corners.attributes(), "tex_coord", 2);
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