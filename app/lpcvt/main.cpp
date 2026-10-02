//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/3.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <cstdlib>
#include <exception>
#include <fstream>
#include <string>
#include <CLI/CLI.hpp>
#include <geogram/basic/command_line.h>
#include <geogram/basic/command_line_args.h>
#include <geogram/basic/common.h>
#include <geogram/mesh/mesh_io.h>
#include <geogram/mesh/mesh_repair.h>
#include <geolio/common/config.h>
#include <geolio/common/log.h>
#include <geolio/io/io.h>
#include <geolio/LpCVT/lp_cvt.h>
#include <geolio/LpCVT/lp_measure.h>
#include <geolio/common/parse_filepath.h>

using namespace geolio;

const std::string APP_NAME = "LpCVT";

namespace
{
    /**
     * @brief An LpCVT that reports the objective as the optimizer evaluates it.
     * @details The objective evaluation is protected, since client code is meant to
     *          drive it through Newton_iterations(). Deriving from it is however
     *          exactly the intended way of extending the algorithm, and it gives the
     *          command line front end a way to report that the optimization actually
     *          progressed.
     */
    class ReportingLpCVT : public geolio::LpCentroidalVoronoiTesselation {
    public:
        using LpCentroidalVoronoiTesselation::LpCentroidalVoronoiTesselation;

        /**
         * @brief Returns the objective values observed so far.
         * @param[out] first Value of the first evaluation.
         * @param[out] last Value of the most recent evaluation.
         * @param[out] nb_evals Number of evaluations performed.
         */
        void report(double& first, double& last, unsigned int& nb_evals) const {
            first = first_f_;
            last = last_f_;
            nb_evals = nb_evals_;
        }

    protected:
        /**
         * @brief Evaluates the objective and records it.
         * @copydetails geolio::LpCentroidalVoronoiTesselation::funcgrad
         */
        void funcgrad(
            const GEO::index_t n,
            double* x,
            double& f,
            double* g
            ) override {
            LpCentroidalVoronoiTesselation::funcgrad(n, x, f, g);
            if (nb_evals_ == 0) {
                first_f_ = f;
            }
            last_f_ = f;
            ++nb_evals_;
        }

    private:
        /** @brief Objective value of the first evaluation. */
        double first_f_ = 0.0;
        /** @brief Objective value of the most recent evaluation. */
        double last_f_ = 0.0;
        /** @brief Number of evaluations performed so far. */
        unsigned int nb_evals_ = 0;
    };
}

int main(int argc, char** argv) {
    /*== Initialize ================================================================================================ */

    /* Init spdlog */
    spdlog::set_level(spdlog::level::trace);

    /* Init Geogram */
    GEO::initialize(GEO::GEOGRAM_INSTALL_ALL);
    GEO::CmdLine::import_arg_group("standard");
    GEO::CmdLine::import_arg_group("algo");
    geolio::register_additional_MeshIOHandlers();

    GEO::index_t exponent = 2;
    bool volumetric = false;
    GEO::index_t nb_iterations = 100;
    GEO::index_t nb_samples = 10000;
    std::string predicates = "fast";
    std::string in_mesh_filepath;
    std::string in_pts_filepath;
    std::string out_pts_filepath;
    std::string out_rvd_filepath;
    std::string out_rdt_filepath;

    /*== App ======================================================================================================= */
    CLI::App app{APP_NAME};
    argv = app.ensure_utf8(argv);

    app.description(APP_NAME + " [" + get_configuration_description() + "]");
    app.footer(CMDLINE_FOOTER);

    app.add_option(
        "--i-mesh,--in-mesh",
        in_mesh_filepath,
        "Reference mesh; it has to be a triangulated surface, or a tetrahedralized "
        "volume when --volumetric is used."
        )->required()->check(CLI::ExistingFile)->required();

    app.add_option(
        "--i-pts,--in-pts",
        in_pts_filepath,
        "Optional input point file, in the reference implementation's \".pts\" format: "
        "skips the initial sampling and starts from these points."
        )->check(CLI::ExistingFile);

    app.add_option(
        "-p,--exponent",
        exponent,
        "Lp norm exponent: a larger even exponent sharpens the tessellation (default: "+std::to_string(exponent)+")."
        )->check(CLI::IsMember({2, 4, 6, 8, 10, 12, 14, 16}));

    app.add_flag(
        "-v,--volumetric",
        volumetric,
        "Mesh the volume bounded by the surface instead of the surface itself (default: false); "
        "the mesh then has to be filled with tetrahedra."
        );

    app.add_option(
        "-m,--iterations",
        nb_iterations,
        "Maximum number of Newton iterations (default: "+std::to_string(nb_iterations)+")."
        )->check(CLI::Range(0, 1000000));

    app.add_option(
        "-s,--samples",
        nb_samples,
        "Number of points to sample the mesh with when no input point file is given (default: "+std::to_string(nb_samples)+")."
        )->check(CLI::Range(1, 100000000));

    app.add_option(
        "--predicates",
        predicates,
        "Geogram's geometric predicate mode for the restricted Voronoi diagram; "
        "\"exact\" is slower but avoids misclassifications on degenerate configurations (default: "+predicates+")."
        )->check(CLI::IsMember({"fast", "exact"}));

    app.add_option(
        "--o-pts,--out-pts",
        out_pts_filepath,
        "Output the optimized points to this file, in the reference implementation's "
        "\".pts\" format (default: <mesh>.res_sample.pts)."
        );

    app.add_option(
        "--o-rvd,--out-rvd",
        out_rvd_filepath,
        "Output the restricted Voronoi diagram of the optimized point set to this file."
        );

    app.add_option(
        "--o-rdt,--out-rdt",
        out_rdt_filepath,
        "Output the restricted Delaunay triangulation of the optimized point set to this file."
        );

    CLI11_PARSE(app, argc, argv);

    /*== Parse ===================================================================================================== */
    GEO::CmdLine::set_arg("algo:predicates", predicates);

    const auto model_name = get_filename(in_mesh_filepath);
    if (!out_pts_filepath.empty()) {
        if (std::filesystem::is_directory(out_pts_filepath))
            out_pts_filepath = out_pts_filepath + get_filename(in_mesh_filepath) + ".res_sample.pts";
    }
    if (!out_rvd_filepath.empty()) {
        if (std::filesystem::is_directory(out_rvd_filepath))
            out_rvd_filepath = out_rvd_filepath + model_name + ".res_rvd.geogram";
    }
    if (!out_rdt_filepath.empty()) {
        if (std::filesystem::is_directory(out_rdt_filepath))
            out_rdt_filepath = out_rdt_filepath + model_name + ".res_rdt.obj";
    }

    LOG::DEBUG("in_mesh_filepath: {}", in_mesh_filepath);
    LOG::DEBUG("in_pts_filepath: {}", in_pts_filepath);
    LOG::DEBUG("exponent: {}", exponent);
    LOG::DEBUG("volumetric: {}", volumetric);
    LOG::DEBUG("iterations: {}", nb_iterations);
    LOG::DEBUG("samples: {}", nb_samples);
    LOG::DEBUG("predicates: {}", predicates);
    LOG::DEBUG("out_pts_filepath: {}", out_pts_filepath);
    LOG::DEBUG("out_rvd_filepath: {}", out_rvd_filepath);
    LOG::DEBUG("out_rdt_filepath: {}", out_rdt_filepath);

    /*== Let's go! ================================================================================================= */
    LOG::TRACE("Start {}!", APP_NAME);

    try {
        /* Load mesh */
        GEO::Mesh mesh;
        if (!GEO::mesh_load(in_mesh_filepath, mesh)) {
            LOG::ERROR("Could not load mesh file: {}", in_mesh_filepath);
            return EXIT_FAILURE;
        }

        if (volumetric) {
            if (mesh.cells.nb() == 0) {
                LOG::ERROR("The input mesh has no volume elements!");
                return EXIT_FAILURE;
            }

            if (!mesh.cells.are_simplices()) {
                LOG::ERROR("The input mesh needs to be a simplicial mesh!");
                return EXIT_FAILURE;
            }
        }
        else {
            if (mesh.facets.nb() == 0) {
                LOG::ERROR("The input mesh has no surface elements!");
                return EXIT_FAILURE;
            }

            if (!mesh.facets.are_simplices()) {
                LOG::INFO("The input mesh needs to be a simplicial mesh, so triangulation is performed.");

                // The background mesh has to be triangulated and its adjacency computed before
                // the restricted Voronoi diagram is built, and mesh_repair() renumbers the
                // elements, so it has to happen before the LpCVT is constructed.
                GEO::mesh_repair(mesh, GEO::MESH_REPAIR_TRIANGULATE);
            }
        }

        /* Load sites */
        GEO::Mesh pts;
        if (!in_pts_filepath.empty()) {
            if (!pts.load(in_pts_filepath)) {
                LOG::ERROR("Could not load points file: {}", in_pts_filepath);
                return EXIT_FAILURE;
            }

            if (pts.vertices.dimension() == 2) {
                LOG::ERROR("LpCVT only support 3D sites!");
                return EXIT_FAILURE;
            }
        }

        mesh.show_stats("LpCVT input");

        /* Optimize */
        {
            ReportingLpCVT cvt(&mesh, exponent, volumetric);

            if (pts.vertices.nb() > 0)
                cvt.set_points(pts.vertices.nb(), pts.vertices.point_ptr(0));
            else if (!cvt.compute_initial_sampling(nb_samples, true)) {
                LOG::ERROR("Initial sampling failed.");
                return EXIT_FAILURE;
            }

            cvt.Newton_iterations(nb_iterations);

            pts.vertices.create_vertices(cvt.nb_points()-pts.vertices.nb());
            for (const auto& v : pts.vertices) {
                const double* pp = cvt.embedding(v);
                pts.vertices.point(v) = GEO::vec3(pp[0], pp[1], pp[2]);
            }

            // Report the objective, divided by the constant that the reference
            // implementation omits, so that the printed value is the physical energy.
            double f_first = 0.0, f_last = 0.0;
            unsigned int nb_evals = 0;
            cvt.report(f_first, f_last, nb_evals);
            const double normalization = volumetric
                                             ? lp_volume_energy_normalization(exponent)
                                             : lp_surface_energy_normalization(exponent);

            LOG::INFO("Newton iterations: {} objective evaluations, energy {} -> {}",
                      nb_evals, f_first / normalization, f_last / normalization);

            /* Output */
            if (!out_pts_filepath.empty()) {
                if (!pts.save(out_pts_filepath)) {
                    LOG::ERROR("Could not save the points file: {}", out_pts_filepath);
                    return EXIT_FAILURE;
                }

                LOG::INFO("Saved final sites: {}", out_pts_filepath);
            }
            if (!out_rvd_filepath.empty()) {
                GEO::Mesh rvd;
                cvt.RVD()->compute_RVD(rvd, 0);

                if (!GEO::mesh_save(rvd, out_rvd_filepath)) {
                    LOG::ERROR("Could not save the RVD to {}", out_rvd_filepath);
                    return EXIT_FAILURE;
                }
                LOG::INFO("Saved RVD: {}", out_rvd_filepath);
            }
            if (!out_rdt_filepath.empty()) {
                GEO::Mesh rdt;

                cvt.set_use_RVC_centroids(false);
                if (volumetric)
                    cvt.compute_volume(&rdt);
                else
                    cvt.compute_surface(&rdt, true);

                if (!GEO::mesh_save(rdt, out_rdt_filepath)) {
                    LOG::ERROR("Could not save the RDT to {}", out_rdt_filepath);
                    return EXIT_FAILURE;
                }
                LOG::INFO("Saved RDT: {}", out_rdt_filepath);
            }
        }
    }
    catch (const std::exception& e) {
        // The LpCVT constructor validates its own preconditions (unsupported norm
        // exponent, mesh missing triangles or tetrahedra) by throwing, so report those
        // as an ordinary command line error rather than letting them escape.
        LOG::ERROR("{}", e.what());
        return EXIT_FAILURE;
    }

    LOG::TRACE("{} done!", APP_NAME);
    return EXIT_SUCCESS;
}
