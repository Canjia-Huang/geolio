//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/3.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
/**
 * @file main.cpp
 * @brief Command line front end for the LpCVT reimplementation.
 * @details Mirrors the interface of the reference implementation
 *          (``LpCVT [OPTIONS] meshPath [ptsPath]``) so that the two can be driven
 *          with the same arguments and exchange point files in both directions.
 */
#include <geolio/LpCVT/lp_cvt.h>
#include <geolio/LpCVT/lp_measure.h>

#include <geogram/basic/command_line.h>
#include <geogram/basic/command_line_args.h>
#include <geogram/basic/common.h>
#include <geogram/basic/file_system.h>
#include <geogram/basic/logger.h>
#include <geogram/mesh/mesh_io.h>
#include <geogram/mesh/mesh_repair.h>

#include <fstream>
#include <string>
#include <vector>

namespace
{
    /**
     * @brief Loads a point set in the ".pts" format of the reference LpCVT.
     * @details One point per line, written as ``v x y z``. This is exactly the
     *          format the reference implementation reads and writes, so point sets
     *          can be exchanged between the two programs.
     * @param[in] filename Path to the point file.
     * @param[out] points The loaded coordinates, three doubles per point.
     * @return true on success.
     */
    bool load_points(const std::string& filename, std::vector<double>& points) {
        std::ifstream in(filename);
        if (!in) {
            GEO::Logger::err("LpCVT") << "Could not open " << filename << std::endl;
            return false;
        }

        points.clear();
        std::string keyword;
        while (in >> keyword) {
            if (keyword != "v") {
                // Skip the rest of the line of any other directive.
                std::string rest;
                std::getline(in, rest);
                continue;
            }
            double x = 0.0, y = 0.0, z = 0.0;
            in >> x >> y >> z;
            points.push_back(x);
            points.push_back(y);
            points.push_back(z);
        }

        GEO::Logger::out("LpCVT") << "Loaded " << points.size() / 3
            << " points from " << filename << std::endl;
        return !points.empty();
    }

    /**
     * @brief Saves a point set in the ".pts" format of the reference LpCVT.
     * @param[in] filename Path to the point file to write.
     * @param[in] points The coordinates, three doubles per point.
     * @return true on success.
     */
    bool save_points(const std::string& filename, const std::vector<double>& points) {
        std::ofstream out(filename);
        if (!out) {
            GEO::Logger::err("LpCVT") << "Could not write " << filename << std::endl;
            return false;
        }
        out.precision(17);
        for (std::size_t i = 0; i + 2 < points.size(); i += 3) {
            out << "v " << points[i] << ' ' << points[i + 1] << ' ' << points[i + 2] << '\n';
        }
        GEO::Logger::out("LpCVT") << "Wrote " << points.size() / 3
            << " points to " << filename << std::endl;
        return true;
    }

    /**
     * @brief An LpCVT that reports the objective as the optimizer evaluates it.
     * @details The objective evaluation is protected, since client code is meant to
     *          drive it through Newton_iterations(). Deriving from it is however
     *          exactly the intended way of extending the algorithm, and it gives the
     *          command line front end a way to report that the optimization actually
     *          progressed.
     */
    class ReportingLpCVT : public geolio::LpCVT {
    public:
        using LpCVT::LpCVT;

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
         * @copydetails geolio::LpCVT::funcgrad
         */
        void funcgrad(
            const GEO::index_t n,
            double* x,
            double& f,
            double* g
            ) override {
            LpCVT::funcgrad(n, x, f, g);
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
    GEO::initialize(GEO::GEOGRAM_INSTALL_ALL);
    GEO::CmdLine::import_arg_group("standard");
    GEO::CmdLine::import_arg_group("algo");

    GEO::CmdLine::declare_arg("p", 2, "Lp norm exponent, an even integer in [2, 16]");
    GEO::CmdLine::declare_arg("volumetric", false, "Mesh the volume instead of the surface");
    GEO::CmdLine::declare_arg("iter", 100, "Maximum number of Newton iterations");
    GEO::CmdLine::declare_arg("samples", 10000, "Number of points of the initial sampling");
    GEO::CmdLine::declare_arg(
        "save_pts", std::string(""),
        "Save the optimized points to this file (default: <meshfile>.res_sample.pts)"
        );
    GEO::CmdLine::declare_arg(
        "save_rdt", std::string(""),
        "Save the restricted Delaunay triangulation to this file"
        );

    // The positional arguments have to be declared here: CmdLine::parse() rejects
    // any unparsed argument unless the specification allows for it.
    std::vector<std::string> filenames;
    if (!GEO::CmdLine::parse(argc, argv, filenames, "<meshfile> <ptsfile>*")) {
        return 1;
    }
    if (filenames.empty()) {
        GEO::Logger::err("LpCVT") << "Usage: lpcvt [options] meshfile [ptsfile]" << std::endl;
        return 1;
    }

    const unsigned int p = GEO::CmdLine::get_arg_uint("p");
    const bool volumetric = GEO::CmdLine::get_arg_bool("volumetric");
    const unsigned int nb_iter = GEO::CmdLine::get_arg_uint("iter");
    const unsigned int nb_samples = GEO::CmdLine::get_arg_uint("samples");

    if (p < 2 || p > 16 || (p / 2) * 2 != p) {
        GEO::Logger::err("LpCVT") << "p must be an even integer in [2, 16]" << std::endl;
        return 1;
    }

    // The background mesh has to be triangulated and its adjacency computed before
    // the restricted Voronoi diagram is built, and mesh_repair() renumbers the
    // elements, so it has to happen before the LpCVT is constructed.
    GEO::Mesh mesh;
    if (!GEO::mesh_load(filenames[0], mesh)) {
        return 1;
    }
    GEO::mesh_repair(mesh, GEO::MESH_REPAIR_DEFAULT);
    mesh.facets.connect();
    if (mesh.cells.nb() != 0) {
        mesh.cells.connect();
    }
    mesh.show_stats("LpCVT input");

    if (volumetric && mesh.cells.nb() == 0) {
        GEO::Logger::err("LpCVT") << "Volumetric mode requires a tetrahedralized mesh; "
            "this build does not tetrahedralize surfaces on the fly" << std::endl;
        return 1;
    }

    std::vector<double> points;
    std::vector<double> final_points;
    int status = 0;

    {
        ReportingLpCVT cvt(&mesh, p, volumetric);

        if (filenames.size() > 1) {
            if (!load_points(filenames[1], points)) {
                return 1;
            }
            cvt.set_points(GEO::index_t(points.size() / 3), points.data());
        } else if (!cvt.compute_initial_sampling(nb_samples, true)) {
            GEO::Logger::err("LpCVT") << "Initial sampling failed" << std::endl;
            return 1;
        }

        cvt.Newton_iterations(nb_iter, 7);

        final_points.resize(cvt.nb_points() * 3);
        for (GEO::index_t i = 0; i < cvt.nb_points(); ++i) {
            const double* pi = cvt.embedding(i);
            for (int c = 0; c < 3; ++c) {
                final_points[3 * i + c] = pi[c];
            }
        }

        // Report the objective, divided by the constant that the reference
        // implementation omits, so that the printed value is the physical energy.
        double f_first = 0.0, f_last = 0.0;
        unsigned int nb_evals = 0;
        cvt.report(f_first, f_last, nb_evals);
        const double normalization = volumetric
                                        ? geolio::lp_volume_energy_normalization(p)
                                        : geolio::lp_surface_energy_normalization(p);

        GEO::Logger::out("LpCVT") << "Newton iterations: " << nb_evals
            << " objective evaluations, energy " << (f_first / normalization)
            << " -> " << (f_last / normalization) << std::endl;

        // Note: the points are read out here, while the LpCVT is still alive, because
        // R3_embedding() is only valid for the current instance.
        const std::string rdt_file = GEO::CmdLine::get_arg("save_rdt");
        if (!rdt_file.empty()) {
            GEO::Mesh rdt;
            // Use the seeds themselves as vertices, i.e. the restricted Delaunay
            // triangulation of the optimized point set, rather than the centroids of
            // the restricted Voronoi cells.
            cvt.set_use_RVC_centroids(false);
            if (volumetric) {
                cvt.compute_volume(&rdt);
            } else {
                cvt.compute_surface(&rdt, true);
            }
            if (!GEO::mesh_save(rdt, rdt_file)) {
                status = 1;
            }
        }
    }

    std::string pts_file = GEO::CmdLine::get_arg("save_pts");
    if (pts_file.empty()) {
        pts_file = filenames[0] + ".res_sample.pts";
    }
    if (!save_points(pts_file, final_points)) {
        status = 1;
    }

    return status;
}
