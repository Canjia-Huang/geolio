#include "miq/miq_proc.h"
#include "miq/miq.h"
#include "miq/nrosy.h"
#include "utils/log.h"
#include "utils/throw_error.h"
#include "utils/parse_filepath.h"
#include <igl/readOBJ.h>
#include <geogram/basic/command_line_args.h>
#include <geogram/mesh/mesh_geometry.h>
#include <geogram/mesh/mesh_io.h>

namespace {

/**
 * @brief Load a cross field from a text file (three doubles per line for each representative vector).
 * @param[in]  input_file_path  Path to the cross field file.
 * @param[out] crosses          Flat array of cross vectors (6 values per face: v0.xyz, v1.xyz).
 * @return true on success.
 */
bool load_cross_field(
    const std::string& input_file_path,
    std::vector<double>& crosses
) {
    LOG::TRACE("{}({})", __FUNCTION__, input_file_path);

    std::ifstream in(input_file_path);
    if (!in.is_open())
        THROW_RUNTIME_ERROR("Cannot open file: " + input_file_path);

    std::vector<double>().swap(crosses);

    double x, y, z;
    while (in >> x >> y >> z) {
        crosses.push_back(x);
        crosses.push_back(y);
        crosses.push_back(z);
    }

    in.close();
    return true;
}

}

namespace NeurFrame::proc_utils {

void run_miq(
    const std::string& input_surface_mesh_path,
    const std::string& input_cross_path,
    const std::string& output_path,
    double scale
) {
    const std::string model_name = get_filename(input_surface_mesh_path);

    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    igl::readOBJ(input_surface_mesh_path, V, F);
    LOG::INFO("#V: {}, #F: {}", V.rows(), F.rows());

    Eigen::MatrixXd PD1(F.rows(), 3);
    Eigen::MatrixXd PD2(F.rows(), 3);
    if (!input_cross_path.empty()) {
        LOG::INFO("load cross field from {}", input_cross_path);
        std::vector<double> cross_field;
        load_cross_field(input_cross_path, cross_field);

        if (cross_field.size() != F.rows() * 6)
            THROW_RUNTIME_ERROR("Invalid cross field!");
        for (int f = 0; f < F.rows(); ++f) {
            PD1(f, 0) = cross_field[6 * f];
            PD1(f, 1) = cross_field[6 * f + 1];
            PD1(f, 2) = cross_field[6 * f + 2];
            PD2(f, 0) = cross_field[6 * f + 3];
            PD2(f, 1) = cross_field[6 * f + 4];
            PD2(f, 2) = cross_field[6 * f + 5];
        }
    }
    else {
        LOG::INFO("generate cross field by libigl");

        Eigen::VectorXi b(1);
        b << 0;
        Eigen::MatrixXd bc(1, 3);
        bc << 1, 0, 0;
        Eigen::MatrixXd R;
        Eigen::VectorXd S;

        igl::copyleft::comiso::nrosy(V, F, b, bc, 4, R, S);
        for (int f = 0; f < F.rows(); ++f) {
            PD1(f, 0) = R(f, 0);
            PD1(f, 1) = R(f, 1);
            PD1(f, 2) = R(f, 2);

            GEO::vec3 d0(PD1(f, 0), PD1(f, 1), PD1(f, 2));
            GEO::vec3 n = GEO::Geom::triangle_normal(
                GEO::vec3(V(F(f, 0), 0), V(F(f, 0), 1), V(F(f, 0), 2)),
                GEO::vec3(V(F(f, 1), 0), V(F(f, 1), 1), V(F(f, 1), 2)),
                GEO::vec3(V(F(f, 2), 0), V(F(f, 2), 1), V(F(f, 2), 2)));
            GEO::vec3 d1 = GEO::normalize(GEO::cross(n, d0));

            PD2(f, 0) = d1.x;
            PD2(f, 1) = d1.y;
            PD2(f, 2) = d1.z;
        }

        const std::string output_libigl_cross_field_path = output_path + model_name + "_miq.fra";
        {
            std::ofstream out(output_libigl_cross_field_path);
            if (!out.is_open())
                THROW_RUNTIME_ERROR("Cannot write to " + output_libigl_cross_field_path);
            for (int f = 0; f < F.rows(); ++f) {
                out << PD1(f, 0) << " " << PD1(f, 1) << " " << PD1(f, 2) << std::endl;
                out << PD2(f, 0) << " " << PD2(f, 1) << " " << PD2(f, 2) << std::endl;
            }
            out.close();
            LOG::INFO("save cross field to {}", output_libigl_cross_field_path);
        }
    }

    const std::string debug_cross_field_path = output_path + model_name + "_cross_field.geogram";
    {
        GEO::Mesh M;
        GEO::mesh_load(input_surface_mesh_path, M);

        constexpr double l = 0.01;
        GEO::index_t new_v = M.vertices.create_vertices(4 * M.facets.nb());
        GEO::index_t new_e = M.edges.create_edges(2 * M.facets.nb());
        for (const auto& f : M.facets) {
            const auto center = (M.facets.point(f, 0) + M.facets.point(f, 1) + M.facets.point(f, 2)) / 3;
            const GEO::vec3 d0(PD1(f, 0), PD1(f, 1), PD1(f, 2));
            const GEO::vec3 d1(PD2(f, 0), PD2(f, 1), PD2(f, 2));
            M.vertices.point(new_v) = center + l * d0;
            M.vertices.point(new_v + 1) = center - l * d0;
            M.vertices.point(new_v + 2) = center + l * d1;
            M.vertices.point(new_v + 3) = center - l * d1;
            M.edges.set_vertex(new_e, 0, new_v);
            M.edges.set_vertex(new_e, 1, new_v + 1);
            M.edges.set_vertex(new_e + 1, 0, new_v + 2);
            M.edges.set_vertex(new_e + 1, 1, new_v + 3);
            new_v += 4;
            new_e += 2;
        }
        GEO::mesh_save(M, debug_cross_field_path);
    }

    LOG::INFO("MIQ");
    Eigen::MatrixXd UV;
    Eigen::MatrixXi FUV;
    igl::copyleft::comiso::miq(V, F, PD1, PD2, UV, FUV, scale);
    LOG::INFO("#UV: {}, #FUV: {}", UV.rows(), FUV.rows());

    const std::string output_param_path = output_path + model_name + "_param.obj";
    {
        std::ofstream out(output_param_path);
        if (!out.is_open())
            THROW_RUNTIME_ERROR("Cannot write to " + output_param_path);
        for (int v = 0; v < V.rows(); ++v)
            out << "v " << V(v, 0) << " " << V(v, 1) << " " << V(v, 2) << std::endl;
        for (int v = 0; v < UV.rows(); ++v)
            out << "vt " << UV(v, 0) << " " << UV(v, 1) << std::endl;
        for (int f = 0; f < F.rows(); ++f)
            out << "f " << F(f, 0) + 1 << "/" << FUV(f, 0) + 1
            << " " << F(f, 1) + 1 << "/" << FUV(f, 1) + 1
            << " " << F(f, 2) + 1 << "/" << FUV(f, 2) + 1
            << std::endl;
        out.close();
        LOG::INFO("save result to {}", output_param_path);
    }

    LOG::INFO("libQEx");
    const std::string output_quad_path = output_path + model_name + "_param_quad.obj";
    const std::string cmd = "/home/huangcanjia/libQEx/build/demo/cmdline_tool/cmdline_tool " + output_param_path + " " + output_quad_path;
    system(cmd.c_str());
}

}
