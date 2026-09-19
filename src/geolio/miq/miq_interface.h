//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/19.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_MIQ_INTERFACE_H
#define GEOLIO_MIQ_INTERFACE_H
#include "miq.h"
#include <geogram/mesh/mesh.h>
#include "nrosy.h"

namespace geolio
{
    struct MIQParameters {
        const double* cross = nullptr;
        GEO::index_t vectors_nb_per_facet = 2; // For the input 4-NoSy field, are N vectors (N*3=3N) provided?

        bool output_cross = false;
        std::vector<double> out_cross;
        const GEO::index_t out_vectors_nb_per_facet = 2;

        double scale = 30.0;
        double stiffness = 5.0;
        bool direct_round = false;
        int iter = 5;
        int local_iter = 5;
        bool DoRound = true;
        bool SingularityRound = true;
        std::vector<int> round_vertices = std::vector<int>();
        std::vector<std::vector<int>> hard_features = std::vector<std::vector<int>>();
    };

    template <GEO::index_t DIM>
    void miq(
        const GEO::Mesh& mesh,
        GEO::Attribute<double>& mesh_fc_uv,
        MIQParameters& params
        ) requires (DIM == 2 || DIM == 3) {
        assert(mesh.facets.are_simplices());
        assert(mesh_fc_uv.is_bound());
        assert(mesh_fc_uv.dimension() == 2);
        assert(mesh_fc_uv.size() == mesh.facet_corners.nb());
        assert(params.vectors_nb_per_facet >= 2);

        /* Load mesh */
        Eigen::MatrixXd V(mesh.vertices.nb(), 3);
        Eigen::MatrixXi F(mesh.facets.nb(), 3);
        for (const auto& v : mesh.vertices) {
            const auto& p = mesh.vertices.point<DIM>(v);
            V(v, 0) = p[0];
            V(v, 1) = p[1];
            if constexpr (DIM == 2)
                V(v, 2) = 0;
            else
                V(v, 2) = p[2];
        }
        for (const auto& f : mesh.facets) {
            F(f, 0) = mesh.facets.vertex(f, 0);
            F(f, 1) = mesh.facets.vertex(f, 1);
            F(f, 2) = mesh.facets.vertex(f, 2);
        }

        /* Load cross field */
        Eigen::MatrixXd PD1(mesh.facets.nb(), 3);
        Eigen::MatrixXd PD2(mesh.facets.nb(), 3);
        if (params.cross == nullptr) {
            Eigen::VectorXi b(1);
            b << 0;
            Eigen::MatrixXd bc(1, 3);
            bc << 1, 0, 0;
            Eigen::MatrixXd R;
            Eigen::VectorXd S;

            igl::copyleft::comiso::nrosy(V, F, b, bc, 4, R, S);
            for (const auto& f : mesh.facets) {
                PD1(f, 0) = R(f, 0);
                PD1(f, 1) = R(f, 1);
                PD1(f, 2) = R(f, 2);

                const GEO::vec3 d0(PD1(f, 0), PD1(f, 1), PD1(f, 2));
                const GEO::vec3 n = GEO::Geom::triangle_normal(mesh.facets.point(f, 0), mesh.facets.point(f, 1), mesh.facets.point(f, 2));
                const GEO::vec3 d1 = GEO::normalize(GEO::cross(n, d0));

                PD2(f, 0) = d1.x;
                PD2(f, 1) = d1.y;
                PD2(f, 2) = d1.z;
            }
        }
        else {
            for (const auto& f : mesh.facets) {
                PD1(f, 0) = params.cross[3*params.vectors_nb_per_facet*f];
                PD1(f, 1) = params.cross[3*params.vectors_nb_per_facet*f+1];
                PD1(f, 2) = params.cross[3*params.vectors_nb_per_facet*f+2];
                PD2(f, 0) = params.cross[3*params.vectors_nb_per_facet*f+3];
                PD2(f, 1) = params.cross[3*params.vectors_nb_per_facet*f+4];
                PD2(f, 2) = params.cross[3*params.vectors_nb_per_facet*f+5];
            }
        }
        if (params.output_cross) {
            params.out_cross.clear();
            params.out_cross.reserve(3*params.out_vectors_nb_per_facet * mesh.facets.nb());
            for (const auto& f : mesh.facets) {
                params.out_cross.push_back(PD1(f, 0));
                params.out_cross.push_back(PD1(f, 1));
                params.out_cross.push_back(PD1(f, 2));
                params.out_cross.push_back(PD2(f, 0));
                params.out_cross.push_back(PD2(f, 1));
                params.out_cross.push_back(PD2(f, 2));
            }
        }

        /* MIQ */
        Eigen::MatrixXd UV;
        Eigen::MatrixXi FUV;
        igl::copyleft::comiso::miq(
            V, F,
            PD1, PD2,
            UV, FUV,
            params.scale,
            params.stiffness,
            params.direct_round,
            params.iter,
            params.local_iter,
            params.DoRound,
            params.SingularityRound,
            params.round_vertices,
            params.hard_features);

        /* Output */
        for (const auto& f : mesh.facets) {
            for (GEO::index_t lv = 0; lv < 3; ++lv) {
                const auto& fc = mesh.facets.corner(f, lv);
                mesh_fc_uv[2*fc] = UV(FUV(f, lv), 0);
                mesh_fc_uv[2*fc+1] = UV(FUV(f, lv), 1);
            }
        }
    }
}

#endif //GEOLIO_MIQ_INTERFACE_H
