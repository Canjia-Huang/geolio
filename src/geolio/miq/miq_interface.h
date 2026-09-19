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
    /**
     * @brief Parameters controlling mixed-integer quadrangulation.
     *
     * The structure also stores the optional input and generated cross fields.
     */
    struct MIQParameters {
        /// Optional input cross field, flattened as three components per vector.
        const double* cross = nullptr;
        /// Number of input cross-field vectors stored per mesh facet.
        GEO::index_t vectors_nb_per_facet = 2; // For the input 4-NoSy field, are N vectors (N*3=3N) provided?

        /// Whether to store the cross field generated or used by MIQ.
        bool output_cross = false;
        /// Output cross field, flattened as three components per vector.
        std::vector<double> out_cross;
        /// Number of output cross-field vectors stored per mesh facet.
        const GEO::index_t out_vectors_nb_per_facet = 2;

        /// Global gradient scale controlling the resulting quad resolution.
        double scale = 30.0;
        /// Weight applied during stiffness optimization.
        double stiffness = 5.0;
        /// Whether to round all integer variables directly.
        bool direct_round = false;
        /// Number of stiffness iterations.
        int iter = 5;
        /// Number of local integer-rounding iterations.
        int local_iter = 5;
        /// Whether integer rounding is enabled.
        bool DoRound = true;
        /// Whether singularity-aware rounding is enabled.
        bool SingularityRound = true;
        /// Additional vertex indices to snap to integer coordinates.
        std::vector<int> round_vertices = std::vector<int>();
        /// Vertex pairs defining hard features to snap to integer coordinates.
        std::vector<std::vector<int>> hard_features = std::vector<std::vector<int>>();
    };

    /**
     * @brief Computes a mixed-integer quadrangulation parameterization.
     *
     * Computes per-facet UV coordinates for a triangular mesh using MIQ. If no
     * input cross field is provided, a 4-RoSy field is generated automatically.
     * The output UV coordinates are written to the facet-corner attribute.
     *
     * @tparam DIM Spatial dimension of the input mesh; must be 2 or 3.
     * @param[in] mesh Triangular input mesh to parameterize.
     * @param[out] mesh_fc_uv Two-dimensional UV attribute defined on mesh corners.
     * @param[in,out] params MIQ configuration parameters and optional cross-field
     *                       input/output buffers.
     *
     * @pre `mesh` contains only triangular facets.
     * @pre `mesh_fc_uv` is bound, has dimension 2, and has one entry per facet corner.
     * @pre `params.vectors_nb_per_facet` is at least 2.
     */
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
