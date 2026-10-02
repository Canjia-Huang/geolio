//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/3.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "lp_cvt.h"

#include "lp_integration_simplex.h"
#include "lp_measure.h"

#include <geolio/common/log.h>

#include <geogram/bibliography/bibliography.h>
#include <geogram/numerics/optimizer.h>

namespace geolio
{
    LpCentroidalVoronoiTesselation::LpCentroidalVoronoiTesselation(
        GEO::Mesh* mesh,
        const GEO::index_t p,
        const bool volumetric,
        const std::string& delaunay
        ) : GEO::CentroidalVoronoiTesselation(mesh, 3, delaunay),
            p_(p),
            volumetric_(volumetric),
            nb_matrices_(0) {
        geo_cite("DBLP:journals/tog/LevyL10");

        if (!is_supported_norm_exponent(p_))
            throw std::logic_error("Unsupported Lp norm exponent p = "+std::to_string(p)+" (expected an even integer in [2, 16])");

        // Meshing a volume needs the background mesh to be filled with tetrahedra,
        // and meshing a surface needs the facets to be triangles with a valid
        // adjacency, because Geogram neither triangulates nor repairs the input mesh
        // for us (only the RDT it produces on output is repaired).
        if (volumetric_) {
            if (mesh->cells.nb() == 0) {
                throw std::logic_error("Volume meshing requires a tetrahedralized background mesh, "
                                       "but mesh.cells is empty; call GEO::mesh_tetrahedralize() "
                                       "before constructing the LpCVT");
            }
        } else {
            if (!mesh->facets.are_simplices()) {
                throw std::logic_error("Surface meshing requires a triangulated background mesh; call "
                                       "GEO::mesh_repair() with MESH_REPAIR_DEFAULT or "
                                       "mesh.facets.triangulate() before constructing the LpCVT");
            }
        }

        GEO::CentroidalVoronoiTesselation::set_volumetric(volumetric_);
        rebuild_integrand();
    }

    LpCentroidalVoronoiTesselation::~LpCentroidalVoronoiTesselation() = default;

    void LpCentroidalVoronoiTesselation::set_p(const GEO::index_t p) {
        if (!is_supported_norm_exponent(p)) {
            LOG::ERROR("Unsupported Lp norm exponent p = {} (expected an even integer in "
                       "[2, 16]); keeping p = {}", p, p_);
            return;
        }
        p_ = p;
        rebuild_integrand();
    }

    void LpCentroidalVoronoiTesselation::set_matrices(const std::vector<double>& matrices) {
        if (matrices.empty()) {
            clear_matrices();
            return;
        }
        if (matrices.size() % 9 != 0) {
            LOG::ERROR("Per-element matrices must be given as blocks of 9 doubles "
                       "(row-major 3x3), but {} doubles were supplied", matrices.size());
            return;
        }

        matrices_ = matrices;
        // The number of covered elements follows from the array size, so the two can
        // never disagree. Elements beyond the supplied blocks keep the identity.
        nb_matrices_ = GEO::index_t(matrices_.size() / 9);

        // Geogram's restricted Voronoi diagram partitions the background mesh with a
        // Hilbert order on its first multi-threaded traversal, and partitioning
        // reorders the mesh in place (vertices, facets and cells alike). A
        // per-element attribute would then no longer match the element it was built
        // for, so partitioning is disabled by pinning the traversal ranges to the
        // whole mesh. Pinning both ranges is required because the early-out in the
        // RVD only inspects the facet range.
        RVD_->set_facets_range(0, mesh_->facets.nb());
        if (mesh_->cells.nb() != 0) {
            RVD_->set_tetrahedra_range(0, mesh_->cells.nb());
        }

        rebuild_integrand();
    }

    void LpCentroidalVoronoiTesselation::clear_matrices() {
        matrices_.clear();
        nb_matrices_ = 0;
        rebuild_integrand();
    }

    void LpCentroidalVoronoiTesselation::set_volumetric(const bool x) {
        volumetric_ = x;
        GEO::CentroidalVoronoiTesselation::set_volumetric(x);
        rebuild_integrand();
    }

    void LpCentroidalVoronoiTesselation::Newton_iterations(
        const GEO::index_t nb_iter,
        const GEO::index_t m
        ) {
        // The base class falls back to Lloyd iterations, with a mere warning, when
        // the optimizer could not be created. That path never reaches the integrand,
        // so the Lp objective would be silently ignored; refuse to proceed instead.
        {
            const GEO::Optimizer_var probe = GEO::Optimizer::create("HLBFGS");
            if (probe.is_null()) {
                LOG::ERROR("This Geogram build has no HLBFGS optimizer, so the Lp objective "
                           "cannot be minimized. Rebuild Geogram with "
                           "GEOGRAM_WITH_HLBFGS=ON");
                return;
            }
        }

        // Newton_iterations() resets simplex_func_ once optimization is over, so the
        // integrand has to be installed again before every run.
        rebuild_integrand();
        GEO::CentroidalVoronoiTesselation::Newton_iterations(nb_iter, m);
    }

    void LpCentroidalVoronoiTesselation::Lloyd_iterations(const GEO::index_t nb_iter) {
        if (p_ != 2) {
            LOG::WARN("Lloyd_iterations() replaces each point with the centroid of its "
                      "restricted Voronoi cell, which is the stationary condition of the "
                      "L2 energy only; it therefore ignores the Lp objective (p = {}). Use "
                      "Newton_iterations() for LpCVT, and keep Lloyd iterations only as a "
                      "cheap L2 pre-relaxation", p_);
        }
        GEO::CentroidalVoronoiTesselation::Lloyd_iterations(nb_iter);
    }

    void LpCentroidalVoronoiTesselation::funcgrad(
        const GEO::index_t n,
        double* x,
        double& f,
        double* g
        ) {
        // Defensive: the base class drops the integrand at the end of every Newton
        // run, and evaluate without it would silently fall back to the L2 CVT
        // objective instead of failing.
        if (simplex_func_.is_null()) {
            rebuild_integrand();
        }
        GEO::CentroidalVoronoiTesselation::funcgrad(n, x, f, g);
    }

    void LpCentroidalVoronoiTesselation::rebuild_integrand() {
        integrand_ = create_lp_integration_simplex(
            *mesh_,
            p_,
            volumetric_,
            nb_matrices_,
            matrices_.empty() ? nullptr : matrices_.data()
            );

        if (integrand_.is_null()) {
            LOG::ERROR("Could not build the Lp integrand (p = {}, volumetric = {})",
                       p_, volumetric_);
            return;
        }

        simplex_func_ = integrand_;
    }
}
