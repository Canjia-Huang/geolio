//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/3.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_LP_CVT_H
#define GEOLIO_LP_CVT_H

#include <geogram/voronoi/CVT.h>
#include <geogram/mesh/mesh.h>

#include <string>
#include <vector>

namespace geolio
{
    /**
     * @brief Lp Centroidal Voronoi Tesselation.
     * @details Reimplementation of the algorithm of
     *
     *          Bruno Lévy and Yang Liu, "Lp Centroidal Voronoi Tessellation and
     *          its applications", ACM Transactions on Graphics 29(4), 2010,
     *
     *          built on top of ``GEO::CentroidalVoronoiTesselation`` instead of the
     *          original CGAL/Geex codebase. The whole Geogram infrastructure is
     *          reused unchanged: the Delaunay triangulation and its exact
     *          predicates, the restricted Voronoi diagram and its multi-threaded
     *          traversal, the point locking mechanism, and the HLBFGS optimizer
     *          driven by ``Newton_iterations()``.
     *
     *          All that this class adds is the objective function: it installs an
     *          @ref LpIntegrationSimplex into the protected
     *          ``CentroidalVoronoiTesselation::simplex_func_`` slot, which is the
     *          documented extension point for custom integrands ("Integration
     *          simplex used by custom codes, e.g. LpCVT").
     *
     *          The minimized objective is
     *          @f[ F = \sum_{\text{integration simplices}} |T| \; E @f]
     *          with @f$E@f$ the coefficient-free Lp polynomial of @ref LpPolynomial
     *          and @f$|T|@f$ the triangle area (surface meshing) or tetrahedron
     *          volume (volume meshing). Because the boundary terms of the
     *          integration simplices no longer cancel for @f$p > 2@f$, the gradient
     *          also has to be propagated from the Voronoi cell vertices back onto
     *          the Delaunay vertices, which is what @ref LpVoronoiVertex does.
     *
     * @note Deliberately omitted constants. The reference implementation omits the
     *       binomial coefficient and the 1/2 (resp. 1/6) factor of the measure, so
     *       the computed objective differs from the physical energy
     *       @f$\int \|M(x - p_i)\|_p^p\,dx@f$ by a constant factor of
     *       lp_surface_energy_normalization() resp.
     *       lp_volume_energy_normalization(). Those factors do not depend on the
     *       point positions and therefore do not change the minimizer; they are
     *       kept out so that results stay comparable with the original code, and
     *       they are exposed as free functions for callers that want the
     *       physically scaled energy.
     *
     * @warning Side effect on the background mesh. Geogram's
     *          ``RestrictedVoronoiDiagram`` partitions (and therefore reorders in
     *          place, vertices/facets/cells alike) the background mesh the first
     *          time it traverses it with several threads. Since @p mesh is shared
     *          with the caller, its element indices may change as a side effect of
     *          running the optimization. Set anisotropy frames, if any, through
     *          set_frames(), which pins the traversal ranges and disables that
     *          partitioning.
     *
     * @warning Only one ``GEO::CentroidalVoronoiTesselation`` (and therefore one
     *          @ref LpCVT) may exist at a time: the base class constructor claims a
     *          process-wide instance pointer that its static optimizer callbacks
     *          dereference.
     *
     * @pre In surface mode @p mesh must be a @b triangulated surface whose facet
     *      adjacency has been computed (``mesh.facets.are_simplices()`` and
     *      ``mesh.facets.connect()``), since both are required by
     *      ``RestrictedVoronoiDiagram::compute_initial_sampling_on_surface()`` and
     *      by the RVD propagation. In volume mode @p mesh must additionally be
     *      filled with tetrahedra (``mesh.cells``) with computed adjacency, for
     *      instance through ``GEO::mesh_tetrahedralize()``. ``GEO::mesh_repair()``
     *      with ``MESH_REPAIR_DEFAULT`` performs the triangulation and the
     *      connection in one call, but it renumbers the elements, so it must be
     *      called @b before constructing this object.
     */
    class LpCVT : public GEO::CentroidalVoronoiTesselation {
    public:
        /**
         * @brief Constructs an LpCVT over a background mesh.
         * @details Also registers the paper in Geogram's bibliography and sets the
         *          volumetric mode of the underlying restricted Voronoi diagram.
         * @param[in] mesh The background mesh. Not owned, must outlive this object
         *                 and must satisfy the preconditions documented on the
         *                 class.
         * @param[in] p The integer norm exponent. Must be even and in ``[2, 16]``;
         *                2 reproduces the usual (L2) centroidal Voronoi
         *                tessellation, larger values sharpen the tessellation.
         * @param[in] volumetric true to mesh the volume bounded by @p mesh, false to
         *                 mesh its surface.
         * @param[in] delaunay Name of the Delaunay implementation to use. Defaults
         *                 to ``"BDEL"``, Geogram's 3d Delaunay with exact predicates
         *                 and symbolic perturbation, which is the closest analogue of
         *                 the exact CGAL triangulation used by the original code.
         *                 Note that Geogram's own ``"default"`` resolves to
         *                 ``algo:delaunay``, whose default value is ``"NN"``.
         */
        LpCVT(
            GEO::Mesh* mesh,
            unsigned int p,
            bool volumetric = false,
            const std::string& delaunay = "BDEL"
            );

        /**
         * @brief Destroys the LpCVT and releases the process-wide CVT instance.
         */
        ~LpCVT() override;

        /**
         * @brief Returns the current norm exponent.
         * @return The value of @p p.
         */
        unsigned int p() const {
            return p_;
        }

        /**
         * @brief Changes the norm exponent and rebuilds the integrand.
         * @details Must not be called while an optimization is running.
         * @param[in] p The new norm exponent, even and in ``[2, 16]``. Out-of-range
         *              values are rejected with an error message and leave the
         *              object unchanged.
         */
        void set_p(unsigned int p);

        /**
         * @brief Installs one anisotropy frame per background element.
         * @details Frames implement the anisotropic variant of the paper. Element
         *          @p t uses the matrix ``M`` such that the objective is expressed in
         *          terms of ``M (p_k - p0)``; the frame of facet @p f (surface mode)
         *          or tetrahedron @p t (volume mode) is therefore expected to encode
         *          the inverse of the desired metric in the tangent plane, after the
         *          usual normalization.
         *
         *          The storage layout is 9 doubles per element, row-major, i.e.
         *          ``frames[9 * t + 3 * i + j]`` is coefficient ``(i, j)``, which is
         *          the layout ``GEO::mat3(const double*)`` expects. Note that
         *          ``GEO::FrameField::frames()`` stores three basis vectors
         *          consecutively, which is the transposed convention.
         *
         *          @b Side effect: setting frames disables the RVD mesh
         *          partitioning, and therefore its multi-threaded traversal, because
         *          partitioning reorders the background mesh in place and would
         *          otherwise misalign a per-element attribute. This is documented in
         *          @ref LpCVT's class comment; the restriction lasts for the lifetime
         *          of this object.
         * @param[in] frames The frame coefficients, ``9 * nb_frames`` of them.
         * @param[in] nb_frames Number of background elements, or 0 to return to the
         *                 isotropic case.
         */
        void set_frames(const std::vector<double>& frames, GEO::index_t nb_frames);

        /**
         * @brief Removes the anisotropy frames and returns to the isotropic LpCVT.
         * @details Equivalent to ``set_frames({}, 0)``.
         */
        void clear_frames();

        /**
         * @brief Switches between surface and volume meshing and rebuilds the
         *        integrand.
         * @details Hides the non-virtual base class method on purpose, so that the
         *          integrand can never get out of sync with the restricted Voronoi
         *          diagram's mode. Note that switching to volume mode requires the
         *          background mesh to be tetrahedralized.
         * @param[in] x true for volume meshing, false for surface meshing.
         */
        void set_volumetric(bool x);

        /**
         * @brief Minimizes the Lp objective with the Newton (L-BFGS) solver.
         * @details Reinstalls the integrand and delegates to
         *          ``CentroidalVoronoiTesselation::Newton_iterations()``, which runs
         *          Geogram's HLBFGS optimizer with the tolerance parameters it
         *          hard-codes (all three set to 0, i.e. a pure iteration-count
         *          stopping criterion). Fails loudly instead of silently degrading to
         *          L2 Lloyd iterations when Geogram was built without
         *          ``GEOGRAM_WITH_HLBFGS``.
         * @param[in] nb_iter Maximum number of iterations.
         * @param[in] m Number of L-BFGS corrections kept in the Hessian
         *              approximation.
         */
        void Newton_iterations(
            GEO::index_t nb_iter,
            GEO::index_t m = 7
            ) override;

        /**
         * @brief Runs Lloyd iterations and warns that they ignore the Lp objective.
         * @details Lloyd relaxation as implemented by Geogram replaces every point
         *          with the centroid of its restricted Voronoi cell, which is the
         *          stationary condition of the @b L2 energy only. It is therefore
         *          not a valid optimizer for @f$p \neq 2@f$ and is kept accessible
         *          only as a cheap pre-relaxation step, which is how the reference
         *          LpCVT pipeline used it too. A warning is emitted whenever
         *          @f$p \neq 2@f$.
         * @param[in] nb_iter Number of Lloyd iterations.
         */
        void Lloyd_iterations(GEO::index_t nb_iter) override;

    protected:
        /**
         * @brief Evaluates the Lp objective and its gradient.
         * @details Reinstalls the integrand if the base class dropped it, which
         *          happens after every call to ``Newton_iterations()``, then
         *          delegates to the base implementation. This guard guarantees that
         *          the L2 CVT objective can never be silently substituted for the Lp
         *          one.
         * @param[in] n Number of variables, i.e. three times the number of points.
         * @param[in] x Current point coordinates.
         * @param[out] f The objective value.
         * @param[out] g The gradient, of size @p n.
         */
        void funcgrad(
            GEO::index_t n,
            double* x,
            double& f,
            double* g
            ) override;

    private:
        /**
         * @brief Creates (or re-creates) the integrand and installs it.
         * @details Keeps a reference to the created object so that it can be
         *          reinstalled without reallocating, and reports an error if the
         *          parameters are unsupported.
         */
        void rebuild_integrand();

        /** @brief The norm exponent. */
        unsigned int p_;
        /** @brief Whether the volume bounded by the mesh is being meshed. */
        bool volumetric_;
        /** @brief Anisotropy frames, 9 doubles per background element. */
        std::vector<double> frames_;
        /** @brief Number of anisotropy frames, or 0 when isotropic. */
        GEO::index_t nb_frames_;
        /**
         * @brief Keeps the integrand alive.
         * @details ``simplex_func_`` already owns a reference, but the base class
         *          resets it at the end of every Newton run, so holding our own
         *          reference lets rebuild_integrand() reinstall it for free.
         */
        GEO::IntegrationSimplex_var integrand_;
    };
}

#endif //GEOLIO_LP_CVT_H
