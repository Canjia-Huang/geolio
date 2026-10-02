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
     *          running the optimization. Set per-element matrices, if any, through
     *          set_matrices(), which pins the traversal ranges and disables that
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
     *      by the RVD propagation. The surface may be @b open: Geogram's restricted
     *      Voronoi diagram supports borders, which it encodes as "virtual" boundary
     *      facets standing for the border edges, and the Lp gradient resolves those
     *      by recovering the border edge from the facet that owns it. In volume mode
     *      @p mesh must additionally be filled with tetrahedra (``mesh.cells``) with
     *      computed adjacency, for instance through ``GEO::mesh_tetrahedralize()``.
     *      ``GEO::mesh_repair()`` with ``MESH_REPAIR_DEFAULT`` performs the
     *      triangulation and the connection in one call, but it renumbers the
     *      elements, so it must be called @b before constructing this object.
     */
    class LpCentroidalVoronoiTesselation : public GEO::CentroidalVoronoiTesselation {
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
        LpCentroidalVoronoiTesselation(
            GEO::Mesh* mesh,
            GEO::index_t p,
            bool volumetric = false,
            const std::string& delaunay = "BDEL");

        /**
         * @brief Destroys the LpCVT and releases the process-wide CVT instance.
         */
        ~LpCentroidalVoronoiTesselation() override;

        /**
         * @brief Tests whether a norm exponent is supported.
         * @details The reference implementation supports the even exponents from 2
         *          to 16; its dispatch switch has no case for anything else, which
         *          silently yields a null objective.
         * @param[in] p The norm exponent to test.
         * @return true if @p p is even and in ``[2, 16]``.
         */
        static bool is_supported_norm_exponent(const GEO::index_t p) {
            return p >= 2 && p <= 16 && (p / 2) * 2 == p;
        }

        /**
         * @brief Returns the current norm exponent.
         * @return The value of @p p.
         */
        [[nodiscard]] GEO::index_t p() const { return p_; }

        /**
         * @brief Changes the norm exponent and rebuilds the integrand.
         * @details Must not be called while an optimization is running.
         * @param[in] p The new norm exponent, even and in ``[2, 16]``. Out-of-range
         *              values are rejected with an error message and leave the
         *              object unchanged.
         */
        void set_p(GEO::index_t p);

        /**
         * @brief Installs one matrix per background element.
         * @details Implements the anisotropic variant of the paper by attaching a
         *          matrix to every element of the background mesh. For an
         *          integration simplex belonging to element @p t, the objective is
         *          expressed in terms of ``M (p_k - p0)`` where ``M`` is the matrix
         *          of element @p t; the gradient is then projected back with ``M^T``,
         *          which is the chain rule for ``grad(F(MX))``.
         *
         *          @b Any 3x3 matrix is accepted: ``M`` need not be orthonormal, nor
         *          even symmetric. This is strictly more general than a rotation
         *          frame, and covers a metric tensor, a shear, a non-uniform scale,
         *          or the inverse of an anisotropy field. Note that Geogram's
         *          underlying storage calls these objects "frames" and documents
         *          ``nb_comp_per_frame`` as "3 for 3-axis anisotropy, 1 for vector
         *          anisotropy"; that plumbing is unused by Geogram itself and the
         *          layout below is the one this class defines.
         *
         *          An element @p t uses the nine doubles starting at ``9 * t``, stored
         *          row-major, i.e. ``matrices[9 * t + 3 * i + j]`` is coefficient
         *          ``(i, j)``. That is the layout ``GEO::mat3(const double*)``
         *          expects. Beware that ``GEO::FrameField::frames()`` instead stores
         *          three basis vectors consecutively, which is the transposed
         *          convention.
         *
         *          If fewer matrices than background elements are supplied, the
         *          remaining elements keep the identity and the objective stays
         *          isotropic there.
         *
         *          @b Side effect: setting matrices disables the RVD mesh
         *          partitioning, and therefore its multi-threaded traversal, because
         *          partitioning reorders the background mesh in place and would
         *          otherwise misalign a per-element attribute. See the class comment;
         *          the restriction lasts for the lifetime of this object.
         * @param[in] matrices The matrix coefficients, a multiple of nine of them,
         *                 which is ``9 * nb_elements`` to cover every element. An
         *                 empty vector restores the isotropic objective.
         * @note Prefer this over building the array by hand when the matrices are
         *       meant to be uniform: fill ``9 * nb_elements`` identical blocks.
         */
        void set_matrices(const std::vector<double>& matrices);

        /**
         * @brief Removes the per-element matrices and returns to the isotropic LpCVT.
         * @details Equivalent to ``set_matrices({})``.
         */
        void clear_matrices();

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
        void Newton_iterations(GEO::index_t nb_iter, GEO::index_t m = 7) override;

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
        void funcgrad(GEO::index_t n, double* x, double& f, double* g) override;

    private:
        /**
         * @brief Creates (or re-creates) the integrand and installs it.
         * @details Keeps a reference to the created object so that it can be
         *          reinstalled without reallocating, and reports an error if the
         *          parameters are unsupported.
         */
        void rebuild_integrand();

        /** @brief The norm exponent. */
        GEO::index_t p_;
        /** @brief Whether the volume bounded by the mesh is being meshed. */
        bool volumetric_;
        /** @brief Per-element matrices, nine doubles per element, row-major. */
        std::vector<double> matrices_;
        /** @brief Number of elements covered by @ref matrices_, 0 when isotropic. */
        GEO::index_t nb_matrices_;
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
