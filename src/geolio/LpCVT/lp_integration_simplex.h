//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/3.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_LP_INTEGRATION_SIMPLEX_H
#define GEOLIO_LP_INTEGRATION_SIMPLEX_H

#include "lp_measure.h"
#include "lp_polynomial.h"
#include "lp_voronoi_vertex.h"

#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_geometry.h>
#include <geogram/voronoi/integration_simplex.h>
#include <geogram/voronoi/generic_RVD_vertex.h>

namespace geolio
{
    /**
     * @brief Lp objective evaluated over the integration simplices of a
     *        restricted Voronoi diagram.
     * @details This is the piece that plugs the LpCVT energy into Geogram: it
     *          derives from ``GEO::IntegrationSimplex``, the extension point that
     *          ``GEO::CentroidalVoronoiTesselation`` documents as being "used by
     *          custom codes, e.g. LpCVT". Once such an object is installed in
     *          ``CentroidalVoronoiTesselation::simplex_func_``, the inherited
     *          ``funcgrad()`` routes every integration simplex to eval() and the
     *          whole Geogram machinery (RVD traversal, exact predicates,
     *          multi-threading, HLBFGS) is reused unchanged.
     *
     *          For each integration simplex the objective is the sum, over the
     *          simplices of the restricted Voronoi cells, of
     *          ``|T| * E`` where ``E`` is the coefficient-free Lp polynomial of
     *          @ref LpPolynomial and ``|T|`` is the measure (triangle area for
     *          surface meshing, tetrahedron volume for volume meshing). The
     *          gradient is obtained by two chained applications of the chain
     *          rule:
     *          -# from the four simplex vertices to the Voronoi cell vertices,
     *             handled by @ref LpPolynomial;
     *          -# from a Voronoi cell vertex to the Delaunay vertices it depends
     *             on, handled by @ref LpVoronoiVertex.
     *
     *          This second step is what makes the Lp objective different from the
     *          usual L2 CVT one: for ``p > 2`` the boundary terms of the
     *          integration simplices no longer cancel between neighbouring cells,
     *          so the Voronoi cell vertices have to be differentiated explicitly.
     * @tparam P The integer norm exponent, even and in ``[2, 16]``.
     * @tparam MEASURE Either @ref LpTriArea (surface meshing) or @ref LpTetVolume
     *         (volume meshing).
     */
    template <unsigned int P, class MEASURE>
    class LpIntegrationSimplex : public GEO::IntegrationSimplex {
    public:
        /**
         * @brief Constructs an Lp integrand over a background mesh.
         * @details @p mesh is stored by reference by the base class and must
         *          therefore outlive this object. Note that Geogram's RVD may
         *          @b reorder the background mesh in place (it partitions it with a
         *          Hilbert order when it runs multi-threaded), which is why this
         *          class deliberately caches no mesh-derived data: facet normals
         *          are recomputed on the fly in eval().
         * @param[in] mesh The background mesh: a triangulated surface for surface
         *                 meshing, a tetrahedralized volume for volume meshing.
         * @param[in] volumetric true for volume meshing, false for surface meshing.
         * @param[in] nb_matrices Number of per-element matrices, or 0 for the
         *                 isotropic case (identity matrix), which is what the
         *                 reference LpCVT implementation always used.
         * @param[in] nb_comp_per_matrix Number of doubles per matrix, or 0 when
         *                 @p nb_matrices is 0. A value of 9 denotes a full 3x3
         *                 matrix.
         * @param[in] matrices Pointer to the ``9 * nb_matrices`` matrix
         *                 coefficients, or nullptr when @p nb_matrices is 0. Not
         *                 owned: the array must outlive this object. Matrices are
         *                 stored row-major, i.e. ``matrices[9 * t + 3 * i + j]`` is
         *                 coefficient ``(i, j)`` of the matrix of element @p t,
         *                 matching the layout expected by ``GEO::mat3(const
         *                 double*)``.
         * @note Geogram's base class names these parameters ``nb_frames``,
         *       ``nb_comp_per_frame`` and ``frames`` and documents them in terms of
         *       anisotropy frames. That is narrower than what is actually used here:
         *       the objective accepts any 3x3 matrix, orthonormal or not.
         */
        LpIntegrationSimplex(
            const GEO::Mesh& mesh,
            const bool volumetric,
            const GEO::index_t nb_matrices,
            const GEO::index_t nb_comp_per_matrix,
            const double* matrices
            ) : GEO::IntegrationSimplex(
                    mesh, volumetric, nb_matrices, nb_comp_per_matrix, matrices
                    )
        {}

        /**
         * @brief Evaluates the Lp objective and its gradient on one integration
         *        simplex.
         * @details The integration simplex is ``(seed, v0, v1, v2)``, i.e. the
         *          tetrahedron joining the Voronoi cell seed to a triangle of the
         *          background mesh. In surface mode the three vertices are the
         *          vertices of a triangle of the RVD cell clipped to the current
         *          facet, and in volume mode they are the vertices of a facet of
         *          the clipped polyhedron. This matches ``I[0]`` / ``C[0..2]`` of
         *          the reference implementation, except that Geogram hands the
         *          seed index, the symbolic representation and the current element
         *          over directly, so the precomputed ``I``/``C`` arrays and the
         *          two-phase architecture become unnecessary.
         * @param[in] center_vertex_index Index of the Delaunay vertex (the seed /
         *                 Voronoi cell center) the simplex belongs to.
         * @param[in] v0 First vertex of the integration simplex.
         * @param[in] v1 Second vertex of the integration simplex.
         * @param[in] v2 Third vertex of the integration simplex.
         * @param[in] t Index of the current background element: a facet in surface
         *                 mode, a tetrahedron in volume mode. Also used to look up
         *                 the anisotropy frame.
         * @param[in] t_adj Index of the tetrahedron adjacent across the simplex, or
         *                 ``GEO::NO_INDEX``. Unused here.
         * @param[in] v_adj Index of the Voronoi cell adjacent across the simplex, or
         *                 ``GEO::NO_INDEX``. Unused here.
         * @return The contribution of this simplex to the objective.
         */
        double eval(
            GEO::index_t center_vertex_index,
            const GEOGen::Vertex& v0,
            const GEOGen::Vertex& v1,
            const GEOGen::Vertex& v2,
            GEO::index_t t,
            GEO::index_t t_adj = GEO::NO_INDEX,
            GEO::index_t v_adj = GEO::NO_INDEX
            ) override {
            GEO::geo_argused(t_adj);
            GEO::geo_argused(v_adj);

            const GEO::vec3 p0 = to_vec3(point(center_vertex_index));
            const GEO::vec3 p1 = to_vec3(v0.point());
            const GEO::vec3 p2 = to_vec3(v1.point());
            const GEO::vec3 p3 = to_vec3(v2.point());

            const GEO::mat3 M = element_matrix(t);

            // dFdC[j] is the gradient with respect to the j-th integration simplex
            // vertex other than the seed.
            GEO::vec3 dFdp0, dFdC[3];
            const double f = poly_.eval(
                measure_, p0, p1, p2, p3, M, dFdp0, dFdC[0], dFdC[1], dFdC[2]
                );

            const LpGradientAccumulator acc(g_, points_stride_, spinlocks_);

            // Compose grad for the three integration simplex vertices, i.e. propagate
            // dF/dC down to the Delaunay vertices that C depends on.
            compose_vertex_gradient(v0, center_vertex_index, p0, dFdC[0], acc);
            compose_vertex_gradient(v1, center_vertex_index, p0, dFdC[1], acc);
            compose_vertex_gradient(v2, center_vertex_index, p0, dFdC[2], acc);

            // Compose grad for p0
            acc.add(center_vertex_index, dFdp0);

            return f;
        }

    private:
        /**
         * @brief Converts a raw coordinate pointer to a vector.
         * @param[in] p Pointer to at least three doubles.
         * @return The corresponding vector.
         */
        static GEO::vec3 to_vec3(const double* p) {
            return GEO::vec3(p[0], p[1], p[2]);
        }

        /**
         * @brief Returns the matrix attached to a background element.
         * @details When no matrix array was supplied, or when the element is beyond
         *          the supplied blocks, the identity is returned, which is the
         *          isotropic LpCVT of the paper.
         * @param[in] t Index of the background element, or ``GEO::NO_INDEX``.
         * @return The 3x3 matrix ``M``; the objective is expressed in terms of
         *         ``U_k = M (p_k - p0)``.
         * @note ``nb_frames_`` and ``frame(t)`` are Geogram's names for the
         *       underlying storage; see the constructor's note.
         */
        GEO::mat3 element_matrix(const GEO::index_t t) const {
            GEO::mat3 M;
            if (nb_frames_ == 0 || t >= nb_frames_) {
                return M;
            }
            const double* m = frame(t);
            for (GEO::index_t i = 0; i < 3; ++i) {
                for (GEO::index_t j = 0; j < 3; ++j) {
                    M(i, j) = m[3 * i + j];
                }
            }
            return M;
        }

        /**
         * @brief Returns the three vertices of the triangle supporting a facet
         *        referenced by a symbolic representation.
         * @details The meaning of a boundary facet entry depends on the mode: in
         *          surface mode it is an index into ``mesh.facets``, whereas in
         *          volume mode ``GEOGen::SymbolicVertex`` stores a tetrahedron
         *          half-facet id ``4 * t + lf``. Both are resolved here so that the
         *          callers can stay mode-agnostic.
         * @param[in] facet_id The boundary facet entry of the symbolic
         *                 representation.
         * @param[out] a First vertex of the supporting triangle.
         * @param[out] b Second vertex of the supporting triangle.
         * @param[out] c Third vertex of the supporting triangle.
         * @return true if the facet could be resolved, false if it is a "virtual"
         *         facet, i.e. the encoding of an open boundary edge of the surface
         *         mesh, which has no supporting triangle.
         */
        bool symbolic_facet_triangle(
            const GEO::index_t facet_id,
            GEO::vec3& a,
            GEO::vec3& b,
            GEO::vec3& c
            ) const {
            if (volumetric_) {
                const GEO::index_t t = facet_id / 4;
                const GEO::index_t lf = facet_id % 4;
                if (t >= mesh_.cells.nb()) {
                    return false;
                }
                a = to_vec3(mesh_.vertices.point_ptr(mesh_.cells.tet_vertex(
                    t, GEO::MeshCells::local_tet_facet_vertex_index(lf, 0)
                    )));
                b = to_vec3(mesh_.vertices.point_ptr(mesh_.cells.tet_vertex(
                    t, GEO::MeshCells::local_tet_facet_vertex_index(lf, 1)
                    )));
                c = to_vec3(mesh_.vertices.point_ptr(mesh_.cells.tet_vertex(
                    t, GEO::MeshCells::local_tet_facet_vertex_index(lf, 2)
                    )));
                return true;
            }

            if (facet_id >= mesh_.facets.nb()) {
                // "Virtual" boundary facet: encodes an edge on the border of an
                // open surface mesh, and has no supporting plane of its own.
                return false;
            }

            if (mesh_.facets.nb_vertices(facet_id) < 3) {
                return false;
            }

            a = to_vec3(mesh_.vertices.point_ptr(mesh_.facets.vertex(facet_id, 0)));
            b = to_vec3(mesh_.vertices.point_ptr(mesh_.facets.vertex(facet_id, 1)));
            c = to_vec3(mesh_.vertices.point_ptr(mesh_.facets.vertex(facet_id, 2)));
            return true;
        }

        /**
         * @brief Returns an (unnormalized) normal of the supporting plane of a
         *        boundary facet.
         * @details Only the direction of the normal matters: every configuration of
         *          @ref LpVoronoiVertex uses either ``W0``, or ``W0`` and ``W1``,
         *          and all of those are invariant under a rescaling of any single
         *          plane normal. This is why the area-weighted, non-normalized
         *          normals are used, exactly like the reference implementation
         *          (``Mesh::facet_normal`` there, ``GEO::Geom::mesh_facet_normal``
         *          here).
         * @param[in] facet_id The boundary facet entry of the symbolic
         *                 representation.
         * @param[out] N The facet normal.
         * @return true if the facet is a real facet, false if it is a "virtual"
         *         boundary facet, in which case @p N is left untouched.
         */
        bool symbolic_facet_normal(const GEO::index_t facet_id, GEO::vec3& N) const {
            GEO::vec3 a, b, c;
            if (!symbolic_facet_triangle(facet_id, a, b, c)) {
                return false;
            }
            if (volumetric_) {
                N = GEO::cross(b - a, c - a);
            } else {
                N = GEO::Geom::mesh_facet_normal(mesh_, facet_id);
            }
            return true;
        }

        /**
         * @brief Returns the two extremities of the edge shared by the two
         *        boundary facets of a configuration-(B) vertex.
         * @details Deliberately does @b not use
         *          ``GEOGen::SymbolicVertex::get_boundary_edge()``: that accessor
         *          only returns meaningful data when ``intersect_symbolic()`` took
         *          one of its three edge-copying branches, and otherwise trips a
         *          compile-time-optional assertion (which, in Geogram debug builds,
         *          throws). The edge is therefore re-derived from the two facet
         *          triangles, and from the corner index carried by a "virtual"
         *          boundary facet when the surface mesh is open. This is the same
         *          information the reference implementation gathered through
         *          ``Mesh::find_edge_extremities()``.
         *
         *          Open surface meshes are fully supported. When a cell vertex lies
         *          on an edge on the border of the mesh, that edge is represented by
         *          a "virtual" boundary facet, i.e. a synthetic index
         *          ``facets.nb() + corner`` rather than a facet. Such an index is
         *          numerically larger than every real facet index, so the sorted
         *          entry list puts the real facets first and
         *          ``boundary_facet(nb_boundary_facets() - 1)`` always names a real
         *          facet. Since a vertex has at most two boundary facets, at most one
         *          of them can be virtual, and the other one is therefore usable as
         *          the facet whose normal the orthogonal-plane substitution needs.
         * @param[in] sym Symbolic representation of the Voronoi vertex; it must have
         *                exactly two boundary facets.
         * @param[out] e0 First extremity, as a mesh vertex index.
         * @param[out] e1 Second extremity, as a mesh vertex index.
         * @return true on success, false if the edge could not be determined, which
         *         only happens for a degenerate mesh whose two boundary facets do not
         *         share exactly two vertices; the caller then skips the gradient
         *         contribution of that vertex.
         */
        bool boundary_edge_extremities(
            const GEOGen::SymbolicVertex& sym,
            GEO::index_t& e0,
            GEO::index_t& e1
            ) const {
            GEO::index_t f0 = sym.boundary_facet(0);
            GEO::index_t f1 = sym.boundary_facet(1);

            if (volumetric_) {
                // Both entries are tetrahedron half-facet ids; the two facets of a
                // tetrahedron share exactly two vertices.
                GEO::index_t shared[2];
                GEO::index_t nb_shared = 0;
                for (GEO::index_t i = 0; i < 3; ++i) {
                    const GEO::index_t vi = mesh_.cells.tet_vertex(
                        f0 / 4, GEO::MeshCells::local_tet_facet_vertex_index(f0 % 4, i)
                        );
                    for (GEO::index_t j = 0; j < 3; ++j) {
                        if (vi == mesh_.cells.tet_vertex(
                                f1 / 4,
                                GEO::MeshCells::local_tet_facet_vertex_index(f1 % 4, j)
                                )) {
                            if (nb_shared < 2) {
                                shared[nb_shared] = vi;
                            }
                            ++nb_shared;
                        }
                    }
                }
                if (nb_shared != 2) {
                    return false;
                }
                e0 = shared[0];
                e1 = shared[1];
                return true;
            }

            // Surface mode: the facet indices are sorted in descending order, so
            // boundary_facet(0) is the larger one and is the "virtual" one whenever
            // a virtual facet is present.
            if (f1 >= mesh_.facets.nb()) {
                return false;
            }
            if (f0 >= mesh_.facets.nb()) {
                // boundary_facet(0) is virtual: it encodes the local corner index of
                // the edge inside the real facet boundary_facet(1).
                const GEO::index_t fr = f1;
                const GEO::index_t nv = mesh_.facets.nb_vertices(fr);
                if (nv == 0) {
                    return false;
                }
                const GEO::index_t corner = (f0 - mesh_.facets.nb()) % nv;
                e0 = mesh_.facets.vertex(fr, corner);
                e1 = mesh_.facets.vertex(fr, (corner + 1) % nv);
                return true;
            }

            GEO::index_t shared[2];
            GEO::index_t nb_shared = 0;
            for (GEO::index_t i = 0; i < mesh_.facets.nb_vertices(f0); ++i) {
                const GEO::index_t vi = mesh_.facets.vertex(f0, i);
                for (GEO::index_t j = 0; j < mesh_.facets.nb_vertices(f1); ++j) {
                    if (vi == mesh_.facets.vertex(f1, j)) {
                        if (nb_shared < 2) {
                            shared[nb_shared] = vi;
                        }
                        ++nb_shared;
                    }
                }
            }
            if (nb_shared != 2) {
                return false;
            }
            e0 = shared[0];
            e1 = shared[1];
            return true;
        }

        /**
         * @brief Sets up the correct configuration of a Voronoi vertex and
         *        propagates its gradient.
         * @details Selects configuration (A), (B), (C) or (D) from the number of
         *          bisectors in the symbolic representation, which is the
         *          equivalent of the cascade of tests over ``I[1]``, ``I[2]`` and
         *          ``I[3]`` in the reference implementation. Since Geogram exposes
         *          bisectors and boundary facets through separate accessors, the
         *          cascade reduces to a switch on the bisector count; a valid RVD
         *          vertex always satisfies
         *          ``nb_bisectors() + nb_boundary_facets() == 3``.
         * @param[in] vertex The Voronoi cell vertex, in symbolic and geometric form.
         * @param[in] center_vertex_index Index of the cell seed.
         * @param[in] p0 Position of the cell seed.
         * @param[in] dFdC Gradient of the objective with respect to the vertex
         *                 position.
         * @param[in] acc The accumulator receiving the gradient contributions.
         */
        void compose_vertex_gradient(
            const GEOGen::Vertex& vertex,
            const GEO::index_t center_vertex_index,
            const GEO::vec3& p0,
            const GEO::vec3& dFdC,
            const LpGradientAccumulator& acc
            ) const {
            const GEOGen::SymbolicVertex& sym = vertex.sym();
            const GEO::index_t nb_b = sym.nb_bisectors();
            const GEO::index_t nb_f = sym.nb_boundary_facets();

            // A genuine RVD vertex is the intersection of exactly three planes.
            if (nb_b + nb_f != 3) {
                return;
            }

            const GEO::vec3 C = to_vec3(vertex.point());

            GEO::index_t point_index[4] = {
                center_vertex_index, GEO::NO_INDEX, GEO::NO_INDEX, GEO::NO_INDEX
            };

            LpVoronoiVertex V;

            switch (nb_b) {
            case 3: {
                GEO::vec3 p[3];
                for (GEO::index_t k = 0; k < 3; ++k) {
                    point_index[k + 1] = sym.bisector(k);
                    p[k] = to_vec3(point(point_index[k + 1]));
                }
                V.set_three_bisectors(p0, p[0], p[1], p[2], C);
            }
            break;

            case 2: {
                // Configuration (C): two bisectors and one boundary facet. In surface
                // mode that single facet is always a real facet: every cell vertex is
                // built from a polygon initialized from one background facet and is
                // never merged across facets, so it always carries that facet's real
                // index in its symbolic representation. In volume mode the entry is a
                // tetrahedron half-facet id instead, in its own index space.
                if (!volumetric_) {
                    geo_debug_assert(sym.boundary_facet(0) < mesh_.facets.nb());
                }

                GEO::vec3 N;
                if (!symbolic_facet_normal(sym.boundary_facet(0), N)) {
                    return;
                }
                GEO::vec3 p[2];
                for (GEO::index_t k = 0; k < 2; ++k) {
                    point_index[k + 1] = sym.bisector(k);
                    p[k] = to_vec3(point(point_index[k + 1]));
                }
                V.set_two_bisectors_one_plane(p0, p[0], p[1], N, C);
            }
            break;

            case 1: {
                point_index[1] = sym.bisector(0);

                GEO::index_t e0, e1;
                if (!boundary_edge_extremities(sym, e0, e1)) {
                    return;
                }
                const GEO::vec3 w1 = to_vec3(mesh_.vertices.point_ptr(e0));
                const GEO::vec3 w2 = to_vec3(mesh_.vertices.point_ptr(e1));

                // Intersection between a bisector and an edge of the input mesh.
                // To resist coplanar facets, the second facet is replaced with the
                // plane orthogonal to the first one that passes through the common
                // edge, because two coplanar facets would make the 3x3 system
                // resolved by compute_W() singular.
                // This is not an approximation: the two facet planes and the
                // orthogonal plane all contain the shared edge, so they intersect
                // along the very same line and define the same locus for C as a
                // function of the Delaunay points.
                //
                // In surface mode boundary_facet(1) is the highest boundary facet
                // index and is therefore always a real facet: a "virtual" boundary
                // facet, i.e. the encoding of a border edge of an open surface mesh,
                // uses a synthetic index larger than every real one and would sort
                // first. In volume mode the entries are tetrahedron half-facet ids,
                // in their own index space.
                if (!volumetric_) {
                    geo_debug_assert(sym.boundary_facet(1) < mesh_.facets.nb());
                }

                GEO::vec3 N0;
                if (volumetric_) {
                    if (!symbolic_facet_normal(sym.boundary_facet(1), N0)) {
                        return;
                    }
                } else if (sym.boundary_facet(1) < mesh_.facets.nb()) {
                    N0 = GEO::Geom::mesh_facet_normal(mesh_, sym.boundary_facet(1));
                } else {
                    return;
                }
                const GEO::vec3 N1 = GEO::cross(w2 - w1, N0);

                V.set_one_bisector_two_planes(
                    p0, to_vec3(point(point_index[1])), N0, N1, C
                    );
            }
            break;

            default: {
                // Configuration (A): a vertex of the background mesh. It cannot be
                // moved by any Delaunay point.
                V.set_no_free_parameter(C);
            }
            break;
            }

            V.compose_gradient(dFdC, point_index, acc);
        }

        /** @brief The Lp polynomial, i.e. the value and the vertex gradients. */
        LpPolynomial<P> poly_;
        /** @brief The measure functor, i.e. area or volume. */
        MEASURE measure_;
    };

    /**
     * @brief Creates the Lp integrand matching a runtime norm exponent and mode.
     * @details Runtime counterpart of the ``switch(p)`` dispatch of
     *          ``compute_F_Lp_internal()`` in ``LpCVT/algebra/F_Lp.cpp``, which
     *          instantiated the templated integrand for every supported exponent.
     *          The result is meant to be stored in
     *          ``GEO::CentroidalVoronoiTesselation::simplex_func_``.
     * @param[in] mesh The background mesh. Stored by reference, must outlive the
     *                 returned object.
     * @param[in] p The integer norm exponent. Must be even and in ``[2, 16]``.
     * @param[in] volumetric true for volume meshing, false for surface meshing.
     * @param[in] nb_matrices Number of per-element matrices, or 0 for the isotropic
     *                 case.
     * @param[in] matrices Pointer to the ``9 * nb_matrices`` matrix coefficients, or
     *                 nullptr when @p nb_matrices is 0. Row-major 3x3 blocks.
     * @return The newly created integrand, or a null pointer if @p p is not a
     *         supported exponent.
     */
    GEO::IntegrationSimplex_var create_lp_integration_simplex(
        const GEO::Mesh& mesh,
        unsigned int p,
        bool volumetric,
        GEO::index_t nb_matrices = 0,
        const double* matrices = nullptr
        );
}

#endif //GEOLIO_LP_INTEGRATION_SIMPLEX_H
