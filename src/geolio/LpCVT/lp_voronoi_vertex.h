//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/3.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_LP_VORONOI_VERTEX_H
#define GEOLIO_LP_VORONOI_VERTEX_H

#include "lp_math.h"

#include <geogram/basic/thread_sync.h>

namespace geolio
{
    /**
     * @brief Accumulates gradient contributions into the flat gradient array of
     *        an LpCVT objective evaluation.
     * @details The gradient array layout is the one imposed by
     *          ``GEO::IntegrationSimplex::set_points_and_gradient()``: a
     *          contiguous array of ``stride * nb_points`` doubles where point
     *          @p i owns the @p stride consecutive entries starting at
     *          ``i * stride``.
     *
     *          When ``GEO::RestrictedVoronoiDiagram`` traverses the RVD with
     *          several worker threads, several integration simplices that share
     *          the same Delaunay vertex update the same gradient entries
     *          concurrently, so the accumulation has to be protected by the
     *          per-vertex spin locks that the RVD passes down. In single-threaded
     *          mode @p spinlocks is null and no locking is performed. This
     *          mirrors what Geogram's own ``ComputeCVTFuncGrad*`` actions do.
     * @note The lock is acquired and released around a single point update, so
     *       an accumulator never holds more than one lock at a time and cannot
     *       deadlock.
     */
    class LpGradientAccumulator {
    public:
        /**
         * @brief Constructs an accumulator over an existing gradient array.
         * @param[in,out] g The gradient array to accumulate into. Not owned.
         * @param[in] stride Number of doubles between two consecutive points,
         *                   i.e. the dimension of the Delaunay triangulation.
         * @param[in] spinlocks Per-vertex spin locks to use in multi-threaded
         *                      mode, or nullptr in single-threaded mode.
         */
        LpGradientAccumulator(
            double* g,
            const GEO::index_t stride,
            GEO::Process::SpinLockArray* spinlocks
            ) : g_(g), stride_(stride), spinlocks_(spinlocks) {
        }

        /**
         * @brief Adds a contribution to the gradient of one point.
         * @param[in] point_index Index of the Delaunay vertex (or LpCVT seed)
         *                        whose gradient is being accumulated.
         * @param[in] value The contribution to add.
         */
        void add(
            const GEO::index_t point_index,
            const GEO::vec3& value
            ) const {
            if (spinlocks_ != nullptr) {
                spinlocks_->acquire_spinlock(point_index);
            }
            double* gi = g_ + point_index * stride_;
            gi[0] += value.x;
            gi[1] += value.y;
            gi[2] += value.z;
            if (spinlocks_ != nullptr) {
                spinlocks_->release_spinlock(point_index);
            }
        }

    private:
        /** @brief The gradient array being accumulated into (not owned). */
        double* g_;
        /** @brief Number of doubles between two consecutive points. */
        GEO::index_t stride_;
        /** @brief Per-vertex spin locks, or nullptr in single-threaded mode. */
        GEO::Process::SpinLockArray* spinlocks_;
    };

    /**
     * @brief Computes the derivatives of a Voronoi vertex and propagates them
     *        back onto the Delaunay vertices it depends on.
     * @details Direct port of ``ThreePlanesIntersection`` from
     *          ``LpCVT/algebra/three_planes_intersection.h``, see Appendix B.2 in
     *          the paper. A vertex of a restricted Voronoi cell is the
     *          intersection of three planes, each of which is either
     *          - a @b bisector between the cell center ``p0`` and another
     *            Delaunay vertex ``p_k``, whose plane normal is ``p_k - p0``, or
     *          - a @b boundary plane, i.e. the supporting plane of a facet of the
     *            background mesh, which does not depend on any Delaunay vertex.
     *
     *          Four configurations are possible, named (A) to (D) after Figure 4
     *          of the paper:
     *          - (D) three bisectors: an RDT vertex in the interior of a facet.
     *          - (C) two bisectors and one boundary plane: a Voronoi vertex on a
     *            facet of the background mesh.
     *          - (B) one bisector and two boundary planes: the intersection of a
     *            bisector with an edge of the background mesh.
     *          - (A) three boundary planes: a vertex of the background mesh.
     *
     *          The vertex position @p C is always passed in from the combinatorial
     *          phase (the "C given directly" variants of the reference
     *          implementation), because it has already been computed by the RVD
     *          clipping with all of Geogram's exact predicates. What is
     *          @b recomputed here, from the current Delaunay coordinates, are the
     *          three plane normals and therefore the whole Jacobian
     *          ``dC/dp_k``: the geometric cell vertices alone do not determine the
     *          planes, so the Delaunay vertex coordinates are genuinely needed.
     * @note Configuration (A) has no free parameter and therefore contributes
     *       nothing to the gradient: moving a Delaunay point cannot move a vertex
     *       that lies at a vertex of the background mesh.
     */
    class LpVoronoiVertex {
    public:
        /**
         * @brief Sets up configuration (D): three bisectors.
         * @details Corresponds to the yellow vertices of Figure 4 in the paper.
         *          The circumcenter C is given by ``M C = B`` where the columns of
         *          ``M`` are ``p1 - p0``, ``p2 - p0`` and ``p3 - p0``.
         * @param[in] p0 The cell center, i.e. the Delaunay seed.
         * @param[in] p1 First bisector neighbor of the cell center.
         * @param[in] p2 Second bisector neighbor of the cell center.
         * @param[in] p3 Third bisector neighbor of the cell center.
         * @param[in] C The Voronoi vertex, as computed by the RVD.
         */
        void set_three_bisectors(
            const GEO::vec3& p0,
            const GEO::vec3& p1,
            const GEO::vec3& p2,
            const GEO::vec3& p3,
            const GEO::vec3& C
            ) {
            nb_bisectors_ = 3;
            P_[0] = p0;
            P_[1] = p1;
            P_[2] = p2;
            P_[3] = p3;
            compute_W(p1 - p0, p2 - p0, p3 - p0);
            C_ = C;
        }

        /**
         * @brief Sets up configuration (C): two bisectors and one boundary plane.
         * @details Corresponds to the red vertices of Figures 4 and 6 in the paper.
         * @param[in] p0 The cell center, i.e. the Delaunay seed.
         * @param[in] p1 First bisector neighbor of the cell center.
         * @param[in] p2 Second bisector neighbor of the cell center.
         * @param[in] N_plane Normal of the supporting plane of the background
         *                    mesh facet that contains the Voronoi vertex.
         * @param[in] C The Voronoi vertex, as computed by the RVD.
         * @note @p N_plane does not need to be normalized. Only ``W0`` and ``W1``
         *       are used by this configuration, and both are invariant under a
         *       rescaling of the third plane normal. This is what allows the
         *       ported code to keep using the unnormalized, area-weighted facet
         *       normals that ``GEO::Geom::mesh_facet_normal()`` returns, exactly
         *       like the reference implementation.
         */
        void set_two_bisectors_one_plane(
            const GEO::vec3& p0,
            const GEO::vec3& p1,
            const GEO::vec3& p2,
            const GEO::vec3& N_plane,
            const GEO::vec3& C
            ) {
            nb_bisectors_ = 2;
            P_[0] = p0;
            P_[1] = p1;
            P_[2] = p2;
            P_[3] = GEO::vec3(0.0, 0.0, 0.0);
            compute_W(p1 - p0, p2 - p0, N_plane);
            C_ = C;
        }

        /**
         * @brief Sets up configuration (B): one bisector and two boundary planes.
         * @details Corresponds to the green vertices of Figures 4 and 6 in the
         *          paper. The Voronoi vertex lies on the bisector and on an edge of
         *          the background mesh shared by the two boundary facets.
         * @param[in] p0 The cell center, i.e. the Delaunay seed.
         * @param[in] p1 The bisector neighbor of the cell center.
         * @param[in] N_plane0 Normal of the supporting plane of the first boundary
         *                     facet.
         * @param[in] N_plane1 Normal of the supporting plane of the second boundary
         *                     facet. Callers are expected to have replaced it by
         *                     the plane through the shared edge that is orthogonal
         *                     to the first one whenever the two facets are
         *                     coplanar; see LpIntegrationSimplex.
         * @param[in] C The Voronoi vertex, as computed by the RVD.
         * @note Only ``W0`` is used by this configuration, and it is invariant
         *       under independent rescaling of either plane normal, so unnormalized
         *       normals are fine here too.
         */
        void set_one_bisector_two_planes(
            const GEO::vec3& p0,
            const GEO::vec3& p1,
            const GEO::vec3& N_plane0,
            const GEO::vec3& N_plane1,
            const GEO::vec3& C
            ) {
            nb_bisectors_ = 1;
            P_[0] = p0;
            P_[1] = p1;
            P_[2] = GEO::vec3(0.0, 0.0, 0.0);
            P_[3] = GEO::vec3(0.0, 0.0, 0.0);
            compute_W(p1 - p0, N_plane0, N_plane1);
            C_ = C;
        }

        /**
         * @brief Sets up configuration (A): three boundary planes.
         * @details Corresponds to the blue vertices of Figures 4 and 6 in the
         *          paper. Only the vertex position is needed, since nothing can be
         *          propagated to the gradient.
         * @param[in] C The Voronoi vertex, which is a vertex of the background mesh.
         */
        void set_no_free_parameter(const GEO::vec3& C) {
            nb_bisectors_ = 0;
            C_ = C;
        }

        /**
         * @brief Propagates the gradient of the objective with respect to the
         *        vertex position back onto the Delaunay vertices.
         * @details Applies the chain rule ``dF/dp_k = (dC/dp_k)^T dF/dC`` for every
         *          Delaunay vertex the vertex position depends on, and accumulates
         *          the result into @p g. See Appendix B.2 in the paper.
         * @param[in] dFdC Gradient of the objective with respect to the vertex
         *                 position.
         * @param[in] point_index Indices of the Delaunay vertices that define the
         *                 vertex, in the same order as the positions passed to the
         *                 set_xxx() method: ``point_index[0]`` is the cell center
         *                 (the seed) and ``point_index[1..]`` are the bisector
         *                 neighbors. Only the entries actually used by the current
         *                 configuration are read.
         * @param[in] g The accumulator receiving the contributions.
         */
        void compose_gradient(
            const GEO::vec3& dFdC,
            const GEO::index_t* point_index,
            const LpGradientAccumulator& g
            ) const {
            switch (nb_bisectors_) {
            case 3: {
                compose_gradient_3_bisectors(dFdC, point_index, g);
            }
            break;
            case 2: {
                compose_gradient_2_bisectors(dFdC, point_index, g);
            }
            break;
            case 1: {
                compose_gradient_1_bisector(dFdC, point_index, g);
            }
            break;
            default: {
                // Configuration (A): no Delaunay vertex moves this vertex, so
                // there is nothing to propagate.
            }
            break;
            }
        }

    protected:
        /**
         * @brief Computes ``W0``, ``W1`` and ``W2``, the columns of ``M^-1``.
         * @details The circumcenter C is given by ``M C = B`` where the columns of
         *          ``M`` are the three plane normals ``N0``, ``N1`` and ``N2``.
         *          With ``Tinv = 1 / dot(N0, cross(N1, N2))`` the columns of
         *          ``M^-1`` are ``cross(N1, N2) / Tinv^-1`` and its cyclic
         *          permutations, so that ``C = B.x W0 + B.y W1 + B.z W2``.
         * @param[in] N0 Normal of the first plane.
         * @param[in] N1 Normal of the second plane.
         * @param[in] N2 Normal of the third plane.
         */
        void compute_W(
            const GEO::vec3& N0,
            const GEO::vec3& N1,
            const GEO::vec3& N2
            ) {
            W0_ = GEO::cross(N1, N2);
            W1_ = GEO::cross(N2, N0);
            W2_ = GEO::cross(N0, N1);
            const double Tinv = 1.0 / GEO::dot(N0, W0_);
            W0_.x *= Tinv;
            W0_.y *= Tinv;
            W0_.z *= Tinv;
            W1_.x *= Tinv;
            W1_.y *= Tinv;
            W1_.z *= Tinv;
            W2_.x *= Tinv;
            W2_.y *= Tinv;
            W2_.z *= Tinv;
        }

        /**
         * @brief Builds ``J = W_k (x) (p - C)``, i.e. ``dC/dp`` for a point that
         *        enters the plane system as the normal ``N_k``.
         * @details Differentiating ``M C = B`` with respect to such a point ``p``
         *          gives zero on the two rows whose normals do not depend on
         *          ``p`` and ``p - C`` on the row of ``N_k``, hence
         *          ``dC/dp = W_k (p - C)^T``.
         * @param[out] J The Jacobian, overwritten.
         * @param[in] Wk The corresponding column of ``M^-1``.
         * @param[in] p The Delaunay vertex position.
         */
        void set_jacobian(
            GEO::mat3& J,
            const GEO::vec3& Wk,
            const GEO::vec3& p
            ) const {
            const GEO::vec3 d = p - C_;
            J(0, 0) = d.x * Wk.x;
            J(1, 0) = d.x * Wk.y;
            J(2, 0) = d.x * Wk.z;
            J(0, 1) = d.y * Wk.x;
            J(1, 1) = d.y * Wk.y;
            J(2, 1) = d.y * Wk.z;
            J(0, 2) = d.z * Wk.x;
            J(1, 2) = d.z * Wk.y;
            J(2, 2) = d.z * Wk.z;
        }

        /**
         * @brief Builds ``J = W_k (x) (C - p)``, i.e. ``dC/dp`` for the cell center.
         * @details The cell center plays the role of ``-N_k`` in every bisector
         *          normal, so the sign of the difference is flipped with respect to
         *          set_jacobian(). When several bisectors are present, the rows
         *          add up, which is why @p Wk is passed as a sum.
         * @param[out] J The Jacobian, overwritten.
         * @param[in] Wk The sum of the columns of ``M^-1`` that correspond to the
         *               bisectors involving the cell center.
         * @param[in] p0 The cell center position.
         */
        void set_jacobian_seed(
            GEO::mat3& J,
            const GEO::vec3& Wk,
            const GEO::vec3& p0
            ) const {
            const GEO::vec3 d = C_ - p0;
            J(0, 0) = d.x * Wk.x;
            J(1, 0) = d.x * Wk.y;
            J(2, 0) = d.x * Wk.z;
            J(0, 1) = d.y * Wk.x;
            J(1, 1) = d.y * Wk.y;
            J(2, 1) = d.y * Wk.z;
            J(0, 2) = d.z * Wk.x;
            J(1, 2) = d.z * Wk.y;
            J(2, 2) = d.z * Wk.z;
        }

        /**
         * @brief Applies the chain rule for one moving point and accumulates.
         * @details Computes ``(dC/dp)^T dF/dC`` and adds it to the gradient of the
         *          corresponding Delaunay vertex.
         * @param[in] J The Jacobian ``dC/dp``.
         * @param[in] dFdC Gradient with respect to the vertex position.
         * @param[in] point_index Index of the Delaunay vertex being moved.
         * @param[in] g The accumulator receiving the contribution.
         */
        void accumulate(
            const GEO::mat3& J,
            const GEO::vec3& dFdC,
            const GEO::index_t point_index,
            const LpGradientAccumulator& g
            ) const {
            GEO::vec3 dFdCJ;
            lp_matTvecmul(J, dFdC, dFdCJ);
            g.add(point_index, dFdCJ);
        }

        /**
         * @brief Propagates the gradient for configuration (D).
         * @details All three planes are bisectors, so the vertex moves with the
         *          cell center and with all three bisector neighbors.
         *          See Appendix B.2 in the paper.
         * @param[in] dFdC Gradient with respect to the vertex position.
         * @param[in] point_index Delaunay vertex indices (seed first).
         * @param[in] g The accumulator receiving the contributions.
         */
        void compose_gradient_3_bisectors(
            const GEO::vec3& dFdC,
            const GEO::index_t* point_index,
            const LpGradientAccumulator& g
            ) const {
            GEO::mat3 J;

            // ================  dC/dp0

            // The cell center appears in all three bisectors with a flipped sign.
            set_jacobian_seed(J, W0_ + W1_ + W2_, P_[0]);
            accumulate(J, dFdC, point_index[0], g);

            // ================  dC/dp1

            set_jacobian(J, W0_, P_[1]);
            accumulate(J, dFdC, point_index[1], g);

            // ================  dC/dp2

            set_jacobian(J, W1_, P_[2]);
            accumulate(J, dFdC, point_index[2], g);

            // ================  dC/dp3

            set_jacobian(J, W2_, P_[3]);
            accumulate(J, dFdC, point_index[3], g);
        }

        /**
         * @brief Propagates the gradient for configuration (C).
         * @details Two bisectors and one boundary plane, so the gradient reaches
         *          the cell center and the two bisector neighbors, but not the
         *          background mesh. See Appendix B.2 in the paper.
         * @param[in] dFdC Gradient with respect to the vertex position.
         * @param[in] point_index Delaunay vertex indices (seed first).
         * @param[in] g The accumulator receiving the contributions.
         */
        void compose_gradient_2_bisectors(
            const GEO::vec3& dFdC,
            const GEO::index_t* point_index,
            const LpGradientAccumulator& g
            ) const {
            GEO::mat3 J;

            // ================  dC/dp0

            set_jacobian_seed(J, W0_ + W1_, P_[0]);
            accumulate(J, dFdC, point_index[0], g);

            // ================  dC/dp1

            set_jacobian(J, W0_, P_[1]);
            accumulate(J, dFdC, point_index[1], g);

            // ================  dC/dp2

            set_jacobian(J, W1_, P_[2]);
            accumulate(J, dFdC, point_index[2], g);
        }

        /**
         * @brief Propagates the gradient for configuration (B).
         * @details One bisector and two boundary planes, so only the cell center and
         *          the single bisector neighbor matter, and both use ``W0``.
         *          See Appendix B.2 in the paper.
         * @param[in] dFdC Gradient with respect to the vertex position.
         * @param[in] point_index Delaunay vertex indices (seed first).
         * @param[in] g The accumulator receiving the contributions.
         */
        void compose_gradient_1_bisector(
            const GEO::vec3& dFdC,
            const GEO::index_t* point_index,
            const LpGradientAccumulator& g
            ) const {
            GEO::mat3 J;

            // ================  dC/dp0

            set_jacobian_seed(J, W0_, P_[0]);
            accumulate(J, dFdC, point_index[0], g);

            // ================  dC/dp1

            set_jacobian(J, W0_, P_[1]);
            accumulate(J, dFdC, point_index[1], g);
        }

    private:
        /** @brief Number of bisectors among the three defining planes (0 to 3). */
        unsigned int nb_bisectors_ = 0;
        /** @brief Positions of the defining Delaunay vertices, cell center first. */
        GEO::vec3 P_[4];
        /** @brief Columns of the inverse of the plane-normal matrix. */
        GEO::vec3 W0_, W1_, W2_;
        /** @brief Position of the Voronoi vertex. */
        GEO::vec3 C_;
    };
}

#endif //GEOLIO_LP_VORONOI_VERTEX_H
