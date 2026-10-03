//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/3.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "frame_field.h"
#include <geogram/basic/assert.h>
#include <geogram/basic/geometry.h>
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_geometry.h>
#include <geogram/numerics/matrix_util.h>
#include <geolio/common/log.h>
#include <algorithm>
#include <cmath>

/****************************************************************************/
/*            Ported from Geogram's mesh/mesh_frame_field.cpp               */
/****************************************************************************/

// Geogram keeps its curvature machinery in an anonymous namespace of its own translation
// unit, next to its FrameField implementation, so a derived class cannot reach any of it:
// MeshFacetBasis (a local orthonormal basis of a facet), NormalCycle (the curvature tensor
// estimator) and the estimation of the two principal curvature directions with their
// magnitudes. The three are reproduced below, algorithm unchanged. Compared with Geogram's
// copy, the members nothing here uses have been dropped (Geogram keeps them and silences
// -Wunused-member-function with a pragma), and the documentation follows this repository's
// style.

namespace {
    using namespace GEO;

    /**
     * @brief Symmetry of a principal curvature direction field.
     * @details A cross field has 4-fold symmetry (``symd = 4.0`` in Geogram's PGP code);
     *          principal curvature directions have 2-fold symmetry instead, a direction
     *          and its opposite describing the same curvature line.
     */
    constexpr double curvature_symd = 2.0;

    /**
     * @brief Represents a local orthonormal basis of a mesh facet.
     * @details Both in-plane vectors are normalized, which is precisely why a frame
     *          built with this basis comes out free of any length information.
     */
    class MeshFacetBasis {
    public:
        /**
         * @brief Constructs a new MeshFacetBasis.
         * @param[in] M a const reference to the mesh
         * @param[in] f the index of the facet in @p M
         */
        MeshFacetBasis(const Mesh& M, const index_t f) {
            X = normalize(Geom::mesh_corner_vector(M, M.facets.corners_begin(f)));
            N = normalize(Geom::mesh_facet_normal(M, f));
            Y = cross(N, X);
        }

        /**
         * @brief Transforms a 3d vector into the local 2d basis.
         * @param[in] v the input 3d vector
         * @return the representation of @p v in the local 2d basis.
         */
        vec2 project(const vec3& v) const {
            return {dot(v, X), dot(v, Y)};
        }

        /**
         * @brief Transforms a local 2d vector into the global 3d basis.
         * @param[in] v the input 2d vector in the local basis
         * @return the representation of @p v in the global 3d basis.
         */
        vec3 unproject(const vec2& v) const {
            return v.x * X + v.y * Y;
        }

    private:
        vec3 X;
        vec3 Y;
        vec3 N;
    };

    /**
     * @brief Estimates the curvature tensor using a set of samples, each sample being a
     *        vector and a dihedral angle.
     * @details The algorithm is detailed in the following reference: Restricted Delaunay
     *          Triangulation and Normal Cycle, D. Cohen-Steiner and J.M. Morvan,
     *          SOCG 2003.
     */
    class NormalCycle {
    public:
        /**
         * @brief Constructs a new NormalCycle.
         */
        NormalCycle() {
            clear();
        }

        /**
         * @brief Clears the currently accumulated matrix.
         */
        void clear() {
            for (index_t i = 0; i < 6; ++i) {
                M_[i] = 0.0;
            }
        }

        /**
         * @brief Computes the eigenvalues and eigenvectors of the accumulated tensor.
         */
        void compute() {
            const double trace = M_[0] + M_[2] + M_[5];
            double s = 1e-6 * trace;
            if (trace == 0.0) {
                s = 1e-6;
            }
            M_[0] += s;
            M_[2] += s;
            M_[5] += s;

            double eigen_vectors[9];
            MatrixUtil::semi_definite_symmetric_eigen(M_, 3, eigen_vectors, eigen_value_);

            axis_[0] = vec3(eigen_vectors[0], eigen_vectors[1], eigen_vectors[2]);
            axis_[1] = vec3(eigen_vectors[3], eigen_vectors[4], eigen_vectors[5]);
            axis_[2] = vec3(eigen_vectors[6], eigen_vectors[7], eigen_vectors[8]);

            // Normalize the eigen vectors
            for (index_t i = 0; i < 3; ++i) {
                axis_[i] = normalize(axis_[i]);
            }

            // Sort the eigen vectors by increasing eigenvalue magnitude
            i_[0] = 0;
            i_[1] = 1;
            i_[2] = 2;

            double l0 = ::fabs(eigen_value_[0]);
            double l1 = ::fabs(eigen_value_[1]);
            double l2 = ::fabs(eigen_value_[2]);

            if (l1 > l0) {
                std::swap(l0, l1);
                std::swap(i_[0], i_[1]);
            }
            if (l2 > l1) {
                std::swap(l1, l2);
                std::swap(i_[1], i_[2]);
            }
            if (l1 > l0) {
                std::swap(l0, l1);
                std::swap(i_[0], i_[1]);
            }
        }

        /**
         * @brief Accumulates a dihedral angle to the current tensor.
         * @param[in] edge the supporting edge of the dihedron
         * @param[in] angle the angle of the dihedron
         * @param[in] neigh_area the area of the clipped neighborhood
         */
        void accumulate_dihedral_angle(
            const vec3& edge, const double angle, const double neigh_area = 1.0
        ) {
            const vec3 e = normalize(edge);
            const double s = length(edge) * angle * neigh_area;
            M_[0] += s * e.x * e.x;
            M_[1] += s * e.x * e.y;
            M_[2] += s * e.y * e.y;
            M_[3] += s * e.x * e.z;
            M_[4] += s * e.y * e.z;
            M_[5] += s * e.z * e.z;
        }

        /**
         * @brief Gets the estimated direction of maximum curvature.
         * @return the estimated direction of maximum curvature.
         */
        const vec3& Kmax() const {
            return eigen_vector(0);
        }

        /**
         * @brief Gets the estimated direction of minimum curvature.
         * @return the estimated direction of minimum curvature.
         */
        const vec3& Kmin() const {
            return eigen_vector(1);
        }

        /**
         * @brief Gets the estimated maximum curvature.
         * @return the estimated maximum curvature.
         */
        double kmax() const {
            return eigen_value(0);
        }

        /**
         * @brief Gets the estimated minimum curvature.
         * @return the estimated minimum curvature.
         */
        double kmin() const {
            return eigen_value(1);
        }

        /**
         * @brief Adds the currently accumulated tensor to a matrix.
         * @param[out] M an array of 6 doubles that represents the tensor to which the
         *  current tensor will be added
         */
        void add_to_matrix(double* M) const {
            for (index_t i = 0; i < 6; ++i) {
                M[i] += M_[i];
            }
        }

        /**
         * @brief Adds a matrix to the currently accumulated tensor.
         * @param[in] M an array of 6 doubles that represents the matrix that should be
         *  added to the currently accumulated tensor.
         */
        void add_matrix(const double* M) {
            for (index_t i = 0; i < 6; ++i) {
                M_[i] += M[i];
            }
        }

    private:
        /**
         * @brief Gets an eigenvector by index.
         * @param[in] i the index of the eigenvector (0, 1 or 2), the eigenvectors being
         *  sorted by increasing eigenvalue magnitude
         * @return the @p i-th eigenvector.
         */
        const vec3& eigen_vector(const int i) const {
            return axis_[i_[i]];
        }

        /**
         * @brief Gets an eigenvalue by index.
         * @param[in] i the index of the eigenvalue (0, 1 or 2), the eigenvalues being
         *  sorted by increasing magnitude
         * @return the @p i-th eigenvalue.
         */
        double eigen_value(const int i) const {
            return eigen_value_[i_[i]];
        }

        vec3 axis_[3];
        double eigen_value_[3];
        double M_[6];
        int i_[3];
    };

    /**
     * @brief Estimates the directions of minimum and maximum principal curvature, with
     *        their magnitudes.
     * @details Each direction is encoded as the cosine and the sine of the angle it makes
     *          relative to the first edge of its triangle, as defined by the
     *          MeshFacetBasis class. The magnitudes are the absolute values of the
     *          corresponding eigenvalues of the curvature tensor, that is @f$|k_{\max}|@f$
     *          and @f$|k_{\min}|@f$.
     * @param[in] M a const reference to the surface mesh
     * @param[out] max_sincos_alpha a vector of 2*M.facets.nb() doubles that receives the
     *  cosines and sines of the maximum curvature direction of every facet
     * @param[out] min_sincos_alpha the same for the minimum curvature direction
     * @param[in] locked a vector of M.facets.nb() booleans that indicates for each facet
     *  whether it is locked; locked facets are skipped, and an empty vector locks
     *  nothing
     * @param[out] max_magnitude a vector of M.facets.nb() doubles that receives the
     *  magnitude of the maximum curvature of every facet
     * @param[out] min_magnitude the same for the minimum curvature
     */
    void estimate_min_max_curvature_direction(
        const Mesh& M,
        std::vector<double>& max_sincos_alpha, std::vector<double>& min_sincos_alpha,
        const vector<bool>& locked,
        std::vector<double>& max_magnitude, std::vector<double>& min_magnitude
        ) {
        NormalCycle NC;
        vector<double> matrices(M.vertices.nb() * 6, 0.0);

        // Compute tensors of vertex neighborhoods
        for (const index_t f1 : M.facets) {
            for (const index_t c : M.facets.corners(f1)) {
                const index_t f2 = M.facet_corners.adjacent_facet(c);
                if (f2 == NO_FACET || f2 < f1) {
                    continue;
                }

                const index_t v1 = M.facet_corners.vertex(c);
                const index_t v2 = M.facet_corners.vertex(c + 1);

                const vec3 e = Geom::mesh_corner_vector(M, c);
                const double alpha = Geom::mesh_normal_angle(M, c);

                NC.clear();
                NC.accumulate_dihedral_angle(e, alpha);
                NC.add_to_matrix(&matrices[6 * v1]);
                NC.add_to_matrix(&matrices[6 * v2]);
            }
        }

        // For each facet, accumulate the tensors of all its vertices.
        for (const index_t f : M.facets) {
            if (locked.size() != 0 && locked[f]) {
                continue;
            }
            NC.clear();
            for (const index_t c : M.facets.corners(f)) {
                const index_t v = M.facet_corners.vertex(c);
                NC.add_matrix(&matrices[6 * v]);
            }
            NC.compute();

            const MeshFacetBasis basis(M, f);

            // Maximum curvature direction
            {
                const vec2 K = basis.project(NC.Kmax());
                const double angle = atan2(K.y, K.x) * curvature_symd;
                max_sincos_alpha[2 * f] = cos(angle);
                max_sincos_alpha[2 * f + 1] = sin(angle);
            }
            max_magnitude[f] = ::fabs(NC.kmax());

            // Minimum curvature direction
            {
                const vec2 K = basis.project(NC.Kmin());
                const double angle = atan2(K.y, K.x) * curvature_symd;
                min_sincos_alpha[2 * f] = cos(angle);
                min_sincos_alpha[2 * f + 1] = sin(angle);
            }
            min_magnitude[f] = ::fabs(NC.kmin());
        }
    }
}

namespace geolio
{
    void FrameField::create_curvature_directions(
        const GEO::Mesh& M,
        const double sharp_angle_threshold,
        const double autoscale
        ) {
        assert(std::isfinite(autoscale));
        assert(autoscale >= 0.0);

        if (M.facets.nb() == 0) {
            LOG::WARN("The mesh has no facet: nothing to do, the frame field is left "
                      "untouched.");
            return;
        }

        geo_assert(M.facets.are_simplices());

        // Step 1: the two principal curvature directions and their magnitudes, one of
        // each per facet.
        std::vector<double> max_sincos_alpha(2 * M.facets.nb(), 0.0);
        std::vector<double> min_sincos_alpha(2 * M.facets.nb(), 0.0);
        const GEO::vector<bool> locked(M.facets.nb());
        certainty_max_.assign(M.facets.nb(), 0.0);
        certainty_min_.assign(M.facets.nb(), 0.0);
        estimate_min_max_curvature_direction(
            M, max_sincos_alpha, min_sincos_alpha, locked,
            certainty_max_, certainty_min_
        );

        // Step 2: the factor that turns the raw magnitudes into the sizes of the frame
        // vectors. They are curvature measures rather than curvatures - lengths proportional
        // to the local facet area, see the class documentation - so autoscale fixes the
        // overall scale of a field that still follows the local sampling density.
        double scale = 1.0;
        if (const double max_magnitude = *std::ranges::max_element(certainty_max_);
            max_magnitude > 0.0
            ) {
            if (autoscale > 0.0) {
                scale = autoscale / max_magnitude;
                LOG::DEBUG(
                    "Largest curvature magnitude {}, scaling the frame sizes by {}",
                    max_magnitude, scale
                );
            }
        }

        // Step 3: the base class allocates the frame storage, the facet centers and the
        // spatial search, and solves for its own cross field, which Step 4 overwrites.
        // Its members are private, so running it is the only way to obtain that storage,
        // and it is the price of keeping the inherited queries consistent with the frames
        // published below (see the class documentation).
        LOG::DEBUG(
            "Initializing the frame storage and the spatial search with the base class' "
            "own field, which the curvature directions then replace"
        );
        GEO::FrameField::create_from_surface_mesh(M, false, sharp_angle_threshold);

        // Step 4: the frames themselves. Writing through frames() needs a const_cast: it
        // is the only accessor Geogram exposes for its private frames_, and it is well
        // defined here because this member function is non-const, hence the object is not
        // const.
        auto& frames = const_cast<GEO::vector<double>&>(this->frames());
        geo_assert(frames.size() == 9 * M.facets.nb());

        for (const GEO::index_t f : M.facets) {
            const MeshFacetBasis basis(M, f);

            const double max_angle = atan2(
                max_sincos_alpha[2 * f + 1],
                max_sincos_alpha[2 * f]
            ) / curvature_symd;
            vec3 U = basis.unproject(vec2(cos(max_angle), sin(max_angle)));

            const double min_angle = atan2(
                min_sincos_alpha[2 * f + 1],
                min_sincos_alpha[2 * f]
            ) / curvature_symd;
            vec3 V = basis.unproject(vec2(cos(min_angle), sin(min_angle)));

            const vec3 W = normalize(Geom::mesh_facet_normal(M, f));

            U *= scale * certainty_max_[f];
            V *= scale * certainty_min_[f];

            frames[9 * f + 0] = U.x;
            frames[9 * f + 1] = U.y;
            frames[9 * f + 2] = U.z;
            frames[9 * f + 3] = V.x;
            frames[9 * f + 4] = V.y;
            frames[9 * f + 5] = V.z;
            frames[9 * f + 6] = W.x;
            frames[9 * f + 7] = W.y;
            frames[9 * f + 8] = W.z;
        }
    }
}
