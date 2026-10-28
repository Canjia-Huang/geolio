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
#include <geogram/points/nn_search.h>
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
        [[nodiscard]] vec2 project(const vec3& v) const {
            return {dot(v, X), dot(v, Y)};
        }

        /**
         * @brief Transforms a local 2d vector into the global 3d basis.
         * @param[in] v the input 2d vector in the local basis
         * @return the representation of @p v in the global 3d basis.
         */
        [[nodiscard]] vec3 unproject(const vec2& v) const {
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
        [[nodiscard]] const vec3& Kmax() const {
            return eigen_vector(0);
        }

        /**
         * @brief Gets the estimated direction of minimum curvature.
         * @return the estimated direction of minimum curvature.
         */
        [[nodiscard]] const vec3& Kmin() const {
            return eigen_vector(1);
        }

        /**
         * @brief Gets the estimated maximum curvature.
         * @return the estimated maximum curvature.
         */
        [[nodiscard]] double kmax() const {
            return eigen_value(0);
        }

        /**
         * @brief Gets the estimated minimum curvature.
         * @return the estimated minimum curvature.
         */
        [[nodiscard]] double kmin() const {
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
        [[nodiscard]] const vec3& eigen_vector(const int i) const {
            return axis_[i_[i]];
        }

        /**
         * @brief Gets an eigenvalue by index.
         * @param[in] i the index of the eigenvalue (0, 1 or 2), the eigenvalues being
         *  sorted by increasing magnitude
         * @return the @p i-th eigenvalue.
         */
        [[nodiscard]] double eigen_value(const int i) const {
            return eigen_value_[i_[i]];
        }

        vec3 axis_[3];
        double eigen_value_[3];
        double M_[6];
        int i_[3];
    };

    /**
     * @brief Computes the direction in which a dihedron folds.
     * @details The two facets sharing @p edge meet at that edge, so both of their normals
     *          are orthogonal to it, and the surface turns in the direction perpendicular
     *          to the edge inside the (average) tangent plane. That perpendicular is the
     *          direction a curvature tensor has to accumulate.
     * @details Accumulating the edge itself instead rotates every principal direction by
     *          90 degrees in its facet plane. A 4-fold symmetric cross field - the object
     *          the original formulation of this estimator was written for - is invariant
     *          under that rotation and therefore does not see the difference, but a 2-fold
     *          symmetric curvature direction field does: it gets the two directions
     *          swapped, together with the two magnitudes.
     * @param[in] M a const reference to the surface mesh
     * @param[in] f1 the index of the first facet of the dihedron
     * @param[in] f2 the index of the second facet of the dihedron
     * @param[in] edge the edge the two facets share, oriented either way
     * @return a vector perpendicular to @p edge in the average tangent plane, with the
     *         length of @p edge, so that the weight of the sample does not change.
     */
    vec3 fold_direction(const Mesh& M, const index_t f1, const index_t f2, const vec3& edge) {
        vec3 n = Geom::mesh_facet_normal(M, f1) + Geom::mesh_facet_normal(M, f2);
        if (length(n) < 1e-30) {
            // A 180-degree fold has no average normal; either one defines the same
            // perpendicular.
            n = Geom::mesh_facet_normal(M, f1);
        }
        if (length(n) < 1e-30) {
            // Degenerate facets: there is no tangent plane to turn in, keep the edge.
            return edge;
        }
        return cross(normalize(n), edge);
    }

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
                NC.accumulate_dihedral_angle(fold_direction(M, f1, f2, e), alpha);
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

    /**
     * @brief Finds, for every cell of a tetrahedral mesh, the facet whose center is nearest
     *        to the center of the cell.
     * @details This is the extrapolation step of
     *          ``GEO::FrameField::create_from_surface_mesh()`` in volumetric mode: the field
     *          is known on the facets, and a cell inherits the frame of the nearest facet.
     *          Geogram performs the lookup with the same kind of search structure, over the
     *          facet centers, hence the same notion of "nearest".
     * @param[in] M a const reference to the mesh
     * @return one facet index per cell, in cell order.
     */
    std::vector<index_t> nearest_facet_per_cell(const Mesh& M) {
        std::vector<double> centers(3 * M.facets.nb());
        for (const index_t f : M.facets) {
            const vec3 c = Geom::mesh_facet_center(M, f);
            centers[3 * f] = c.x;
            centers[3 * f + 1] = c.y;
            centers[3 * f + 2] = c.z;
        }

        NearestNeighborSearch_var NN = NearestNeighborSearch::create(3, "default");
        NN->set_points(M.facets.nb(), centers.data());

        std::vector<index_t> nearest(M.cells.nb(), NO_FACET);
        for (const index_t t : M.cells) {
            const vec3 c = Geom::mesh_cell_center(M, t);
            nearest[t] = NN->get_nearest_neighbor(c.data());
        }
        return nearest;
    }
}

namespace geolio
{
    void FrameField::create_curvature_directions(
        const GEO::Mesh& mesh,
        const bool volumetric,
        const double sharp_angle_threshold,
        const double autoscale
        ) {
        assert(std::isfinite(autoscale));
        assert(autoscale >= 0.0);

        if (mesh.facets.nb() == 0) {
            LOG::WARN("The mesh has no facet: nothing to do, the frame field is left "
                      "untouched.");
            return;
        }

        assert(mesh.facets.are_simplices());

        // Volumetric frames are attached to the cells, so a mesh that has none cannot hold
        // them. Rather than publish an empty field, fall back to the surface field that the
        // curvature estimate defines directly.
        bool volumetric_mode = volumetric;
        if (volumetric_mode && mesh.cells.nb() == 0) {
            LOG::WARN("volumetric was requested but the mesh has no cell: building the "
                      "surface frame field instead.");
            volumetric_mode = false;
        }
        if (volumetric_mode) {
            // The base class' volumetric mode puts the points of its search structure at
            // mesh_tet_center() of every cell, which is the centroid of the cell only for a
            // tetrahedron.
            assert(mesh.cells.are_simplices());
        }
        volumetric_ = volumetric_mode;

        // Step 1: the two principal curvature directions and their magnitudes, one of
        // each per facet.
        std::vector<double> max_sincos_alpha(2 * mesh.facets.nb(), 0.0);
        std::vector<double> min_sincos_alpha(2 * mesh.facets.nb(), 0.0);
        const GEO::vector<bool> locked(mesh.facets.nb());
        certainty_max_.assign(mesh.facets.nb(), 0.0);
        certainty_min_.assign(mesh.facets.nb(), 0.0);
        estimate_min_max_curvature_direction(
            mesh, max_sincos_alpha, min_sincos_alpha, locked,
            certainty_max_, certainty_min_
        );

        // Step 2: the facet each cell inherits its frame from, in volumetric mode. The
        // lookup uses the facet centers, as in the base class' extrapolation.
        std::vector<GEO::index_t> cell_facet;
        if (volumetric_mode) {
            cell_facet = nearest_facet_per_cell(mesh);
        }

        // Step 3: the factor that turns the raw magnitudes into the sizes of the frame
        // vectors. They are curvature measures rather than curvatures - lengths proportional
        // to the local facet area, see the class documentation - so autoscale fixes the
        // overall scale of a field that still follows the local sampling density. The
        // reference is the largest magnitude that is actually published: over the facets in
        // surface mode, over the facets the cells inherit from in volumetric mode, so that
        // autoscale always names the largest size of the field.
        double reference_magnitude = 0.0;
        if (volumetric_mode) {
            for (const GEO::index_t t : mesh.cells) {
                reference_magnitude = std::max(reference_magnitude, certainty_max_[cell_facet[t]]);
            }
        } else {
            for (const double magnitude : certainty_max_) {
                reference_magnitude = std::max(reference_magnitude, magnitude);
            }
        }

        double scale = 1.0;
        if (reference_magnitude > 0.0 && autoscale > 0.0) {
            scale = autoscale / reference_magnitude;
            LOG::DEBUG(
                "Largest curvature magnitude {}, scaling the frame sizes by {}",
                reference_magnitude, scale
            );
        }

        // Step 4: the base class allocates the frame storage, the centers and the spatial
        // search, and solves for its own cross field, which Step 5 overwrites. Its members
        // are private, so running it is the only way to obtain that storage, and it is the
        // price of keeping the inherited queries consistent with the frames published below
        // (see the class documentation). In volumetric mode it allocates one frame per cell
        // and sets the search structure up on the cell centers, which is precisely the
        // storage the extrapolated field needs.
        LOG::DEBUG(
            "Initializing the frame storage and the spatial search with the base class' "
            "own field, which the curvature directions then replace"
        );
        GEO::FrameField::create_from_surface_mesh(mesh, volumetric_mode, sharp_angle_threshold);

        // Step 5: the frames themselves. Writing through frames() needs a const_cast: it
        // is the only accessor Geogram exposes for its private frames_, and it is well
        // defined here because this member function is non-const, hence the object is not
        // const.
        auto& frames = const_cast<GEO::vector<double>&>(this->frames());

        const auto write_facet_frame = [&](const GEO::index_t f, double* frame) {
            const MeshFacetBasis basis(mesh, f);

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

            const vec3 W = normalize(Geom::mesh_facet_normal(mesh, f));

            U *= scale * certainty_max_[f];
            V *= scale * certainty_min_[f];

            frame[0] = U.x;
            frame[1] = U.y;
            frame[2] = U.z;
            frame[3] = V.x;
            frame[4] = V.y;
            frame[5] = V.z;
            frame[6] = W.x;
            frame[7] = W.y;
            frame[8] = W.z;
        };

        if (!volumetric_mode) {
            geo_assert(frames.size() == 9 * mesh.facets.nb());
            for (const GEO::index_t f : mesh.facets) {
                write_facet_frame(f, &frames[9 * f]);
            }
            return;
        }

        geo_assert(frames.size() == 9 * mesh.cells.nb());

        // A facet frame is shared by every cell it is the nearest facet of, so each one is
        // computed once and then copied to the cells that inherit it.
        std::vector<double> facet_frames(9 * mesh.facets.nb());
        for (const GEO::index_t f : mesh.facets) {
            write_facet_frame(f, &facet_frames[9 * f]);
        }
        for (const GEO::index_t t : mesh.cells) {
            std::copy_n(
                facet_frames.data() + 9 * cell_facet[t], 9, frames.data() + 9 * t
            );
        }
    }

    void FrameField::compute_anisotropy_matrices(
        const GEO::Mesh& mesh,
        const double max_aspect,
        std::vector<double>& matrices
        ) const {
        LOG::TRACE("{}({})", __FUNCTION__, max_aspect);

        const auto& frames = this->frames();

        if (volumetric_)
            throw std::logic_error(
                "The frame field is volumetric: it holds one frame per tetrahedron, while "
                "this function computes one matrix per facet");

        if (frames.size() != 9*mesh.facets.nb())
            throw std::logic_error(
                "The frame field does not match the mesh ("+std::to_string(frames.size())+" values for "+std::to_string(mesh.facets.nb())+" facets, "+std::to_string(9*mesh.facets.nb())+" expected), "
               "re-compute it with create_curvature_directions()");

        // The curvature magnitudes give the anisotropy. When the field does not
        // carry them (a field built by create_from_surface_mesh() or create_curvature_directions(), for instance), fall
        // back to the lengths of the two frame vectors: the anisotropic variant of
        // compute_principal_curvature_field() sets them to the certainty values
        // times a single common scale, so their ratio is the same anisotropy. A
        // field with unit vectors therefore yields the identity, i.e. an isotropic
        // metric, which is what it says.
        const bool with_certainty = (
            certainty_max_.size() == mesh.facets.nb() &&
            certainty_min_.size() == mesh.facets.nb());
        if (!with_certainty)
            LOG::WARN("The frame field carries no curvature certainty, deriving the "
                       "anisotropy from the frame vector lengths");

        const double aspect_span = std::max(1.0, max_aspect) - 1.0;

        matrices.clear();
        matrices.reserve(9*mesh.facets.nb());
        for (const auto& f : mesh.facets) {
            // Orthonormal frame (U, V, N): U is the max curvature direction, N the
            // facet normal. The stored vectors are re-orthonormalized rather than
            // trusted: the field stores them per facet and floating point error, or
            // a degenerate eigen decomposition, can leave them slightly off.
            GEO::vec3 N = GEO::Geom::mesh_facet_normal(mesh, f);
            if (GEO::length(N) < 1e-30)
                N = GEO::vec3(0, 0, 1);  // degenerate facet, keep going
            N = GEO::normalize(N);

            const GEO::vec3 stored_U(frames[9*f], frames[9*f+1], frames[9*f+2]);
            const GEO::vec3 stored_V(frames[9*f+3], frames[9*f+4], frames[9*f+5]);

            GEO::vec3 U = stored_U;
            U -= GEO::dot(U, N)*N;  // project into the facet plane
            if (GEO::length(U) < 1e-12) {
                // No direction at all: fall back to the first edge of the facet.
                const GEO::vec3 e = mesh.facets.point(f, 1) - mesh.facets.point(f, 0);
                U = e - GEO::dot(e, N)*N;
            }
            if (GEO::length(U) < 1e-30)
                U = GEO::cross(N, GEO::vec3(1, 0, 0));
            U = GEO::normalize(U);
            const GEO::vec3 V = GEO::normalize(GEO::cross(N, U));

            // Relative anisotropy of the principal curvatures, in [0, 1]:
            // 0 where the two curvatures are equal (planar regions and umbilic
            // points carry no direction) and 1 where the smallest one vanishes
            // (developable regions have a well defined direction).
            const double k0 = std::fabs(with_certainty ? certainty_max_[f] : GEO::length(stored_U));
            const double k1 = std::fabs(with_certainty ? certainty_min_[f] : GEO::length(stored_V));
            const double sum = k0 + k1;
            const double relative_anisotropy = (sum > 0) ? (k0 - k1)/sum : 0.0;

            // Weights of the two curvature directions, normalized to unit
            // determinant: no facet is made globally smaller than another, so the
            // anisotropy cannot turn into a (huge) density bias. The larger weight
            // goes along the max curvature direction, i.e. cells are short across
            // a ridge and long along it.
            const double aspect = 1.0 + aspect_span*std::max(0.0, relative_anisotropy);
            const double s = std::sqrt(aspect);
            const GEO::vec3 w(s, 1.0/s, 1.0);

            // M = R diag(w) R^T, R = (U, V, N)
            for (GEO::index_t i = 0; i < 3; ++i) {
                for (GEO::index_t j = 0; j < 3; ++j) {
                    matrices.push_back(
                        w[0]*U[i]*U[j] +
                        w[1]*V[i]*V[j] +
                        w[2]*N[i]*N[j]);
                }
            }
        }
    }
}
