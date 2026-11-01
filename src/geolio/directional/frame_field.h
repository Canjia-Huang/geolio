//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/3.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_FRAME_FIELD_H
#define GEOLIO_FRAME_FIELD_H
#include <geogram/mesh/mesh_frame_field.h>

namespace geolio
{
    class FrameField : public GEO::FrameField {
    public:
        /**
         * @brief Creates a frame field aligned with the principal curvature directions,
         *        optionally keeping their magnitudes as the vector sizes.
         *
         * The curvature is estimated on the facets, so the field is defined on the surface.
         * By default the frames are attached to those facets. With @p volumetric they are
         * attached to the cells instead, the way
         * ``GEO::FrameField::create_from_surface_mesh()`` extrapolates a surface field to a
         * volume: every cell receives the frame of the facet whose center is nearest to the
         * center of the cell.
         *
         * @param[in] mesh the input mesh, triangulated, with its facet adjacency computed
         *                 (``mesh.facets.connect()``) so that the curvature estimator can
         *                 read dihedral angles; it must also be tetrahedralized when
         *                 @p volumetric is true
         * @param[in] volumetric if true, publish one frame per tetrahedron instead of one per
         *                 facet; a mesh without any cell is built as a surface field, with a
         *                 warning
         * @param[in] sharp_angle_threshold angles smaller than this threshold
         *                 (in degrees) are considered to be sharp features
         * @param[in] autoscale if positive, normalize the sizes so that the largest
         *                 published one is exactly @p autoscale; if 0, store the raw
         *                 magnitudes. A negative or non-finite value is rejected, as is a
         *                 mesh without facets, both with an error or warning message and
         *                 without touching the object.
         * @warning With @p autoscale equal to 0, flat facets get zero-length vectors; see the
         *          class documentation.
         */
        void create_curvature_directions(
            const GEO::Mesh& mesh,
            bool volumetric = false,
            double sharp_angle_threshold = 45.0,
            double autoscale = 0.0);

        /**
         * @brief Computes one anisotropy matrix for each element from a given frame array.
         *
         * An element is a facet of a surface field, or a cell of a volumetric one, as
         * selected by @p volumetric. The matrices are symmetric positive-definite 3x3
         * metrics. Their principal directions follow @p frames, while their anisotropy is
         * controlled by the relative magnitudes of the two curvature directions. The
         * matrices are written in element order and stored consecutively in row-major
         * order.
         *
         * The metrics measure lengths, so a large weight means short cells: the larger
         * weight is the one along the maximum curvature direction, i.e. cells are made
         * short across a ridge and long along it. The weight along the third axis is 1 -
         * the facet normal, or, for a cell, the normal of the facet it inherits its frame
         * from - and the two in-plane weights multiply to 1, so that an element is never
         * made globally smaller or larger than another: the anisotropy cannot turn into a
         * density bias. Their ratio is at most @p max_aspect, which it reaches where the
         * smallest curvature vanishes, i.e. on a developable region. An isotropic element
         * (flat, or umbilic) therefore gets the identity.
         *
         * The curvature magnitudes are read from this field when it carries them, i.e.
         * when it was built by create_curvature_directions() in the same mode as @p
         * volumetric. Otherwise - for a field built by
         * ``GEO::FrameField::create_from_surface_mesh()``, say - the lengths of the two
         * frame vectors are used instead: create_curvature_directions() stores the
         * certainty values times one common scale, so their ratio is the same anisotropy,
         * whereas a field of unit vectors yields the identity, which is what it describes.
         * That fallback is reported with a warning.
         *
         * @param[in] mesh the mesh whose elements correspond to @p frames; it supplies the
         *                 element counts and, when @p volumetric is false, the facet normals
         * @param[in] volumetric if true, the elements are the cells of @p mesh, one frame per
         *                 cell; if false, they are its facets, one frame per facet. It picks
         *                 the number of elements, hence the size @p frames must have, and
         *                 where the third axis of a frame comes from
         * @param[in] frames a flat array of ``9 * nb_elements`` doubles, the row-major
         *                 ``(U, V, N)`` frame of each element, where ``nb_elements`` is
         *                 ``mesh.cells.nb()`` when @p volumetric is true and
         *                 ``mesh.facets.nb()`` otherwise. U is the maximum curvature
         *                 direction and N the third axis: the facet normal on a surface, or
         *                 the inherited facet normal on a cell. The array is trusted as
         *                 given: its size and the orthonormality of the frames are not
         *                 checked, the stored vectors are merely re-orthonormalized
         * @param[in] max_aspect the maximum ratio of the two in-plane weights, i.e. the
         *                       square of the largest aspect ratio the cells can reach;
         *                       values below 1 are treated as 1
         * @param[out] matrices output vector, replaced with exactly
         *                       ``9 * nb_elements`` coefficients. The matrix for element @p e
         *                       starts at ``matrices[9 * e]`` and its coefficient at row @p i
         *                       and column @p j is stored at
         *                       ``matrices[9 * e + 3 * i + j]``
         *
         * @note A volumetric field inherits the frame of the nearest facet, so a cell gets
         *       the anisotropy of that facet.
         * @warning This overload does not check @p frames against @p mesh nor @p volumetric
         *          against the mode the field was built in; prefer the one below, or check
         *          both yourself, before passing an array that may come from another field
         *          or mesh.
         */
        void compute_anisotropy_matrices(
            const GEO::Mesh& mesh,
            bool volumetric,
            const double* frames,
            double max_aspect,
            std::vector<double>& matrices) const;

        /**
         * @brief Computes one anisotropy matrix for each element from this field's frames.
         *
         * Equivalent to the overload above, with @p frames taken from frames(); see it for
         * the layout and the meaning of the matrices.
         *
         * @param[in] mesh the mesh whose elements correspond to the current frame field; it
         *                 must have as many cells as the field holds frames when @p
         *                 volumetric is true, and as many facets otherwise
         * @param[in] volumetric if true, one frame per cell of @p mesh; if false, one frame
         *                 per facet. It must match the mode the field was built in, both for
         *                 the frame count to line up and for the curvature magnitudes to be
         *                 reused instead of falling back to the frame vector lengths
         * @param[in] max_aspect the maximum ratio of the two in-plane weights; values
         *                       below 1 are treated as 1
         * @param[out] matrices output vector, replaced with exactly ``9 * nb_elements``
         *                       coefficients, in the layout described above; it is left
         *                       untouched when the call throws
         *
         * @throws std::logic_error if the field does not hold exactly ``9 * nb_elements``
         *         values, i.e. not one frame per element of @p mesh in the mode requested by
         *         @p volumetric; the message asks to re-compute the field with
         *         create_curvature_directions()
         */
        void compute_anisotropy_matrices(
            const GEO::Mesh& mesh,
            bool volumetric,
            double max_aspect,
            std::vector<double>& matrices) const;

    private:
        /// The estimated @f$|k_{\max}|@f$ per element: per facet, or per cell in volumetric
        /// mode, where a cell carries the value of the facet it inherits its frame from.
        std::vector<double> certainty_max_;

        /// The estimated @f$|k_{\min}|@f$ per element, with the same layout as
        /// @ref certainty_max_.
        std::vector<double> certainty_min_;
    };
}

#endif //GEOLIO_FRAME_FIELD_H
