//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/31.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_DIRECTIONAL_OBJECT_H
#define GEOLIO_DIRECTIONAL_OBJECT_H
#include <string>
#include <vector>
#include <geogram/mesh/mesh.h>
#include "base_object.h"

namespace geolio::geobox
{
    /**
     * @brief Object that displays a directional field: N vectors per element of a
     *        mesh.
     * @details A directional field (a frame field, a cross field, ...) attaches
     *          @c vectors_nb_per_element vectors to every element of a mesh: to
     *          every facet of a surface field, or to every cell of a volumetric
     *          one. The vectors are stored flattened, 3 coordinates each, in
     *          element order: the vectors of element @c e are the ones at
     *          @c 3*N*e, i.e. N consecutive vectors.
     *          The object keeps a mesh of its own for the visualization, whose
     *          edges are the vectors: one edge per vector, from the center of its
     *          element along the direction of the vector, unit long. The length
     *          and the width of the edges, and their color, are per axis of the
     *          field (see AxisStyle), so that the three vectors of a frame can be
     *          told apart.
     *          The vectors are read from, and written to, a .fra file: the
     *          format only stores the vectors, so reload() keeps the geometry the
     *          object was created with.
     */
    class DirectionalObject : public BaseObject {
    public:
        /**
         * @brief How one axis of the field is drawn.
         * @details An axis is one of the @c vectors_nb_per_element vectors every
         *          element carries, i.e. one of the directions of the frames.
         */
        struct AxisStyle {
            /** Color of the axis. */
            GEO::vec4f color = GEO::vec4f(1.0f, 0.0f, 0.0f, 1.0f);

            /** Length of the vectors, as a fraction of the diagonal of the
             *  bounding box of the field (1 would make them as long as it). */
            float length = 0.03f;

            /** Width of the lines the vectors are drawn with, like the sliders
             *  of the mesh wireframes (0.1 .. 2). */
            float width = 0.3f;
        };

        /**
         * @brief Constructs an object that displays a directional field.
         * @param[in] name Display name of the object.
         * @param[in] filepath Path of the .fra file the vectors were read from; an
         *            empty string when they were computed (reload() then has
         *            nothing to read).
         * @param[in] mesh Mesh the field is defined on: the vectors of an element
         *            are drawn from the center of that element.
         * @param[in] vectors_nb_per_element Number N of vectors each element
         *            carries (3 for a frame field, 2 for a cross field in 2D, ...).
         * @param[in] vectors The vectors, 3 coordinates each, N vectors per
         *            element, in element order: 3*N*nb_element values.
         * @param[in] subelements Whether the field is defined on the facets
         *            (GEO::MESH_FACETS) or on the cells (GEO::MESH_CELLS) of
         *            @p mesh.
         */
        DirectionalObject(
            std::string name,
            std::string filepath,
            const GEO::Mesh& mesh,
            GEO::index_t vectors_nb_per_element,
            const std::vector<double>& vectors,
            GEO::MeshElementsFlags subelements = GEO::MESH_FACETS);

        void draw_object_properties() override;

        void draw_scene(bool lighting) override;

        void get_bbox(double* xyzmin, double* xyzmax) const override;

        /**
         * @brief Reads the vectors of the field again, from its .fra file.
         * @details The geometry the object was created with is kept: the format
         *          only holds the vectors. Nothing is changed when the file
         *          cannot be read, holds a different number of vectors per
         *          element, or a different number of elements.
         */
        void reload() override;

        bool save(const std::string& filename) override;

        /**
         * @brief Whether the field can be written in a file format.
         * @details The vectors are stored in the .fra format (see
         *          geolio::fra_save()), and nowhere else.
         */
        [[nodiscard]] bool can_write_extension(
            const std::string& extension
            ) const override;

        /** @brief Number of vectors each element of the field carries. */
        [[nodiscard]] GEO::index_t vectors_nb_per_element() const {
            return vectors_nb_per_element_;
        }

        /** @brief Number of elements the field is defined on. */
        [[nodiscard]] GEO::index_t nb_elements() const {
            return centers_.size();
        }

        /** @brief The vectors of the field, 3 coordinates each. */
        [[nodiscard]] const std::vector<double>& vectors() const {
            return vectors_;
        }

        /** @brief The mesh that shows the field: one edge per vector. */
        [[nodiscard]] const GEO::Mesh& mesh() const { return mesh_; }

        /** @brief How the axis @p axis of the field is drawn. */
        [[nodiscard]] const AxisStyle& axis_style(const GEO::index_t axis) const {
            return axis_styles_[axis];
        }

        /** @brief How the axis @p axis of the field is drawn. */
        [[nodiscard]] AxisStyle& axis_style(const GEO::index_t axis) {
            return axis_styles_[axis];
        }

    protected:
        /**
         * @brief Rebuilds the visualization mesh from the centers and the
         *        vectors.
         * @details One edge per vector: from the center of its element to that
         *          center plus the unit direction of the vector. The axis length
         *          is applied while drawing, so that the length can be changed
         *          without rebuilding the mesh.
         */
        void build_visualization_mesh();

        /** Centers of the elements the field is defined on. */
        std::vector<GEO::vec3> centers_;

        /** The field: 3 coordinates per vector, N vectors per element. */
        std::vector<double> vectors_;

        /** Number of vectors each element carries. */
        GEO::index_t vectors_nb_per_element_ = 0;

        /** Whether the field is defined on the facets or on the cells. */
        GEO::MeshElementsFlags subelements_ = GEO::MESH_FACETS;

        /** How each axis of the field is drawn. */
        std::vector<AxisStyle> axis_styles_;

        /** The visualization mesh: one edge per vector. */
        GEO::Mesh mesh_;

        /** Diagonal of the bounding box of the elements the field is defined on,
         *  used to interpret the axis lengths (see AxisStyle::length). */
        float elements_bbox_diagonal_ = 1.0f;
    };
}

#endif //GEOLIO_DIRECTIONAL_OBJECT_H
