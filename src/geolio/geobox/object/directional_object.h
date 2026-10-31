//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/31.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_DIRECTIONAL_OBJECT_H
#define GEOLIO_DIRECTIONAL_OBJECT_H
#include <memory>
#include <string>
#include <vector>
#include <geogram/mesh/mesh.h>
#include "base_object.h"

namespace geolio::geobox
{
    class MeshObject;

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
     *          element along the direction of the vector, unit long. All the axes
     *          are drawn with the same length and the same width (see length() and
     *          width()); each of them has its own color and can be hidden (see
     *          AxisStyle), so that the three vectors of a frame can be told
     *          apart. draw_opposite() draws every vector in both directions, as a
     *          line through the center of its element.
     *          The vectors are read from, and written to, a .fra file: the
     *          format only stores the vectors, so the object does not know where
     *          to draw them. It is attached to another object of the scene (see
     *          attach_to()): the frames are drawn from the centers of the
     *          vertices, of the facets or of the cells of its mesh, whichever has
     *          as many elements as the field has frames. The object properties
     *          offer the mesh objects and the kind of element to attach to.
     */
    class DirectionalObject : public BaseObject {
    public:
        /**
         * @brief How one axis of the field is drawn.
         * @details An axis is one of the @c vectors_nb_per_element vectors every
         *          element carries, i.e. one of the directions of the frames.
         *          The length and the width of the lines are the same for all
         *          the axes (see length() and width()).
         */
        struct AxisStyle {
            /** Whether the axis is drawn at all. */
            bool visible = true;

            /** Color of the axis. */
            GEO::vec4f color = GEO::vec4f(1.0f, 0.0f, 0.0f, 1.0f);
        };

        /**
         * @brief Constructs an object that displays a directional field.
         * @details The field does not know what it is attached to: the object is
         *          attached to a mesh object with attach_to(), or by picking one
         *          in its properties. Until then it draws nothing.
         * @param[in] name Display name of the object.
         * @param[in] filepath Path of the .fra file the vectors were read from; an
         *            empty string when they were computed (reload() then has
         *            nothing to read).
         * @param[in] objects Objects of the scene; kept by reference, so that the
         *            properties offer the mesh objects the field can be attached
         *            to.
         * @param[in] vectors_nb_per_element Number N of vectors each element
         *            carries (3 for a frame field, 2 for a cross field in 2D, ...).
         * @param[in] vectors The vectors, 3 coordinates each, N vectors per
         *            element, in element order: 3*N*nb_element values.
         */
        DirectionalObject(
            std::string name,
            std::string filepath,
            ApplicationObjects& objects,
            GEO::index_t vectors_nb_per_element,
            const std::vector<double>& vectors);

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

        /** @brief Number of frames of the field, i.e. of elements it describes.
         *  @details It is what the mesh the field is attached to must provide:
         *          as many vertices, facets or cells as there are frames. */
        [[nodiscard]] GEO::index_t nb_frames() const;

        /** @brief Number of elements the vectors are currently drawn on, i.e. 0
         *  while the field is not attached to a mesh. */
        [[nodiscard]] GEO::index_t nb_elements() const {
            return centers_.size();
        }

        /** @brief The mesh object the field is attached to, if any. */
        [[nodiscard]] const std::weak_ptr<MeshObject>& mesh_object() const {
            return mesh_object_;
        }

        /**
         * @brief Whether the field is attached to a mesh it can be drawn on.
         * @details False while no mesh object was picked, and after the one it
         *          was attached to was removed from the scene: the scene owns the
         *          objects, so an attachment never keeps a mesh alive. A field
         *          that is not attached is not drawn, and is not part of the
         *          bounding box of the scene either.
         */
        [[nodiscard]] bool attached() const;

        /**
         * @brief The mesh the frames are attached to, or nullptr when the field
         *        is not attached (see attached()).
         * @details That is the mesh of mesh_object(), the one the frames are
         *          drawn from the elements of. The mutable overload is the one an
         *          operation that works on that mesh needs (e.g. the streamline
         *          tracer, which stores its directions on it).
         */
        [[nodiscard]] const GEO::Mesh* attached_mesh() const;
        [[nodiscard]] GEO::Mesh* attached_mesh();

        /**
         * @brief Name of the elements a field can be attached to.
         * @param[in] subelements One of GEO::MESH_VERTICES, GEO::MESH_FACETS and
         *            GEO::MESH_CELLS (GEO::MESH_NONE for no attachment).
         * @return "vertices", "facets", "cells", or "nothing".
         */
        [[nodiscard]] static const char* subelements_name(
            GEO::MeshElementsFlags subelements);

        /** @brief The elements of the mesh the frames are attached to: one of
         *  GEO::MESH_VERTICES, GEO::MESH_FACETS and GEO::MESH_CELLS, or
         *  GEO::MESH_NONE while the field is not attached. */
        [[nodiscard]] GEO::MeshElementsFlags subelements() const {
            return subelements_;
        }

        /**
         * @brief Attaches the field to the elements of a mesh object.
         * @details The frames are drawn from the center of the elements they are
         *          attached to: the centers are read again from @p mesh_object,
         *          and the visualization mesh is rebuilt. The mesh object must
         *          have as many vertices, facets or cells as the field has frames
         *          (see nb_frames()), and @p subelements says which of them to
         *          use; nothing changes when it does not.
         * @param[in] mesh_object The mesh object to attach the field to.
         * @param[in] subelements The elements of its mesh the frames sit on: one
         *            of GEO::MESH_VERTICES, GEO::MESH_FACETS and GEO::MESH_CELLS.
         * @return true when the field was attached.
         */
        bool attach_to(
            const std::shared_ptr<MeshObject>& mesh_object,
            GEO::MeshElementsFlags subelements);

        /**
         * @brief The kinds of element of @p mesh_object that have as many
         *        elements as the field has frames.
         * @details These are the ones the field can be attached to; there can be
         *          several when the mesh has as many vertices as facets, for
         *          instance.
         * @param[in] mesh_object Mesh object to test.
         * @return The matching kinds, out of GEO::MESH_VERTICES,
         *         GEO::MESH_FACETS and GEO::MESH_CELLS.
         */
        [[nodiscard]] std::vector<GEO::MeshElementsFlags> attachable_subelements(
            const MeshObject& mesh_object) const;

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

        /** @brief Length of the vectors, as a fraction of the diagonal of the
         *  bounding box of the mesh the field is defined on (1 would make them
         *  as long as it). */
        [[nodiscard]] float length() const { return length_; }

        /** @brief Diagonal of the bounding box of the mesh the field is defined
         *  on, i.e. what length() is a fraction of. */
        [[nodiscard]] float bbox_diagonal() const {
            return elements_bbox_diagonal_;
        }

        /** @brief Length of the vectors (see length()). */
        void set_length(const float length) { length_ = length; }

        /** @brief Width of the lines the vectors are drawn with, like the
         *  sliders of the mesh wireframes (0.1 .. 2). */
        [[nodiscard]] float width() const { return width_; }

        /** @brief Width of the lines the vectors are drawn with (see width()). */
        void set_width(const float width) { width_ = width; }

        /** @brief Whether the opposite of every vector is drawn too, i.e. the
         *  axes are drawn as lines through the center rather than as arrows. */
        [[nodiscard]] bool draw_opposite() const { return draw_opposite_; }

        /** @brief Whether the opposite of every vector is drawn (see
         *  draw_opposite()). */
        void set_draw_opposite(const bool draw_opposite) {
            draw_opposite_ = draw_opposite;
        }

    protected:
        /**
         * @brief Rebuilds the visualization mesh from the centers and the
         *        vectors.
         * @details One edge per vector: from the center of its element to that
         *          center plus the unit direction of the vector. The length is
         *          applied while drawing, so that it can be changed without
         *          rebuilding the mesh.
         */
        void build_visualization_mesh();

        /** @brief Length of the vectors in model coordinates, i.e. length()
         *  times the diagonal of the mesh they are defined on. */
        [[nodiscard]] double length_in_model_units() const;

        /**
         * @brief Reads the centers of the elements the field is attached to, and
         *        the size of their mesh.
         * @details Fills centers_ with the center of every vertex, facet or cell
         *          of the mesh object (see subelements_), and
         *          elements_bbox_diagonal_ with the diagonal of the bounding box
         *          of that mesh, which length() is a fraction of.
         */
        void read_centers();

        /**
         * @brief Draws the mesh object picker and the element picker.
         * @return true when the field was attached to something else.
         */
        bool draw_attachment_pickers();

        /** Centers of the elements the field is defined on. */
        std::vector<GEO::vec3> centers_;

        /** The field: 3 coordinates per vector, N vectors per element. */
        std::vector<double> vectors_;

        /** Number of vectors each element carries. */
        GEO::index_t vectors_nb_per_element_ = 0;

        /** The mesh object the frames are attached to, if any. */
        std::weak_ptr<MeshObject> mesh_object_;

        /** The elements of that mesh the frames sit on (GEO::MESH_NONE while the
         *  field is not attached). */
        GEO::MeshElementsFlags subelements_ = GEO::MESH_NONE;

        /** Objects of the scene, the ones the properties offer to attach to. */
        ApplicationObjects& objects_;

        /** How each axis of the field is drawn. */
        std::vector<AxisStyle> axis_styles_;

        /** The visualization mesh: one edge per vector. */
        GEO::Mesh mesh_;

        /** Diagonal of the bounding box of the mesh the field is defined on,
         *  used to interpret the length of the vectors (see length()). */
        float elements_bbox_diagonal_ = 1.0f;

        /** Length of the vectors, as a fraction of elements_bbox_diagonal_. */
        float length_ = 0.03f;

        /** Width of the lines the vectors are drawn with. */
        float width_ = 0.3f;

        /** Whether the opposite of every vector is drawn as well. */
        bool draw_opposite_ = true;
    };
}

#endif //GEOLIO_DIRECTIONAL_OBJECT_H
