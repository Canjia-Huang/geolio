//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/11/23.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "mphtxt_io.h"

#include <cstring>
#include <string>

#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_io.h>

#include "line_stream.h"
#include <geolio/common/log.h>

namespace geolio
{
    namespace
    {
        /**
         * @brief The fields of a .mphtxt file that describe the current Mesh
         *        object.
         * @details A file may hold several Mesh objects (COMSOL exports each
         *          assembly mesh separately), which are merged into the same
         *          Geogram mesh: `vertex_offset` is the number of vertices the
         *          previous objects already created, so that the element
         *          indices of each object can be shifted to the merged mesh.
         */
        struct MPHTXT_Object {
            GEO::index_t vertex_offset = 0; // index of the first vertex in the mesh
            GEO::index_t nb_vertices = 0;   // number of vertices of the object
            GEO::index_t first_vertex = 0;  // lowest vertex index of the object
            GEO::index_t dimension = 3;     // space dimension (3 when `sdim` is missing)
        };

        /**
         * @brief Tests whether the current line is a comment.
         * @details get_fields() must have been called on the line. COMSOL
         *          starts its comments with '#', and `get_fields()` splits on
         *          the blanks, so the first field of a comment starts with
         *          '#' too (`# Type #0` gives the fields `#`, `Type` and
         *          `#0`).
         * @param[in] in the line reader, with the fields of a line extracted
         * @retval true if the line is a comment
         * @retval false otherwise
         */
        bool is_comment(const LineInput& in) {
            return in.nb_fields() != 0 && in.field(0)[0] == '#';
        }

        /**
         * @brief Tests whether the field \p i of the current line is a word.
         * @param[in] in the line reader, with the fields of a line extracted
         * @param[in] i the index of the field
         * @param[in] word the expected value of the field
         * @retval true if the line holds that word at that position
         * @retval false otherwise
         */
        bool field_is(const LineInput& in, GEO::index_t i, const char* word) {
            return i < in.nb_fields() && std::strcmp(in.field(i), word) == 0;
        }

        /**
         * @brief Reads the next line that holds at least one field.
         * @details LineInput::get_line() skips the empty lines, but a line
         *          made of blanks only is not empty and gives no field, so
         *          such lines are skipped here as well.
         * @param[in,out] in the line reader
         * @retval true if a line was read
         * @retval false if the end of the file was reached
         */
        bool read_next_line(LineInput& in) {
            while (in.get_line()) {
                in.get_fields();
                if (in.nb_fields() != 0)
                    return true;
            }
            return false;
        }

        /**
         * @brief Tests whether a field is a sequence of decimal digits.
         * @details Used to tell the `lowest mesh vertex index` field written
         *          by the older versions of the format (a single integer)
         *          from the first row of vertex coordinates (one coordinate
         *          per value of `sdim`, which may contain a decimal point or
         *          an exponent). The sign is not accepted: COMSOL never writes
         *          a negative vertex index.
         * @param[in] field the field
         * @retval true if the field is non-empty and holds digits only
         * @retval false otherwise
         */
        bool is_unsigned_integer(const char* field) {
            if (*field == '\0')
                return false;
            for (const char* c = field; *c != '\0'; ++c) {
                if (*c < '0' || *c > '9')
                    return false;
            }
            return true;
        }

        /**
         * @brief Tests whether the current line holds the number of vertices
         *        of a Mesh object.
         * @details The line reads `<nb vertices> # number of mesh vertices`.
         *          It is the only field of the format that is written with
         *          that comment, in both the version 4 layout (where the
         *          vertices follow the `sdim` field directly) and the version
         *          8 layout (where they follow the `# Mesh data` section
         *          header), and it cannot be confused with the
         *          `<nb> # number of element types` line that comes later.
         * @param[in] in the line reader, with the fields of a line extracted
         * @retval true if the line announces the mesh vertex coordinates
         * @retval false otherwise
         */
        bool is_vertex_count(const LineInput& in) {
            return !is_comment(in) && in.nb_fields() >= 6 &&
                   field_is(in, 1, "#") &&
                   field_is(in, 2, "number") &&
                   field_is(in, 3, "of") &&
                   field_is(in, 4, "mesh") &&
                   field_is(in, 5, "vertices");
        }

        /**
         * @brief Tests whether the current line is the `sdim` field of a Mesh
         *        object.
         * @details The line reads `<space dimension> # sdim`, and gives the
         *          number of coordinates of each vertex, 2 for a 2D mesh and
         *          3 for a 3D one.
         * @param[in] in the line reader, with the fields of a line extracted
         * @retval true if the line holds the space dimension
         * @retval false otherwise
         */
        bool is_space_dimension(const LineInput& in) {
            return !is_comment(in) && in.nb_fields() >= 3 &&
                   field_is(in, 1, "#") &&
                   field_is(in, 2, "sdim");
        }

        /**
         * @brief Tests whether the current line introduces an element type.
         * @details COMSOL writes `# Type #0`, `# Type #1`, ... in the
         *          documented version of the format, and a plain `# Type`
         *          without an index in some others; both are accepted. The
         *          `# Types` header at the beginning of the file does not
         *          match, since its second field is `Types`.
         * @param[in] in the line reader, with the fields of a line extracted
         * @retval true if an element type block follows
         * @retval false otherwise
         */
        bool is_element_type_header(const LineInput& in) {
            return is_comment(in) && field_is(in, 1, "Type");
        }

        /**
         * @brief Tests whether the current line introduces an object.
         * @details The line reads `# --------- Object <i> ----------`, and is
         *          followed by the fields of the object: the class of the
         *          object (`Mesh`, `Selection`, ...), its version and its
         *          space dimension for a Mesh.
         * @param[in] in the line reader, with the fields of a line extracted
         * @retval true if a new object follows
         * @retval false otherwise
         */
        bool is_object_header(const LineInput& in) {
            return is_comment(in) && field_is(in, 2, "Object");
        }

        /**
         * @brief Reads the vertex block that follows a number of vertices.
         * @details The block is a matrix of `d` coordinates per vertex stored
         *          row by row, where `d` is the space dimension given by the
         *          `sdim` field of the object; COMSOL writes one vertex per
         *          line. A 2D vertex gets a zero z coordinate, since the
         *          Geogram mesh is always 3D. The optional `lowest mesh vertex
         *          index` field of the older versions of the format sits
         *          between the number of vertices and the `# Mesh vertex
         *          coordinates` comment: it is read when the first line of the
         *          block holds a single integer, and an element index then has
         *          to be shifted by it.
         * @param[in,out] in the line reader, positioned on the number of vertices
         * @param[in,out] object the description of the current Mesh object
         * @param[in,out] mesh the mesh the vertices are created in
         * @retval true if the block could be read
         * @retval false otherwise, after an error has been logged
         */
        bool read_vertices(LineInput& in, MPHTXT_Object& object, GEO::Mesh& mesh) {
            const GEO::index_t nb_vertices = in.field_as_uint(0);
            const GEO::index_t dimension = object.dimension;
            if (dimension == 0 || dimension > 3) {
                LOG::ERROR("Line {} :Unsupported space dimension {}!", in.line_number(), dimension);
                return false;
            }

            object.vertex_offset = mesh.vertices.nb();
            object.nb_vertices = nb_vertices;
            object.first_vertex = 0;
            mesh.vertices.create_vertices(nb_vertices);

            bool coordinates_header_seen = false;
            for (GEO::index_t v = 0; v < nb_vertices; ) {
                if (!read_next_line(in)) {
                    LOG::ERROR(
                        "Line {} :Unexpected end of file, {} mesh vertices are missing!",
                        in.line_number(), nb_vertices - v);
                    return false;
                }

                if (is_comment(in)) {
                    // The "# Mesh vertex coordinates" header, and any comment
                    // in between.
                    coordinates_header_seen = true;
                    continue;
                }

                if (v == 0 && !coordinates_header_seen &&
                    is_unsigned_integer(in.field(0)) &&
                    (field_is(in, 1, "#") ||
                     (dimension > 1 && in.nb_fields() == 1))) {
                    // "lowest mesh vertex index" of the old versions, with
                    // its comment (`1 # lowest mesh vertex index`). Without
                    // the comment, a single field is the index too, unless
                    // the mesh is 1D, where a coordinate is a single field.
                    object.first_vertex = in.field_as_uint(0);
                    continue;
                }

                if (in.nb_fields() != dimension) {
                    LOG::ERROR(
                        "Line {} :Expect {} coordinates for a vertex of a {}D mesh, got {}!",
                        in.line_number(), dimension, dimension, in.nb_fields());
                    return false;
                }

                GEO::vec3& p = mesh.vertices.point(object.vertex_offset + v);
                p.x = in.field_as_double(0);
                p.y = dimension >= 2 ? in.field_as_double(1) : 0.0;
                p.z = dimension >= 3 ? in.field_as_double(2) : 0.0;
                ++v;
            }

            return true;
        }

        /**
         * @brief Reads the element block introduced by a `# Type` header.
         * @details The block reads
         *          - `<length> <name> # type name`,
         *          - `<vertices per element> # number of vertices per element`,
         *          - `<elements> # number of elements`,
         *          - then one line of vertex indices per element, preceded by
         *            an optional `# Elements` comment.
         *
         *          Only the tetrahedra are kept. A tetrahedron is recognised
         *          by its type name (`tet` for a first-order element, `tet2`
         *          for a second-order one) and keeps its 4 first vertices,
         *          which are the corners in the COMSOL numbering; the edge
         *          nodes of a `tet2` element are discarded. The other element
         *          types are only skipped, which is what makes their geometric
         *          entity indices, their parameter records and their up/down
         *          pairs disappear as well.
         * @param[in,out] in the line reader, positioned on the `# Type` header
         * @param[in,out] object the description of the current Mesh object
         * @param[in,out] mesh the mesh the tetrahedra are created in
         * @retval true if the block could be read
         * @retval false otherwise, after an error has been logged
         */
        bool read_elements(LineInput& in, const MPHTXT_Object& object, GEO::Mesh& mesh) {
            /* `<length> <name> # type name` */
            if (!read_next_line(in) || in.nb_fields() < 2) {
                LOG::ERROR("Line {} :Expect an element type name!", in.line_number());
                return false;
            }
            const GEO::index_t name_length = in.field_as_uint(0);
            const std::string type_name = in.field(1);
            if (name_length != type_name.size()) {
                LOG::WARN(
                    "Line {} :Element type name `{}` is announced with a length of {}!",
                    in.line_number(), type_name, name_length);
            }

            /* `<vertices per element> # number of vertices per element` */
            if (!read_next_line(in)) {
                LOG::ERROR(
                    "Line {} :Unexpected end of file, expected the number of vertices per element of `{}`!",
                    in.line_number(), type_name);
                return false;
            }
            const GEO::index_t nb_element_vertices = in.field_as_uint(0);

            /* `<elements> # number of elements` */
            if (!read_next_line(in)) {
                LOG::ERROR(
                    "Line {} :Unexpected end of file, expected the number of `{}` elements!",
                    in.line_number(), type_name);
                return false;
            }
            const GEO::index_t nb_elements = in.field_as_uint(0);

            const bool is_tet = type_name.compare(0, 3, "tet") == 0;
            if (is_tet && nb_element_vertices < 4) {
                LOG::ERROR(
                    "Line {} :A tetrahedron has 4 corners, element type `{}` is announced with {} vertices!",
                    in.line_number(), type_name, nb_element_vertices);
                return false;
            }
            if (is_tet && nb_element_vertices > 4) {
                LOG::WARN(
                    "Element type `{}` has {} vertices per element, only the 4 corners of each element are kept.",
                    type_name, nb_element_vertices);
            }

            const GEO::index_t first_tet = is_tet ? mesh.cells.create_tets(nb_elements) : 0;

            /* One line of `nb_element_vertices` indices per element */
            for (GEO::index_t e = 0; e < nb_elements; ) {
                if (!read_next_line(in)) {
                    LOG::ERROR(
                        "Line {} :Unexpected end of file, {} `{}` elements are missing!",
                        in.line_number(), nb_elements - e, type_name);
                    return false;
                }
                if (is_comment(in))
                    continue; // the "# Elements" header
                if (in.nb_fields() != nb_element_vertices) {
                    LOG::ERROR(
                        "Line {} :Expect {} vertex indices for a `{}` element, got {}!",
                        in.line_number(), nb_element_vertices, type_name, in.nb_fields());
                    return false;
                }

                if (is_tet) {
                    for (GEO::index_t lv = 0; lv < 4; ++lv) {
                        const GEO::index_t v = in.field_as_uint(lv);
                        if (v < object.first_vertex ||
                            v - object.first_vertex >= object.nb_vertices) {
                            LOG::ERROR(
                                "Line {} :Element `{}` references vertex {}, but the mesh numbers its vertices in [{}, {})!",
                                in.line_number(), type_name, v, object.first_vertex,
                                object.first_vertex + object.nb_vertices);
                            return false;
                        }
                        mesh.cells.set_vertex(
                            first_tet + e, lv,
                            object.vertex_offset + (v - object.first_vertex));
                    }
                }

                ++e;
            }

            return true;
        }
    }

    bool MPHTXT_IOHandler::load(
        const std::string& filename,
        GEO::Mesh& mesh,
        const GEO::MeshIOFlags& ioflags
        ) {
        mesh.clear();
        mesh.vertices.set_dimension(3);

        LineInput in(filename);
        if (!in.OK()) {
            LOG::ERROR("Cannot load file `{}`!", filename);
            return false;
        }

        MPHTXT_Object object;
        try {
            while (read_next_line(in)) {
                if (is_comment(in)) {
                    if (is_object_header(in)) {
                        // The Mesh objects are merged, but the description of
                        // the previous one (its dimension, its vertices) does
                        // not apply to the next one; its `sdim` field follows.
                        object = MPHTXT_Object();
                    }
                    else if (is_element_type_header(in) && object.nb_vertices != 0) {
                        // An element type can only follow the vertex block of
                        // its object, so a `# Type` comment of another section
                        // (the geometric model of a Mesh object can be included
                        // in the file) is not mistaken for one.
                        if (!read_elements(in, object, mesh)) {
                            mesh.clear();
                            return false;
                        }
                    }
                }
                else if (is_vertex_count(in)) {
                    if (!read_vertices(in, object, mesh)) {
                        mesh.clear();
                        return false;
                    }
                }
                else if (is_space_dimension(in)) {
                    object.dimension = in.field_as_uint(0);
                }
            }
        }
        catch (const std::logic_error& e) {
            LOG::ERROR("Invalid file `{}` at line {}: {}", filename, in.line_number(), e.what());
            mesh.clear();
            return false;
        }

        if (mesh.vertices.nb() == 0) {
            LOG::ERROR("File `{}` holds no mesh vertices!", filename);
            mesh.clear();
            return false;
        }

        if (mesh.cells.nb() == 0) {
            LOG::WARN("File `{}` holds no tetrahedra, only the vertices were loaded!", filename);
            return true;
        }

        mesh.cells.connect();

        return true;
    }
}
