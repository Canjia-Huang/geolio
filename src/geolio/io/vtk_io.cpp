//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/11/15.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "vtk_io.h"

#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <geogram/basic/attributes.h>
#include <geogram/basic/geometry.h>
#include <geogram/basic/string.h>
#include <geogram/mesh/mesh.h>

#include "line_stream.h"
#include <geolio/common/log.h>

namespace geolio
{
    namespace
    {
        using GEO::index_t;

        /**
         * @brief Cell types of the legacy VTK file format.
         * @details Only the linear cell types, i.e. the ones that have a
         *          counterpart in GEO::Mesh, are listed here.
         * @see https://vtk.org/doc/nightly/html/vtkCellType_8h.html
         */
        enum VTKCellType {
            VTK_VERTEX = 1,
            VTK_LINE = 3,
            VTK_TRIANGLE = 5,
            VTK_TRIANGLE_STRIP = 6,
            VTK_POLYGON = 7,
            VTK_PIXEL = 8,
            VTK_QUAD = 9,
            VTK_TETRA = 10,
            VTK_VOXEL = 11,
            VTK_HEXAHEDRON = 12,
            VTK_WEDGE = 13,
            VTK_PYRAMID = 14
        };

        /**
         * @brief Scalar types of the data arrays of a legacy VTK file.
         * @details The integer types are all handled the same way: their values
         *          are converted to GEO::index_t when they are used as cell
         *          connectivity, and to Numeric::int32 when they are stored as an
         *          attribute.
         */
        enum VTKScalarType {
            VTK_FLOAT32,
            VTK_FLOAT64,
            VTK_INT8,
            VTK_UINT8,
            VTK_INT16,
            VTK_UINT16,
            VTK_INT32,
            VTK_UINT32,
            VTK_INT64,
            VTK_UINT64
        };

        /**
         * @brief Tests whether a VTK scalar type is an integer type.
         */
        bool is_integer_type(VTKScalarType type) {
            return type >= VTK_INT8;
        }

        /**
         * @brief Converts a VTK data type token to a VTKScalarType.
         * @param[in] token the data type token, e.g. `float` or `vtktypeint64`
         * @param[out] type the converted scalar type
         * @return true if the data type is supported, false otherwise
         */
        bool vtk_scalar_type(const std::string& token, VTKScalarType& type) {
            // Both the historical VTK names (`float`, `unsigned_int`, ...) and the
            // names used since VTK 5.1 (`vtktypefloat32`, `vtktypeint64`, ...) are
            // accepted.
            static const std::pair<const char*, VTKScalarType> TYPE_NAMES[] = {
                {"float", VTK_FLOAT32},
                {"vtkFloat32", VTK_FLOAT32},
                {"vtktypefloat32", VTK_FLOAT32},
                {"double", VTK_FLOAT64},
                {"vtkFloat64", VTK_FLOAT64},
                {"vtktypefloat64", VTK_FLOAT64},
                {"char", VTK_INT8},
                {"signed_char", VTK_INT8},
                {"vtkChar", VTK_INT8},
                {"unsigned_char", VTK_UINT8},
                {"vtkUnsignedChar", VTK_UINT8},
                {"short", VTK_INT16},
                {"vtkShort", VTK_INT16},
                {"unsigned_short", VTK_UINT16},
                {"vtkUnsignedShort", VTK_UINT16},
                {"int", VTK_INT32},
                {"vtkInt", VTK_INT32},
                {"vtktypeint32", VTK_INT32},
                {"unsigned_int", VTK_UINT32},
                {"vtkUnsignedInt", VTK_UINT32},
                {"vtktypeuint32", VTK_UINT32},
                {"long", VTK_INT64},
                {"vtkLong", VTK_INT64},
                {"vtkIdType", VTK_INT64},
                {"vtktypeint64", VTK_INT64},
                {"unsigned_long", VTK_UINT64},
                {"vtkUnsignedLong", VTK_UINT64},
                {"vtktypeuint64", VTK_UINT64}
            };
            for (const auto& [name, scalar_type] : TYPE_NAMES) {
                if (token == name) {
                    type = scalar_type;
                    return true;
                }
            }
            return false;
        }

        /**
         * @brief Returns the number of vertices of a VTK cell type.
         * @return the number of vertices, or 0 for the cell types whose number of
         *         vertices varies (polygons, triangle strips, vertex lists) and
         *         for the cell types that are not supported.
         */
        index_t vtk_cell_nb_vertices(index_t type) {
            switch (type) {
            case VTK_VERTEX: return 1;
            case VTK_LINE: return 2;
            case VTK_TRIANGLE: return 3;
            case VTK_PIXEL:
            case VTK_QUAD:
            case VTK_TETRA: return 4;
            case VTK_PYRAMID: return 5;
            case VTK_WEDGE: return 6;
            case VTK_VOXEL:
            case VTK_HEXAHEDRON: return 8;
            default: return 0;
            }
        }

        /**
         * @brief Tests whether a VTK cell type has a counterpart in GEO::Mesh.
         * @details The supported cell types become an edge (VTK lines), a facet
         *          (triangles, quads, pixels, polygons and triangle strips) or
         *          a cell (tetrahedra, voxels, hexahedra, wedges and pyramids).
         *          VTK vertex cells are supported too: they need no conversion
         *          since the vertices already exist in the mesh.
         */
        bool is_supported_cell_type(index_t type) {
            switch (type) {
            case VTK_VERTEX:
            case VTK_LINE:
            case VTK_TRIANGLE:
            case VTK_TRIANGLE_STRIP:
            case VTK_POLYGON:
            case VTK_PIXEL:
            case VTK_QUAD:
            case VTK_TETRA:
            case VTK_VOXEL:
            case VTK_HEXAHEDRON:
            case VTK_WEDGE:
            case VTK_PYRAMID:
                return true;
            default:
                return false;
            }
        }

        /**
         * @brief Reads the tokens of a legacy VTK ASCII file.
         * @details The legacy VTK format only requires its values to be separated
         *          by white space, so a data array may be split across lines at
         *          any position. The class therefore exposes a token stream
         *          instead of a line stream: the next line is fetched
         *          automatically whenever the fields of the current line are
         *          exhausted.
         */
        class TokenStream {
        public:
            explicit TokenStream(geolio::LineInput& in) : in_(in) {
            }

            /**
             * @brief Advances to the next token.
             * @return true if a token could be read, false at the end of the file
             */
            bool next() {
                if (!pushed_back_.empty()) {
                    token_.swap(pushed_back_);
                    pushed_back_.clear();
                    return true;
                }

                while (true) {
                    if (has_line_ && field_+1 < in_.nb_fields()) {
                        ++field_;
                        token_ = in_.field(field_);
                        return true;
                    }

                    if (!in_.get_line()) {
                        has_line_ = false;
                        token_.clear();
                        return false;
                    }

                    in_.get_fields();
                    if (in_.nb_fields() == 0)
                        continue; // blank line

                    has_line_ = true;
                    field_ = 0;
                    token_ = in_.field(0);
                    return true;
                }
            }

            /**
             * @brief Puts the current token back into the stream.
             * @details The next call to next() returns the same token again.
             */
            void put_back() {
                pushed_back_ = token_;
            }

            /**
             * @brief Tests whether the current token is the given keyword.
             */
            [[nodiscard]] bool matches(const char* keyword) const {
                return token_ == keyword;
            }

            /**
             * @brief Returns the current token.
             */
            [[nodiscard]] const std::string& token() const {
                return token_;
            }

            /**
             * @brief Returns the number of the line being read, for error messages.
             */
            [[nodiscard]] size_t line_number() const {
                return in_.line_number();
            }

            /**
             * @brief Converts the current token to a floating point value.
             * @exception std::logic_error if the token is not a number
             */
            [[nodiscard]] double as_double() const {
                double result = 0;
                if (!GEO::String::from_string(token_.c_str(), result))
                    throw std::logic_error("cannot read `" + token_ + "` as a floating point value");
                return result;
            }

            /**
             * @brief Converts the current token to an unsigned integer.
             * @exception std::logic_error if the token is not an unsigned integer
             */
            [[nodiscard]] index_t as_uint() const {
                index_t result = 0;
                if (!GEO::String::from_string(token_.c_str(), result))
                    throw std::logic_error("cannot read `" + token_ + "` as an unsigned integer");
                return result;
            }

        private:
            geolio::LineInput& in_;
            std::string token_;
            std::string pushed_back_;
            bool has_line_ = false;
            index_t field_ = 0;
        };

        /**
         * @brief Reports an unexpected end of file.
         * @param[in] line_number the number of the last line that was read
         * @return always false, so that it can be returned directly
         */
        bool error_unexpected_end_of_file(size_t line_number) {
            LOG::ERROR("Line {} :Unexpected end of file!", line_number);
            return false;
        }

        /**
         * @brief Reads a data type token.
         * @param[out] type the data type of the current token
         * @return false if no token could be read or if the data type is unknown
         */
        bool read_scalar_type(TokenStream& in, VTKScalarType& type) {
            if (!in.next())
                return error_unexpected_end_of_file(in.line_number());
            if (!vtk_scalar_type(in.token(), type)) {
                LOG::ERROR("Line {} :Unknown data type `{}`!", in.line_number(), in.token());
                return false;
            }
            return true;
        }

        /**
         * @brief Cells read from a legacy VTK file, in file order.
         * @details Kept in a compressed layout (index pointer + connectivity)
         *          until the whole file has been parsed, because the type of the
         *          cells of a `CELLS` block is given by the `CELL_TYPES` block
         *          that usually follows it.
         */
        struct VTKCells {
            /** Index of the first vertex of each cell, plus a sentinel. */
            std::vector<index_t> ptr{0};
            /** Vertex indices of all the cells. */
            std::vector<index_t> conn;
            /** VTK type of each cell. */
            std::vector<index_t> types;

            [[nodiscard]] index_t nb() const {
                return static_cast<index_t>(types.size());
            }

            [[nodiscard]] index_t nb_vertices(index_t c) const {
                return ptr[c+1]-ptr[c];
            }

            [[nodiscard]] const index_t* vertices(index_t c) const {
                return conn.data()+ptr[c];
            }

            /**
             * @brief Appends a cell.
             * @param[in] type VTK type of the cell
             * @param[in] vertices vertex indices of the cell
             * @param[in] nb_vertices number of vertices of the cell
             */
            void push_cell(index_t type, const index_t* vertices, index_t nb_vertices) {
                conn.insert(conn.end(), vertices, vertices+nb_vertices);
                ptr.push_back(static_cast<index_t>(conn.size()));
                types.push_back(type);
            }
        };

        /**
         * @brief Destination of the arrays of a `POINT_DATA` / `CELL_DATA` block.
         */
        struct DataTarget {
            /**
             * @brief Attributes receiving the arrays, nullptr to discard them.
             * @details The arrays are discarded when the VTK cells became
             *          elements of several kinds (e.g. facets and cells), since
             *          the arrays of a single VTK block cannot be attached to
             *          only one of these kinds.
             */
            GEO::AttributesManager* attributes = nullptr;
            /** Number of tuples of the arrays of the block. */
            index_t nb_tuples = 0;
        };

        /**
         * @brief Reads a cell block (`CELLS`, `POLYGONS`, `LINES`, ...).
         * @details Two layouts of the legacy format exist. The historical one
         *          stores, for each cell, its number of vertices followed by its
         *          vertex indices:
         *          \code
         *          CELLS 2 9
         *          3 0 1 2
         *          4 0 1 3 2
         *          \endcode
         *          The layout introduced by VTK 5.1 stores the offsets of the
         *          cells and their connectivity separately:
         *          \code
         *          CELLS 2 12
         *          OFFSETS vtktypeint64
         *          0 3 7
         *          CONNECTIVITY vtktypeint64
         *          0 1 2 0 1 3 2
         *          \endcode
         * @param[in,out] in the token stream, positioned on the block keyword
         * @param[in] implied_type the VTK type implied by the block keyword, 0 for
         *            `CELLS` whose cells take their type from `CELL_TYPES`
         * @param[in,out] cells the cells read from the file
         * @return false on malformed input
         */
        bool read_cell_block(TokenStream& in, index_t implied_type, VTKCells& cells) {
            if (!in.next())
                return error_unexpected_end_of_file(in.line_number());
            const index_t nb_cells = in.as_uint();

            // The second number is the number of connectivity entries in the
            // historical layout, and is not used by the VTK 5.1 one.
            if (!in.next())
                return error_unexpected_end_of_file(in.line_number());
            const index_t size = in.as_uint();

            if (!in.next())
                return error_unexpected_end_of_file(in.line_number());

            if (!in.matches("OFFSETS")) {
                // Historical layout: `nb_vertices v0 ... v(nb_vertices-1)` records.
                // The current token is the number of vertices of the first cell.
                in.put_back();

                std::vector<index_t> cell_vertices;
                index_t nb_values = 0;
                for (index_t c = 0; c < nb_cells; ++c) {
                    if (!in.next())
                        return error_unexpected_end_of_file(in.line_number());
                    const index_t nb_vertices = in.as_uint();

                    cell_vertices.resize(nb_vertices);
                    for (index_t lv = 0; lv < nb_vertices; ++lv) {
                        if (!in.next())
                            return error_unexpected_end_of_file(in.line_number());
                        cell_vertices[lv] = in.as_uint();
                    }

                    cells.push_cell(implied_type, cell_vertices.data(), nb_vertices);
                    nb_values += nb_vertices+1;
                }

                if (nb_values != size) {
                    LOG::ERROR(
                        "Line {} :Invalid cell block, expected {} values but read {}!",
                        in.line_number(), size, nb_values);
                    return false;
                }
                return true;
            }

            // Layout of VTK 5.1: the size that follows the number of cells is not
            // used, the number of connectivity entries is given by the last offset.
            VTKScalarType offsets_type;
            if (!read_scalar_type(in, offsets_type))
                return false;

            std::vector<index_t> offsets(nb_cells+1);
            for (index_t c = 0; c <= nb_cells; ++c) {
                if (!in.next())
                    return error_unexpected_end_of_file(in.line_number());
                offsets[c] = in.as_uint();
            }

            if (offsets[0] != 0) {
                LOG::ERROR("Line {} :Invalid offsets, the first one should be 0!", in.line_number());
                return false;
            }

            if (!in.next() || !in.matches("CONNECTIVITY")) {
                LOG::ERROR("Line {} :Invalid cell block, expect `CONNECTIVITY`!", in.line_number());
                return false;
            }

            VTKScalarType connectivity_type;
            if (!read_scalar_type(in, connectivity_type))
                return false;

            for (index_t c = 0; c < nb_cells; ++c) {
                if (offsets[c+1] < offsets[c]) {
                    LOG::ERROR("Line {} :Invalid offsets, they are not sorted!", in.line_number());
                    return false;
                }
            }

            std::vector<index_t> conn(offsets[nb_cells]);
            for (index_t& v : conn) {
                if (!in.next())
                    return error_unexpected_end_of_file(in.line_number());
                v = in.as_uint();
            }

            for (index_t c = 0; c < nb_cells; ++c) {
                if (offsets[c+1] < offsets[c] || offsets[c+1] > offsets[nb_cells]) {
                    LOG::ERROR("Line {} :Invalid offset {} for cell {}!", in.line_number(), offsets[c+1], c);
                    return false;
                }
                cells.push_cell(
                    implied_type,
                    conn.data()+offsets[c],
                    offsets[c+1]-offsets[c]);
            }
            return true;
        }

        /**
         * @brief Reads and discards the values of a data array.
         * @param[in] nb_values number of values to discard
         * @return false on unexpected end of file
         */
        bool skip_values(TokenStream& in, index_t nb_values) {
            for (index_t i = 0; i < nb_values; ++i) {
                if (!in.next())
                    return error_unexpected_end_of_file(in.line_number());
            }
            return true;
        }

        /**
         * @brief Reads the values of a data array and stores them into an
         *        attribute of type `T`.
         * @tparam T the attribute type, either a scalar type or a GEO::vecng
         * @param[in,out] in the token stream, positioned on the first value
         * @param[in] attributes_manager the manager that receives the attribute
         * @param[in] name name of the attribute
         * @param[in] nb_tuples number of items of the array
         * @return false if the values could not be read or stored
         */
        template <class T>
        bool read_attribute(
            TokenStream& in,
            GEO::AttributesManager& attributes_manager,
            const std::string& name,
            index_t nb_tuples
            ) {
            GEO::Attribute<T> attribute(attributes_manager, name);
            if (!attribute.is_bound() || attribute.size() != nb_tuples) {
                // The name may already be used by an attribute of another
                // type (e.g. the same array name twice in the file).
                LOG::WARN(
                    "Line {} :Cannot store `{}` as an attribute of {} elements, skip it.",
                    in.line_number(), name, nb_tuples);

                index_t nb_values = nb_tuples;
                if constexpr (!std::is_arithmetic_v<T>)
                    nb_values *= T::dim;
                return skip_values(in, nb_values);
            }

            for (index_t i = 0; i < nb_tuples; ++i) {
                if constexpr (std::is_arithmetic_v<T>) {
                    if (!in.next())
                        return error_unexpected_end_of_file(in.line_number());
                    attribute[i] = static_cast<T>(in.as_double());
                }
                else {
                    T value;
                    for (index_t c = 0; c < T::dim; ++c) {
                        if (!in.next())
                            return error_unexpected_end_of_file(in.line_number());
                        value[c] = static_cast<typename T::value_type>(in.as_double());
                    }
                    attribute[i] = value;
                }
            }
            return true;
        }

        /**
         * @brief Reads the values of a data array whose items have `DIM`
         *        components and stores them into a vector attribute.
         */
        template <index_t DIM>
        bool read_vector_attribute(
            TokenStream& in,
            GEO::AttributesManager& attributes_manager,
            const std::string& name,
            bool single_precision,
            index_t nb_tuples
            ) {
            if (single_precision) {
                return read_attribute<GEO::vecng<DIM, float>>(
                    in, attributes_manager, name, nb_tuples);
            }
            return read_attribute<GEO::vecng<DIM, double>>(
                in, attributes_manager, name, nb_tuples);
        }

        /**
         * @brief Reads a VTK data array and stores it into an attribute.
         * @details The attribute type matches both the number of components and
         *          the scalar type of the array: `float` arrays are stored as
         *          `float` (or `vec2f`, `vec3f`, `vec4f`), `double` arrays as
         *          `double` (or `vec2`, `vec3`, `vec4`) and integer arrays as
         *          `Numeric::int32` (or, when they have several components, as
         *          `vec2`, `vec3`, `vec4`).
         * @param[in] target the elements receiving the array
         * @param[in] nb_tuples number of items of the array
         */
        bool read_data_array(
            TokenStream& in,
            const DataTarget& target,
            const std::string& name,
            VTKScalarType scalar_type,
            index_t nb_components,
            index_t nb_tuples
            ) {
            if (target.attributes == nullptr || target.nb_tuples != nb_tuples) {
                if (target.attributes != nullptr) {
                    LOG::WARN(
                        "Line {} :`{}` has {} tuples but the elements have {}; skip it.",
                        in.line_number(), name, nb_tuples, target.nb_tuples);
                }
                return skip_values(in, nb_components*nb_tuples);
            }

            switch (nb_components) {
            case 1:
                if (is_integer_type(scalar_type)) {
                    return read_attribute<GEO::Numeric::int32>(
                        in, *target.attributes, name, nb_tuples);
                }
                if (scalar_type == VTK_FLOAT32) {
                    return read_attribute<float>(
                        in, *target.attributes, name, nb_tuples);
                }
                return read_attribute<double>(
                    in, *target.attributes, name, nb_tuples);

            case 2:
                return read_vector_attribute<2>(
                    in, *target.attributes, name, scalar_type == VTK_FLOAT32, nb_tuples);

            case 3:
                return read_vector_attribute<3>(
                    in, *target.attributes, name, scalar_type == VTK_FLOAT32, nb_tuples);

            case 4:
                return read_vector_attribute<4>(
                    in, *target.attributes, name, scalar_type == VTK_FLOAT32, nb_tuples);

            default:
                LOG::WARN(
                    "Line {} :Storing `{}` with {} components is not supported; skip it.",
                    in.line_number(), name, nb_components);
                return skip_values(in, nb_components*nb_tuples);
            }
        }

        /**
         * @brief Converts the cells read from a legacy VTK file to Geogram elements.
         * @details Polygonal cells become facets, polyhedral cells become cells
         *          and line cells become edges. VTK vertex cells need no
         *          conversion, the vertices already exist in \p mesh.
         * @param[in,out] mesh the mesh receiving the elements
         * @param[in] cells the cells read from the file
         * @return false on malformed input
         */
        bool create_elements(
            GEO::Mesh& mesh,
            const VTKCells& cells
            ) {
            // == Check the input ==================================================
            std::vector<std::pair<index_t, index_t>> unsupported_cells; // type, count
            for (index_t c = 0; c < cells.nb(); ++c) {
                const index_t type = cells.types[c];
                if (type == 0) {
                    LOG::ERROR("Cell {} has no type, the file has no `CELL_TYPES` block!", c);
                    return false;
                }

                if (!is_supported_cell_type(type)) {
                    auto found = unsupported_cells.begin();
                    while (found != unsupported_cells.end() && found->first != type)
                        ++found;
                    if (found == unsupported_cells.end())
                        unsupported_cells.emplace_back(type, 1);
                    else
                        ++found->second;
                    continue;
                }

                if (const index_t expected = vtk_cell_nb_vertices(type);
                    expected != 0 && cells.nb_vertices(c) != expected) {
                    LOG::ERROR(
                        "Cell {} of type {} has {} vertices instead of {}!",
                        c, type, cells.nb_vertices(c), expected);
                    return false;
                }

                for (index_t lv = 0, lv_end = cells.nb_vertices(c); lv < lv_end; ++lv) {
                    if (const index_t v = cells.vertices(c)[lv]; v >= mesh.vertices.nb()) {
                        LOG::ERROR(
                            "Cell {} refers to vertex {} but the file has {} vertices!",
                            c, v, mesh.vertices.nb());
                        return false;
                    }
                }
            }

            for (const auto& [type, count] : unsupported_cells) {
                LOG::WARN(
                    "{} cells of VTK type {} are not supported and are ignored.",
                    count, type);
            }

            // == Create the elements ==============================================
            for (index_t c = 0; c < cells.nb(); ++c) {
                const index_t type = cells.types[c];
                const index_t nb_vertices = cells.nb_vertices(c);
                const index_t* v = cells.vertices(c);

                switch (type) {
                case VTK_VERTEX:
                    // The vertex already exists, there is nothing to create.
                    break;

                case VTK_LINE:
                    mesh.edges.create_edge(v[0], v[1]);
                    break;

                case VTK_TRIANGLE:
                    mesh.facets.create_triangle(v[0], v[1], v[2]);
                    break;

                case VTK_QUAD:
                    mesh.facets.create_quad(v[0], v[1], v[2], v[3]);
                    break;

                case VTK_PIXEL:
                    // VTK enumerates the corners of a pixel in the order 0, 1, 3, 2.
                    mesh.facets.create_quad(v[0], v[1], v[3], v[2]);
                    break;

                case VTK_POLYGON: {
                    // A POLYDATA file may store triangles and quads in a POLYGONS
                    // block without a CELL_TYPES block: create the matching
                    // simplices instead of generic polygons.
                    if (nb_vertices == 3) {
                        mesh.facets.create_triangle(v[0], v[1], v[2]);
                        break;
                    }
                    if (nb_vertices == 4) {
                        mesh.facets.create_quad(v[0], v[1], v[2], v[3]);
                        break;
                    }

                    const index_t f = mesh.facets.create_polygon(nb_vertices);
                    for (index_t lv = 0; lv < nb_vertices; ++lv)
                        mesh.facets.set_vertex(f, lv, v[lv]);
                    break;
                }

                case VTK_TRIANGLE_STRIP:
                    // A strip of n vertices is split into n-2 triangles whose
                    // orientation alternates.
                    for (index_t k = 0; k+2 < nb_vertices; ++k) {
                        if (k%2 == 0)
                            mesh.facets.create_triangle(v[k], v[k+1], v[k+2]);
                        else
                            mesh.facets.create_triangle(v[k+1], v[k], v[k+2]);
                    }
                    break;

                case VTK_TETRA:
                    mesh.cells.create_tet(v[0], v[1], v[2], v[3]);
                    break;

                case VTK_HEXAHEDRON: {
                    // Geogram enumerates the bottom quad of a hexahedron as
                    // 0, 1, 3, 2 and its top quad as 4, 5, 7, 6, while VTK
                    // enumerates them as 0, 1, 2, 3 and 4, 5, 6, 7: the two
                    // conventions differ by the swaps 2 <-> 3 and 6 <-> 7.
                    constexpr index_t VTK_TO_GEOGRAM[8] = {0, 1, 3, 2, 4, 5, 7, 6};
                    index_t h[8];
                    for (index_t lv = 0; lv < 8; ++lv)
                        h[lv] = v[VTK_TO_GEOGRAM[lv]];
                    mesh.cells.create_hex(
                        h[0], h[1], h[2], h[3], h[4], h[5], h[6], h[7]);
                    break;
                }

                case VTK_VOXEL:
                    // VTK voxels are already enumerated in the Geogram order.
                    mesh.cells.create_hex(
                        v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7]);
                    break;

                case VTK_WEDGE:
                    // VTK wedges and Geogram prisms share the same enumeration.
                    mesh.cells.create_prism(
                        v[0], v[1], v[2], v[3], v[4], v[5]);
                    break;

                case VTK_PYRAMID:
                    // VTK pyramids and Geogram pyramids share the same enumeration.
                    mesh.cells.create_pyramid(
                        v[0], v[1], v[2], v[3], v[4]);
                    break;

                default:
                    // Unsupported cell type, already reported.
                    break;
                }
            }
            return true;
        }

        /**
         * @brief Finds the elements that receive the arrays of a `CELL_DATA` block.
         * @details A legacy VTK file stores the arrays of all its cells in a single
         *          block, whereas Geogram splits cells into edges, facets and
         *          cells. The arrays can therefore only be attached when all the
         *          VTK cells became elements of a single kind.
         * @param[in] mesh the mesh that received the cells
         * @param[in] nb_tuples number of tuples declared by the block
         * @return the target receiving the arrays, empty if they must be discarded
         */
        DataTarget find_cell_data_target(GEO::Mesh& mesh, index_t nb_tuples) {
            DataTarget target;
            target.nb_tuples = nb_tuples;

            GEO::AttributesManager* attributes[] = {
                &mesh.edges.attributes(),
                &mesh.facets.attributes(),
                &mesh.cells.attributes()
            };
            const index_t nb_elements[] = {
                mesh.edges.nb(),
                mesh.facets.nb(),
                mesh.cells.nb()
            };

            index_t nb_kinds = 0;
            index_t kind = 0;
            for (index_t k = 0; k < 3; ++k) {
                if (nb_elements[k] == 0)
                    continue;
                ++nb_kinds;
                kind = k;
            }

            if (nb_kinds != 1) {
                LOG::WARN(
                    "CELL_DATA is ignored: {} kinds of elements cannot share the same arrays!",
                    nb_kinds);
                return target;
            }

            if (nb_elements[kind] != nb_tuples) {
                LOG::WARN(
                    "CELL_DATA is ignored: it has {} tuples but the elements are {}!",
                    nb_tuples, nb_elements[kind]);
                return target;
            }

            target.attributes = attributes[kind];
            return target;
        }
    }

    bool VTK_IOHandler::load(
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

        /* == Header ========================================================== */
        // Line 1: `# vtk DataFile Version x.y`
        if (!in.get_line()) {
            LOG::ERROR("File `{}` is empty!", filename);
            return false;
        }
        in.get_fields();
        if (in.nb_fields() < 5 ||
            !in.field_matches(0, "#") || !in.field_matches(1, "vtk") ||
            !in.field_matches(2, "DataFile") || !in.field_matches(3, "Version")
        ) {
            LOG::ERROR("Line {} :Invalid header, expect `# vtk DataFile Version x.y`!", in.line_number());
            return false;
        }

        // Line 2 is the title, a free text that is ignored. LineInput skips
        // empty lines, so when the title is empty the current line is already
        // the encoding line.
        // Line 3: `ASCII` or `BINARY`.
        const auto is_encoding_line = [](const LineInput& line_input) {
            return line_input.nb_fields() == 1 &&
                (line_input.field_matches(0, "ASCII") || line_input.field_matches(0, "BINARY"));
        };

        if (!in.get_line())
            return error_unexpected_end_of_file(in.line_number());
        in.get_fields();
        if (!is_encoding_line(in)) {
            // The current line is the title, the encoding line follows it.
            if (!in.get_line())
                return error_unexpected_end_of_file(in.line_number());
            in.get_fields();
            if (!is_encoding_line(in)) {
                LOG::ERROR("Line {} :Invalid encoding, expect `ASCII` or `BINARY`!", in.line_number());
                return false;
            }
        }
        if (!in.field_matches(0, "ASCII")) {
            LOG::ERROR("Line {} :Binary VTK files are not supported, only ASCII ones are!", in.line_number());
            return false;
        }

        // Line 4: `DATASET <type>`.
        if (!in.get_line())
            return error_unexpected_end_of_file(in.line_number());
        in.get_fields();
        if (in.nb_fields() != 2 || !in.field_matches(0, "DATASET")) {
            LOG::ERROR("Line {} :Invalid line, expect `DATASET <type>`!", in.line_number());
            return false;
        }
        if (!in.field_matches(1, "POLYDATA") &&
            !in.field_matches(1, "UNSTRUCTURED_GRID")
        ) {
            LOG::ERROR(
                "Line {} :Unsupported dataset `{}`, expect `POLYDATA` or `UNSTRUCTURED_GRID`!",
                in.line_number(), in.field(1));
            return false;
        }

        /* == Sections ======================================================== */
        TokenStream tokens(in);
        VTKCells cells;
        DataTarget data_target;
        bool elements_created = false;
        const auto ensure_elements = [&]() -> bool {
            if (elements_created)
                return true;
            if (!create_elements(mesh, cells))
                return false;
            elements_created = true;
            return true;
        };

        try {
            while (tokens.next()) {
                if (tokens.matches("POINTS")) { // == POINTS ===============================================
                    if (!tokens.next())
                        return error_unexpected_end_of_file(tokens.line_number());
                    const index_t nb_vertices = tokens.as_uint();

                    VTKScalarType scalar_type;
                    if (!read_scalar_type(tokens, scalar_type))
                        return false;

                    mesh.vertices.create_vertices(nb_vertices);
                    for (index_t v = 0; v < nb_vertices; ++v) {
                        auto& p = mesh.vertices.point(v);
                        for (index_t d = 0; d < 3; ++d) {
                            if (!tokens.next())
                                return error_unexpected_end_of_file(tokens.line_number());
                            p[d] = tokens.as_double();
                        }
                    }
                }
                else if (tokens.matches("CELLS") || tokens.matches("VERTICES") ||
                         tokens.matches("LINES") || tokens.matches("POLYGONS") ||
                         tokens.matches("TRIANGLE_STRIPS")) { // == CELLS ==========================
                    // `CELLS` cells get their type from `CELL_TYPES`, while
                    // the blocks of a POLYDATA file imply it.
                    index_t implied_type = 0;
                    if (tokens.matches("VERTICES"))
                        implied_type = VTK_VERTEX;
                    else if (tokens.matches("LINES"))
                        implied_type = VTK_LINE;
                    else if (tokens.matches("POLYGONS"))
                        implied_type = VTK_POLYGON;
                    else if (tokens.matches("TRIANGLE_STRIPS"))
                        implied_type = VTK_TRIANGLE_STRIP;

                    if (!read_cell_block(tokens, implied_type, cells))
                        return false;
                }
                else if (tokens.matches("CELL_TYPES")) { // == CELL_TYPES ====================================
                    if (!tokens.next())
                        return error_unexpected_end_of_file(tokens.line_number());
                    const index_t nb_cell_types = tokens.as_uint();
                    if (nb_cell_types != cells.nb()) {
                        LOG::ERROR(
                            "Line {} :{} cell types for {} cells!",
                            tokens.line_number(), nb_cell_types, cells.nb());
                        return false;
                    }

                    for (index_t c = 0; c < nb_cell_types; ++c) {
                        if (!tokens.next())
                            return error_unexpected_end_of_file(tokens.line_number());
                        // CELL_TYPES is authoritative, even for the cells of a
                        // POLYGONS block whose type is implied.
                        cells.types[c] = tokens.as_uint();
                    }
                }
                else if (tokens.matches("POINT_DATA")) { // == POINT_DATA ====================================
                    if (!tokens.next())
                        return error_unexpected_end_of_file(tokens.line_number());
                    const index_t nb_tuples = tokens.as_uint();
                    data_target = {};
                    data_target.nb_tuples = nb_tuples;
                    if (nb_tuples != mesh.vertices.nb()) {
                        LOG::WARN(
                            "Line {} :POINT_DATA has {} tuples but the mesh has {} vertices, the arrays are ignored!",
                            tokens.line_number(), nb_tuples, mesh.vertices.nb());
                    }
                    else {
                        data_target.attributes = &mesh.vertices.attributes();
                    }
                }
                else if (tokens.matches("CELL_DATA")) { // == CELL_DATA =====================================
                    if (!tokens.next())
                        return error_unexpected_end_of_file(tokens.line_number());
                    const index_t nb_tuples = tokens.as_uint();
                    if (!ensure_elements())
                        return false;
                    data_target = find_cell_data_target(mesh, nb_tuples);
                }
                else if (tokens.matches("SCALARS")) { // == SCALARS =======================================
                    // SCALARS name type [nb_components]
                    // LOOKUP_TABLE name
                    if (!tokens.next())
                        return error_unexpected_end_of_file(tokens.line_number());
                    const std::string name = tokens.token();

                    VTKScalarType scalar_type;
                    if (!read_scalar_type(tokens, scalar_type))
                        return false;

                    index_t nb_components = 1;
                    if (!tokens.next())
                        return error_unexpected_end_of_file(tokens.line_number());
                    if (!tokens.matches("LOOKUP_TABLE")) {
                        nb_components = tokens.as_uint();
                        if (!tokens.next())
                            return error_unexpected_end_of_file(tokens.line_number());
                        if (!tokens.matches("LOOKUP_TABLE")) {
                            LOG::ERROR("Line {} :Invalid SCALARS, expect `LOOKUP_TABLE`!", tokens.line_number());
                            return false;
                        }
                    }

                    // The name of the lookup table is not needed.
                    if (!tokens.next())
                        return error_unexpected_end_of_file(tokens.line_number());

                    if (!read_data_array(
                            tokens, data_target, name, scalar_type,
                            nb_components, data_target.nb_tuples)) {
                        return false;
                    }
                }
                else if (tokens.matches("VECTORS") || tokens.matches("NORMALS")) { // == VECTORS ===========
                    // VECTORS name type
                    if (!tokens.next())
                        return error_unexpected_end_of_file(tokens.line_number());
                    const std::string name = tokens.token();

                    VTKScalarType scalar_type;
                    if (!read_scalar_type(tokens, scalar_type))
                        return false;

                    if (!read_data_array(
                            tokens, data_target, name, scalar_type,
                            3, data_target.nb_tuples)) {
                        return false;
                    }
                }
                else if (tokens.matches("TENSORS")) { // == TENSORS ========================================
                    // TENSORS name type
                    if (!tokens.next())
                        return error_unexpected_end_of_file(tokens.line_number());
                    const std::string name = tokens.token();

                    VTKScalarType scalar_type;
                    if (!read_scalar_type(tokens, scalar_type))
                        return false;

                    if (!read_data_array(
                            tokens, data_target, name, scalar_type,
                            9, data_target.nb_tuples)) {
                        return false;
                    }
                }
                else if (tokens.matches("FIELD")) { // == FIELD ==========================================
                    // FIELD name nb_arrays
                    // array_name nb_components nb_tuples type
                    if (!tokens.next())
                        return error_unexpected_end_of_file(tokens.line_number());
                    if (!tokens.next())
                        return error_unexpected_end_of_file(tokens.line_number());
                    const index_t nb_arrays = tokens.as_uint();

                    for (index_t a = 0; a < nb_arrays; ++a) {
                        if (!tokens.next())
                            return error_unexpected_end_of_file(tokens.line_number());
                        const std::string name = tokens.token();

                        if (!tokens.next())
                            return error_unexpected_end_of_file(tokens.line_number());
                        const index_t nb_components = tokens.as_uint();

                        if (!tokens.next())
                            return error_unexpected_end_of_file(tokens.line_number());
                        const index_t nb_tuples = tokens.as_uint();

                        VTKScalarType scalar_type;
                        if (!read_scalar_type(tokens, scalar_type))
                            return false;

                        if (!read_data_array(
                                tokens, data_target, name, scalar_type,
                                nb_components, nb_tuples)) {
                            return false;
                        }
                    }
                }
                else if (tokens.matches("LOOKUP_TABLE")) { // == LOOKUP_TABLE ==============================
                    // LOOKUP_TABLE name size, followed by `size` RGBA values.
                    // The lookup table of a SCALARS array is already consumed
                    // by the SCALARS branch, so this one is a definition.
                    if (!tokens.next())
                        return error_unexpected_end_of_file(tokens.line_number());
                    if (!tokens.next())
                        return error_unexpected_end_of_file(tokens.line_number());
                    if (!skip_values(tokens, 4*tokens.as_uint()))
                        return false;
                }
                else { // == Unknown =========================================================
                    LOG::WARN(
                        "Line {} :Unsupported keyword `{}`, the rest of the file is ignored.",
                        tokens.line_number(), tokens.token());
                    break;
                }
            }
        }
        catch (const std::logic_error& e) {
            LOG::ERROR("Invalid file `{}` at line {}: {}", filename, tokens.line_number(), e.what());
            mesh.clear();
            return false;
        }

        if (!ensure_elements())
            return false;

        if (mesh.facets.nb() != 0)
            mesh.facets.connect();
        if (mesh.cells.nb() != 0)
            mesh.cells.connect();

        return true;
    }
}
