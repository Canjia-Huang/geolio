//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/11/15.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <string>
#include <geogram/basic/attributes.h>
#include <geogram/basic/geometry.h>
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_io.h>

namespace geolio::test
{
    namespace
    {
        /**
         * @brief Writes a legacy VTK file and returns its path.
         * @details The file is written in the temporary directory, so that a
         *          failing test does not pollute the build tree.
         * @param[in] test_name the name of the test, used to make the path unique
         * @param[in] contents the contents of the file
         */
        std::string write_vtk_file(const std::string& test_name, const std::string& contents) {
            const std::string filepath = (
                std::filesystem::temp_directory_path() /
                ("geolio_" + test_name + ".vtk")
            ).string();

            std::ofstream out(filepath);
            out << contents;
            return filepath;
        }

        /**
         * @brief Loads a legacy VTK file and removes it.
         * @param[in] test_name the name of the test, used to make the path unique
         * @param[in] contents the contents of the file
         * @param[out] mesh the loaded mesh
         * @return true if the file could be loaded
         */
        bool load_vtk(const std::string& test_name, const std::string& contents, GEO::Mesh& mesh) {
            const std::string filepath = write_vtk_file(test_name, contents);
            const bool result = GEO::mesh_load(filepath, mesh);
            std::filesystem::remove(filepath);
            return result;
        }
    }

    /**
     * @brief A tetrahedral mesh written by TetWild is loaded as cells.
     * @details This is the file the handler was written for: an
     *          UNSTRUCTURED_GRID of 49 vertices and 123 tetrahedra carrying a
     *          "GeoId" array in its CELL_DATA block.
     */
    TEST(VTKIO, LoadsTetWildTetrahedra) {
        GEO::Mesh mesh;
        ASSERT_TRUE(GEO::mesh_load(
            std::string(TEST_DATA_PATH) + "tetwild_79851_out.vtk", mesh));

        EXPECT_EQ(mesh.vertices.nb(), 49);
        EXPECT_EQ(mesh.cells.nb(), 123);
        EXPECT_EQ(mesh.edges.nb(), 0);
        EXPECT_TRUE(mesh.cells.are_simplices());

        // GEO::mesh_load() computes the boundary facets of a volume mesh, the
        // file itself has no polygonal cell.
        EXPECT_EQ(mesh.facets.nb(), 92);

        // The array of the CELL_DATA block became an attribute of the cells.
        ASSERT_TRUE(mesh.cells.attributes().is_defined("GeoId"));
        GEO::Attribute<float> geo_id(mesh.cells.attributes(), "GeoId");
        ASSERT_TRUE(geo_id.is_bound());
        EXPECT_EQ(geo_id.size(), 123);
        EXPECT_FLOAT_EQ(geo_id[0], 1.0f);
        EXPECT_FLOAT_EQ(geo_id[122], 1.0f);

        // The tetrahedra keep the orientation they have in the file.
        double volume = 0.0;
        for (GEO::index_t c = 0; c < mesh.cells.nb(); ++c) {
            volume += GEO::Geom::tetra_signed_volume(
                mesh.cells.point(c, 0),
                mesh.cells.point(c, 1),
                mesh.cells.point(c, 2),
                mesh.cells.point(c, 3));
        }
        EXPECT_NEAR(volume, 542.4149816777834, 1e-6);
    }

    /**
     * @brief The polygons of a POLYDATA file become facets.
     * @details The file mixes a quad and a triangle, and carries both a scalar
     *          and a vector array of POINT_DATA.
     */
    TEST(VTKIO, LoadsPolygonsAndPointData) {
        GEO::Mesh mesh;
        ASSERT_TRUE(load_vtk(
            "LoadsPolygonsAndPointData",
            "# vtk DataFile Version 3.0\n"
            "a quad and a triangle\n"
            "ASCII\n"
            "DATASET POLYDATA\n"
            "POINTS 5 float\n"
            "0 0 0\n"
            "1 0 0\n"
            "1 1 0\n"
            "0 1 0\n"
            "2 1 0\n"
            "POLYGONS 2 9\n"
            "4 0 1 2 3\n"
            "3 1 4 2\n"
            "POINT_DATA 5\n"
            "SCALARS weight float 1\n"
            "LOOKUP_TABLE default\n"
            "0 1 2 3 4\n"
            "VECTORS normal double\n"
            "0 0 1\n"
            "0 0 1\n"
            "0 0 1\n"
            "0 0 1\n"
            "0 0 1\n",
            mesh));

        ASSERT_EQ(mesh.vertices.nb(), 5);
        EXPECT_DOUBLE_EQ(mesh.vertices.point(4).x, 2.0);
        ASSERT_EQ(mesh.facets.nb(), 2);
        EXPECT_EQ(mesh.facets.nb_vertices(0), 4);
        EXPECT_EQ(mesh.facets.nb_vertices(1), 3);
        EXPECT_EQ(mesh.facets.vertex(1, 0), 1);
        EXPECT_EQ(mesh.facets.vertex(1, 2), 2);

        GEO::Attribute<float> weight(mesh.vertices.attributes(), "weight");
        ASSERT_TRUE(weight.is_bound());
        EXPECT_EQ(weight.size(), 5);
        EXPECT_FLOAT_EQ(weight[4], 4.0f);

        GEO::Attribute<GEO::vec3> normal(mesh.vertices.attributes(), "normal");
        ASSERT_TRUE(normal.is_bound());
        EXPECT_DOUBLE_EQ(normal[3].z, 1.0);
    }

    /**
     * @brief A file whose title line is empty is loaded.
     * @details The title of a legacy VTK file is free text and may be empty;
     *          the encoding line then follows the header directly.
     */
    TEST(VTKIO, LoadsFileWithoutTitle) {
        GEO::Mesh mesh;
        ASSERT_TRUE(load_vtk(
            "LoadsFileWithoutTitle",
            "# vtk DataFile Version 2.0\n"
            "\n"
            "ASCII\n"
            "DATASET POLYDATA\n"
            "POINTS 3 double\n"
            "0 0 0\n"
            "1 0 0\n"
            "0 1 0\n"
            "POLYGONS 1 4\n"
            "3 0 1 2\n",
            mesh));

        EXPECT_EQ(mesh.vertices.nb(), 3);
        EXPECT_EQ(mesh.facets.nb(), 1);
    }

    /**
     * @brief Pixels are renumbered and multi-component arrays are loaded.
     * @details VTK enumerates the corners of a pixel as 0, 1, 3, 2, and its
     *          data arrays may have several components: they are stored as
     *          vector attributes whose type follows the scalar type of the
     *          file.
     */
    TEST(VTKIO, LoadsPixelsAndVectorAttributes) {
        GEO::Mesh mesh;
        ASSERT_TRUE(load_vtk(
            "LoadsPixelsAndVectorAttributes",
            "# vtk DataFile Version 3.0\n"
            "a pixel\n"
            "ASCII\n"
            "DATASET POLYDATA\n"
            "POINTS 4 double\n"
            "0 0 0\n"
            "1 0 0\n"
            "0 1 0\n"
            "1 1 0\n"
            "POLYGONS 1 5\n"
            "4 0 1 2 3\n"
            "CELL_TYPES 1\n"
            "8\n"
            "POINT_DATA 4\n"
            "SCALARS uv float 2\n"
            "LOOKUP_TABLE default\n"
            "0 0\n"
            "1 0\n"
            "0 1\n"
            "1 1\n"
            "SCALARS frame double 4\n"
            "LOOKUP_TABLE default\n"
            "1 0 0 0\n"
            "1 0 0 0\n"
            "1 0 0 0\n"
            "1 0 0 0\n",
            mesh));

        ASSERT_EQ(mesh.facets.nb(), 1);
        // The pixel becomes a quad whose corners are in the Geogram order.
        EXPECT_EQ(mesh.facets.vertex(0, 0), 0);
        EXPECT_EQ(mesh.facets.vertex(0, 1), 1);
        EXPECT_EQ(mesh.facets.vertex(0, 2), 3);
        EXPECT_EQ(mesh.facets.vertex(0, 3), 2);

        GEO::Attribute<GEO::vec2f> uv(mesh.vertices.attributes(), "uv");
        ASSERT_TRUE(uv.is_bound());
        EXPECT_FLOAT_EQ(uv[3].x, 1.0f);
        EXPECT_FLOAT_EQ(uv[3].y, 1.0f);

        GEO::Attribute<GEO::vec4> frame(mesh.vertices.attributes(), "frame");
        ASSERT_TRUE(frame.is_bound());
        EXPECT_DOUBLE_EQ(frame[2].w, 0.0);
    }

    /**
     * @brief The arrays of a FIELD block are loaded as attributes.
     * @details A FIELD block holds several arrays of possibly different
     *          types and numbers of components in a single block.
     */
    TEST(VTKIO, LoadsFieldArrays) {
        GEO::Mesh mesh;
        ASSERT_TRUE(load_vtk(
            "LoadsFieldArrays",
            "# vtk DataFile Version 3.0\n"
            "a triangle with a field\n"
            "ASCII\n"
            "DATASET POLYDATA\n"
            "POINTS 3 double\n"
            "0 0 0\n"
            "1 0 0\n"
            "0 1 0\n"
            "POLYGONS 1 4\n"
            "3 0 1 2\n"
            "POINT_DATA 3\n"
            "FIELD FieldData 2\n"
            "weight 1 3 double\n"
            "0.5 1.5 2.5\n"
            "color 3 3 float\n"
            "1 0 0 0 1 0 0 0 1\n",
            mesh));

        GEO::Attribute<double> weight(mesh.vertices.attributes(), "weight");
        ASSERT_TRUE(weight.is_bound());
        EXPECT_DOUBLE_EQ(weight[2], 2.5);

        GEO::Attribute<GEO::vec3f> color(mesh.vertices.attributes(), "color");
        ASSERT_TRUE(color.is_bound());
        EXPECT_FLOAT_EQ(color[1].y, 1.0f);
    }

    /**
     * @brief The polyhedra of an UNSTRUCTURED_GRID file become cells.
     * @details The file holds one cell of each supported type; the vertices of
     *          a hexahedron are renumbered from the VTK order to the Geogram
     *          one, the other cell types are copied unchanged. The CELL_DATA
     *          array becomes an attribute of the cells.
     */
    TEST(VTKIO, LoadsMixedCellsAndCellData) {
        GEO::Mesh mesh;
        ASSERT_TRUE(load_vtk(
            "LoadsMixedCellsAndCellData",
            "# vtk DataFile Version 2.0\n"
            "one cell of each type\n"
            "ASCII\n"
            "DATASET UNSTRUCTURED_GRID\n"
            "POINTS 23 double\n"
            // tetrahedron
            "0 0 0\n"
            "1 0 0\n"
            "0 1 0\n"
            "0 0 1\n"
            // hexahedron
            "1 0 0\n"
            "2 0 0\n"
            "2 1 0\n"
            "1 1 0\n"
            "1 0 1\n"
            "2 0 1\n"
            "2 1 1\n"
            "1 1 1\n"
            // wedge
            "3 0 0\n"
            "4 0 0\n"
            "3 1 0\n"
            "3 0 1\n"
            "4 0 1\n"
            "3 1 1\n"
            // pyramid
            "5 0 0\n"
            "6 0 0\n"
            "6 1 0\n"
            "5 1 0\n"
            "5.5 0.5 1\n"
            "CELLS 4 27\n"
            "4 0 1 2 3\n"
            "8 4 5 6 7 8 9 10 11\n"
            "6 12 13 14 15 16 17\n"
            "5 18 19 20 21 22\n"
            "CELL_TYPES 4\n"
            "10\n"
            "12\n"
            "13\n"
            "14\n"
            "CELL_DATA 4\n"
            "SCALARS id int 1\n"
            "LOOKUP_TABLE default\n"
            "7 8 9 10\n",
            mesh));

        ASSERT_EQ(mesh.vertices.nb(), 23);
        ASSERT_EQ(mesh.cells.nb(), 4);
        EXPECT_EQ(mesh.cells.type(0), GEO::MESH_TET);
        EXPECT_EQ(mesh.cells.type(1), GEO::MESH_HEX);
        EXPECT_EQ(mesh.cells.type(2), GEO::MESH_PRISM);
        EXPECT_EQ(mesh.cells.type(3), GEO::MESH_PYRAMID);

        // The four cells are disjoint, so their 4+6+5+5 facets are all on the
        // boundary that GEO::mesh_load() computes.
        EXPECT_EQ(mesh.facets.nb(), 20);

        // The tetrahedron is copied unchanged.
        EXPECT_EQ(mesh.cells.vertex(0, 0), 0);
        EXPECT_EQ(mesh.cells.vertex(0, 3), 3);

        // Geogram numbers the bottom quad of a hexahedron 0, 1, 3, 2, whereas
        // VTK numbers it 0, 1, 2, 3.
        EXPECT_EQ(mesh.cells.vertex(1, 0), 4);
        EXPECT_EQ(mesh.cells.vertex(1, 1), 5);
        EXPECT_EQ(mesh.cells.vertex(1, 2), 7);
        EXPECT_EQ(mesh.cells.vertex(1, 3), 6);
        EXPECT_EQ(mesh.cells.vertex(1, 6), 11);
        EXPECT_EQ(mesh.cells.vertex(1, 7), 10);

        // Wedges and pyramids share the VTK numbering.
        EXPECT_EQ(mesh.cells.vertex(2, 5), 17);
        EXPECT_EQ(mesh.cells.vertex(3, 4), 22);

        GEO::Attribute<GEO::Numeric::int32> id(mesh.cells.attributes(), "id");
        ASSERT_TRUE(id.is_bound());
        ASSERT_EQ(id.size(), 4);
        EXPECT_EQ(id[0], 7);
        EXPECT_EQ(id[3], 10);
    }

    /**
     * @brief A cell block written with the VTK 5.1 offsets layout is loaded.
     * @details Since VTK 5.1, `CELLS` may store the offsets of the cells and
     *          their connectivity in two separate blocks instead of the
     *          historical `nb_vertices v0 ... ` records.
     */
    TEST(VTKIO, LoadsCellsWithOffsets) {
        GEO::Mesh mesh;
        ASSERT_TRUE(load_vtk(
            "LoadsCellsWithOffsets",
            "# vtk DataFile Version 4.2\n"
            "two tetrahedra\n"
            "ASCII\n"
            "DATASET UNSTRUCTURED_GRID\n"
            "POINTS 8 double\n"
            "0 0 0\n"
            "1 0 0\n"
            "0 1 0\n"
            "0 0 1\n"
            "2 0 0\n"
            "3 0 0\n"
            "2 1 0\n"
            "2 0 1\n"
            "CELLS 2 10\n"
            "OFFSETS vtktypeint64\n"
            "0 4 8\n"
            "CONNECTIVITY vtktypeint64\n"
            "0 1 2 3 4 5 6 7\n"
            "CELL_TYPES 2\n"
            "10\n"
            "10\n",
            mesh));

        ASSERT_EQ(mesh.vertices.nb(), 8);
        ASSERT_EQ(mesh.cells.nb(), 2);
        EXPECT_EQ(mesh.cells.vertex(0, 0), 0);
        EXPECT_EQ(mesh.cells.vertex(0, 3), 3);
        EXPECT_EQ(mesh.cells.vertex(1, 0), 4);
        EXPECT_EQ(mesh.cells.vertex(1, 3), 7);
    }

    /**
     * @brief A triangle strip is split into triangles of alternating orientation.
     */
    TEST(VTKIO, SplitsTriangleStrips) {
        GEO::Mesh mesh;
        ASSERT_TRUE(load_vtk(
            "SplitsTriangleStrips",
            "# vtk DataFile Version 2.0\n"
            "a triangle strip\n"
            "ASCII\n"
            "DATASET POLYDATA\n"
            "POINTS 5 double\n"
            "0 0 0\n"
            "1 0 0\n"
            "0 1 0\n"
            "1 1 0\n"
            "0 2 0\n"
            "TRIANGLE_STRIPS 1 6\n"
            "5 0 1 2 3 4\n",
            mesh));

        ASSERT_EQ(mesh.vertices.nb(), 5);
        ASSERT_EQ(mesh.facets.nb(), 3);
        EXPECT_EQ(mesh.facets.vertex(0, 0), 0);
        EXPECT_EQ(mesh.facets.vertex(0, 1), 1);
        EXPECT_EQ(mesh.facets.vertex(0, 2), 2);
        EXPECT_EQ(mesh.facets.vertex(1, 0), 2);
        EXPECT_EQ(mesh.facets.vertex(1, 1), 1);
        EXPECT_EQ(mesh.facets.vertex(1, 2), 3);
        EXPECT_EQ(mesh.facets.vertex(2, 0), 2);
        EXPECT_EQ(mesh.facets.vertex(2, 1), 3);
        EXPECT_EQ(mesh.facets.vertex(2, 2), 4);
    }

    /**
     * @brief Line cells become edges.
     */
    TEST(VTKIO, LoadsLinesAsEdges) {
        GEO::Mesh mesh;
        ASSERT_TRUE(load_vtk(
            "LoadsLinesAsEdges",
            "# vtk DataFile Version 2.0\n"
            "a polyline\n"
            "ASCII\n"
            "DATASET POLYDATA\n"
            "POINTS 3 double\n"
            "0 0 0\n"
            "1 0 0\n"
            "2 0 0\n"
            "LINES 2 6\n"
            "2 0 1\n"
            "2 1 2\n",
            mesh));

        ASSERT_EQ(mesh.vertices.nb(), 3);
        ASSERT_EQ(mesh.edges.nb(), 2);
        EXPECT_EQ(mesh.edges.vertex(0, 0), 0);
        EXPECT_EQ(mesh.edges.vertex(0, 1), 1);
        EXPECT_EQ(mesh.edges.vertex(1, 1), 2);
    }

    /**
     * @brief The files the handler cannot read are rejected instead of
     *        producing a partially loaded mesh.
     */
    TEST(VTKIO, RejectsUnsupportedFiles) {
        GEO::Mesh mesh;
        // Binary files are not supported.
        EXPECT_FALSE(load_vtk(
            "RejectsBinaryFiles",
            "# vtk DataFile Version 2.0\n"
            "a binary file\n"
            "BINARY\n"
            "DATASET POLYDATA\n",
            mesh));

        // So are the dataset types that have no explicit connectivity.
        EXPECT_FALSE(load_vtk(
            "RejectsStructuredGrids",
            "# vtk DataFile Version 2.0\n"
            "a structured grid\n"
            "ASCII\n"
            "DATASET STRUCTURED_GRID\n",
            mesh));

        // A CELL_TYPES block that does not match the cells is an error.
        EXPECT_FALSE(load_vtk(
            "RejectsInconsistentCellTypes",
            "# vtk DataFile Version 2.0\n"
            "a broken file\n"
            "ASCII\n"
            "DATASET UNSTRUCTURED_GRID\n"
            "POINTS 4 double\n"
            "0 0 0\n"
            "1 0 0\n"
            "0 1 0\n"
            "0 0 1\n"
            "CELLS 1 5\n"
            "4 0 1 2 3\n"
            "CELL_TYPES 2\n"
            "10\n"
            "10\n",
            mesh));
    }
}
