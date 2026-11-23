//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/11/23.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <string>
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_io.h>
#include <geolio/io/mphtxt_io.h>

namespace geolio::test
{
    namespace
    {
        /**
         * @brief Writes a .mphtxt file and returns its path.
         * @details The file is written in the temporary directory, so that a
         *          failing test does not pollute the build tree.
         * @param[in] test_name the name of the test, used to make the path unique
         * @param[in] contents the contents of the file
         */
        std::string write_mphtxt_file(const std::string& test_name, const std::string& contents) {
            const std::string filepath = (
                std::filesystem::temp_directory_path() /
                ("geolio_" + test_name + ".mphtxt")
            ).string();

            std::ofstream out(filepath);
            out << contents;
            return filepath;
        }

        /**
         * @brief Writes and loads a .mphtxt file, then removes it.
         * @param[in] test_name the name of the test, used to make the path unique
         * @param[in] contents the contents of the file
         * @param[out] mesh the loaded mesh
         * @return true if the file could be loaded
         */
        bool load_mphtxt(const std::string& test_name, const std::string& contents, GEO::Mesh& mesh) {
            const std::string filepath = write_mphtxt_file(test_name, contents);
            const bool result = GEO::mesh_load(filepath, mesh);
            std::filesystem::remove(filepath);
            return result;
        }

        /**
         * @brief The header of a COMSOL text mesh file, up to the `sdim` field.
         * @details The header follows the version 8 layout, that is the one
         *          exported by the current versions of COMSOL Multiphysics,
         *          with the included geometric model and the `# Mesh data`
         *          sections that precede the mesh vertices.
         * @param[in] sdim the space dimension of the mesh
         * @param[in] contents the body of the file, from the number of vertices
         */
        std::string mphtxt_header(GEO::index_t sdim, const std::string& contents) {
            return
                "# Created by COMSOL Multiphysics.\n"
                "\n"
                "# Major & minor version\n"
                "0 1 \n"
                "1 # number of tags\n"
                "# Tags\n"
                "5 mesh1 \n"
                "1 # number of types\n"
                "# Types\n"
                "3 obj \n"
                "\n"
                "# --------- Object 0 ----------\n"
                "\n"
                "0 0 1 \n"
                "4 Mesh # class\n"
                "8 # version\n" +
                std::to_string(sdim) + " # sdim\n"
                "\n"
                "# Include geometric model\n"
                "0 \n"
                "\n"
                "# Mesh data\n"
                "\n" +
                contents;
        }
    }

    /**
     * @brief A version 8 file is loaded: the vertices and the tetrahedra are
     *        read, while the boundary triangles and the geometric entity
     *        indices are discarded.
     */
    TEST(MPHTXTIO, LoadsVerticesAndTetrahedra) {
        GEO::Mesh mesh;
        ASSERT_TRUE(load_mphtxt(
            "LoadsVerticesAndTetrahedra",
            mphtxt_header(3,
                "5 # number of mesh vertices\n"
                "\n"
                "# Mesh vertex coordinates\n"
                "0 0 0 \n"
                "1 0 0 \n"
                "0 1 0 \n"
                "0 0 1 \n"
                "0.25 0.25 0.25 \n"
                "\n"
                "2 # number of element types\n"
                "\n"
                "# Type #0\n"
                "\n"
                "3 tet # type name\n"
                "\n"
                "4 # number of vertices per element\n"
                "2 # number of elements\n"
                "# Elements\n"
                "0 1 2 3 \n"
                "0 3 2 4 \n"
                "\n"
                "2 # number of geometric entity indices\n"
                "# Geometric entity indices\n"
                "1 \n"
                "2 \n"
                "\n"
                "# Type #1\n"
                "\n"
                "3 tri # type name\n"
                "\n"
                "3 # number of vertices per element\n"
                "1 # number of elements\n"
                "# Elements\n"
                "0 1 2 \n"
                "\n"
                "1 # number of geometric entity indices\n"
                "# Geometric entity indices\n"
                "0 \n"),
            mesh));

        ASSERT_EQ(mesh.vertices.nb(), 5);
        EXPECT_DOUBLE_EQ(mesh.vertices.point(0).x, 0.0);
        EXPECT_DOUBLE_EQ(mesh.vertices.point(0).y, 0.0);
        EXPECT_DOUBLE_EQ(mesh.vertices.point(0).z, 0.0);
        EXPECT_DOUBLE_EQ(mesh.vertices.point(4).x, 0.25);
        EXPECT_DOUBLE_EQ(mesh.vertices.point(4).y, 0.25);
        EXPECT_DOUBLE_EQ(mesh.vertices.point(4).z, 0.25);

        ASSERT_EQ(mesh.cells.nb(), 2);
        for (GEO::index_t c = 0; c < 2; ++c)
            ASSERT_EQ(mesh.cells.nb_vertices(c), 4);
        EXPECT_EQ(mesh.cells.vertex(0, 0), 0);
        EXPECT_EQ(mesh.cells.vertex(0, 1), 1);
        EXPECT_EQ(mesh.cells.vertex(0, 2), 2);
        EXPECT_EQ(mesh.cells.vertex(0, 3), 3);
        EXPECT_EQ(mesh.cells.vertex(1, 0), 0);
        EXPECT_EQ(mesh.cells.vertex(1, 1), 3);
        EXPECT_EQ(mesh.cells.vertex(1, 2), 2);
        EXPECT_EQ(mesh.cells.vertex(1, 3), 4);
    }

    /**
     * @brief A version 4 file is loaded: the vertices are numbered from the
     *        `lowest mesh vertex index` field, so the element indices are
     *        shifted to be 0-based.
     * @details The file also holds the parameter records of a mesh with a
     *          geometry, which follow the elements of each type and must be
     *          skipped without breaking the parse of the next type.
     */
    TEST(MPHTXTIO, LoadsMeshStartingAtNonZeroVertexIndex) {
        GEO::Mesh mesh;
        ASSERT_TRUE(load_mphtxt(
            "LoadsMeshStartingAtNonZeroVertexIndex",
            mphtxt_header(3,
                "6 # number of mesh vertices\n"
                "1 # lowest mesh vertex index\n"
                "# Mesh vertex coordinates\n"
                "0 0 0 \n"
                "1 0 0 \n"
                "0 1 0 \n"
                "0 0 1 \n"
                "0.5 0.5 0.5 \n"
                "1 1 1 \n"
                "\n"
                "2 # number of element types\n"
                "\n"
                "# Type #0\n"
                "\n"
                "3 tet # type name\n"
                "\n"
                "4 # number of vertices per element\n"
                "2 # number of elements\n"
                "# Elements\n"
                "1 2 3 4 \n"
                "5 2 1 3 \n"
                "\n"
                "1 # number of parameter values per element\n"
                "0 # number of parameters\n"
                "\n"
                "2 # number of geometric entity indices\n"
                "# Geometric entity indices\n"
                "1 \n"
                "1 \n"
                "\n"
                "0 # number of up/down pairs\n"
                "\n"
                "# Type #1\n"
                "\n"
                "3 tri # type name\n"
                "\n"
                "3 # number of vertices per element\n"
                "2 # number of elements\n"
                "# Elements\n"
                "1 2 5 \n"
                "2 6 3 \n"
                "\n"
                "2 # number of geometric entity indices\n"
                "# Geometric entity indices\n"
                "0 \n"
                "0 \n"
                "\n"
                "0 # number of up/down pairs\n"),
            mesh));

        ASSERT_EQ(mesh.vertices.nb(), 6);
        EXPECT_DOUBLE_EQ(mesh.vertices.point(0).y, 0.0);
        EXPECT_DOUBLE_EQ(mesh.vertices.point(4).z, 0.5);
        EXPECT_DOUBLE_EQ(mesh.vertices.point(5).x, 1.0);

        ASSERT_EQ(mesh.cells.nb(), 2);
        EXPECT_EQ(mesh.cells.vertex(0, 0), 0);
        EXPECT_EQ(mesh.cells.vertex(0, 1), 1);
        EXPECT_EQ(mesh.cells.vertex(0, 2), 2);
        EXPECT_EQ(mesh.cells.vertex(0, 3), 3);
        EXPECT_EQ(mesh.cells.vertex(1, 0), 4);
        EXPECT_EQ(mesh.cells.vertex(1, 1), 1);
        EXPECT_EQ(mesh.cells.vertex(1, 2), 0);
        EXPECT_EQ(mesh.cells.vertex(1, 3), 2);
    }

    /**
     * @brief A second-order tetrahedron keeps its 4 corner vertices.
     * @details A "tet2" element has 10 vertices: the 4 corners first, then the
     *          6 edge nodes, which are dropped because the mesh only holds
     *          linear tetrahedra.
     */
    TEST(MPHTXTIO, KeepsCornersOfSecondOrderTetrahedra) {
        GEO::Mesh mesh;
        ASSERT_TRUE(load_mphtxt(
            "KeepsCornersOfSecondOrderTetrahedra",
            mphtxt_header(3,
                "10 # number of mesh vertices\n"
                "# Mesh vertex coordinates\n"
                "0 0 0 \n"
                "1 0 0 \n"
                "0 1 0 \n"
                "0 0 1 \n"
                "0.5 0 0 \n"
                "0 0.5 0 \n"
                "0.5 0.5 0 \n"
                "0 0 0.5 \n"
                "0.5 0 0.5 \n"
                "0 0.5 0.5 \n"
                "\n"
                "1 # number of element types\n"
                "\n"
                "# Type #0\n"
                "\n"
                "4 tet2 # type name\n"
                "\n"
                "10 # number of vertices per element\n"
                "1 # number of elements\n"
                "# Elements\n"
                "0 1 2 3 4 5 6 7 8 9 \n"
                "\n"
                "0 # number of geometric entity indices\n"),
            mesh));

        ASSERT_EQ(mesh.vertices.nb(), 10);
        ASSERT_EQ(mesh.cells.nb(), 1);
        EXPECT_EQ(mesh.cells.vertex(0, 0), 0);
        EXPECT_EQ(mesh.cells.vertex(0, 1), 1);
        EXPECT_EQ(mesh.cells.vertex(0, 2), 2);
        EXPECT_EQ(mesh.cells.vertex(0, 3), 3);
    }

    /**
     * @brief The Mesh objects of a file holding several of them are merged.
     * @details The elements of the second object are renumbered to point to
     *          the vertices of the merged mesh.
     */
    TEST(MPHTXTIO, MergesSeveralMeshObjects) {
        GEO::Mesh mesh;
        ASSERT_TRUE(load_mphtxt(
            "MergesSeveralMeshObjects",
            mphtxt_header(3,
                "4 # number of mesh vertices\n"
                "# Mesh vertex coordinates\n"
                "0 0 0 \n"
                "1 0 0 \n"
                "0 1 0 \n"
                "0 0 1 \n"
                "\n"
                "1 # number of element types\n"
                "\n"
                "# Type #0\n"
                "\n"
                "3 tet # type name\n"
                "\n"
                "4 # number of vertices per element\n"
                "1 # number of elements\n"
                "# Elements\n"
                "0 1 2 3 \n"
                "\n"
                "0 # number of geometric entity indices\n"
                "\n"
                "# --------- Object 1 ----------\n"
                "\n"
                "0 0 1 \n"
                "4 Mesh # class\n"
                "8 # version\n"
                "3 # sdim\n"
                "\n"
                "# Include geometric model\n"
                "0 \n"
                "\n"
                "# Mesh data\n"
                "\n"
                "4 # number of mesh vertices\n"
                "# Mesh vertex coordinates\n"
                "2 0 0 \n"
                "3 0 0 \n"
                "2 1 0 \n"
                "2 0 1 \n"
                "\n"
                "1 # number of element types\n"
                "\n"
                "# Type #0\n"
                "\n"
                "3 tet # type name\n"
                "\n"
                "4 # number of vertices per element\n"
                "1 # number of elements\n"
                "# Elements\n"
                "0 1 2 3 \n"
                "\n"
                "0 # number of geometric entity indices\n"),
            mesh));

        ASSERT_EQ(mesh.vertices.nb(), 8);
        EXPECT_DOUBLE_EQ(mesh.vertices.point(4).x, 2.0);
        EXPECT_DOUBLE_EQ(mesh.vertices.point(7).z, 1.0);

        ASSERT_EQ(mesh.cells.nb(), 2);
        EXPECT_EQ(mesh.cells.vertex(0, 0), 0);
        EXPECT_EQ(mesh.cells.vertex(0, 3), 3);
        EXPECT_EQ(mesh.cells.vertex(1, 0), 4);
        EXPECT_EQ(mesh.cells.vertex(1, 1), 5);
        EXPECT_EQ(mesh.cells.vertex(1, 2), 6);
        EXPECT_EQ(mesh.cells.vertex(1, 3), 7);
    }

    /**
     * @brief A 2D mesh gives its vertices with a zero z coordinate.
     * @details A 2D .mphtxt file holds triangles instead of tetrahedra: the
     *          element type is discarded, so the mesh has no cell.
     */
    TEST(MPHTXTIO, LoadsTwoDimensionalMesh) {
        GEO::Mesh mesh;
        ASSERT_TRUE(load_mphtxt(
            "LoadsTwoDimensionalMesh",
            mphtxt_header(2,
                "4 # number of mesh vertices\n"
                "# Mesh vertex coordinates\n"
                "0 0 \n"
                "1 0 \n"
                "1 1 \n"
                "0 1 \n"
                "\n"
                "1 # number of element types\n"
                "\n"
                "# Type #0\n"
                "\n"
                "3 tri # type name\n"
                "\n"
                "3 # number of vertices per element\n"
                "2 # number of elements\n"
                "# Elements\n"
                "0 1 2 \n"
                "0 2 3 \n"),
            mesh));

        ASSERT_EQ(mesh.vertices.nb(), 4);
        EXPECT_EQ(mesh.cells.nb(), 0);
        EXPECT_DOUBLE_EQ(mesh.vertices.point(0).x, 0.0);
        EXPECT_DOUBLE_EQ(mesh.vertices.point(0).y, 0.0);
        EXPECT_DOUBLE_EQ(mesh.vertices.point(0).z, 0.0);
        EXPECT_DOUBLE_EQ(mesh.vertices.point(2).x, 1.0);
        EXPECT_DOUBLE_EQ(mesh.vertices.point(2).y, 1.0);
        EXPECT_DOUBLE_EQ(mesh.vertices.point(2).z, 0.0);
    }

    /**
     * @brief A tetrahedron that references a vertex outside of the mesh is an
     *        error, not silently loaded.
     */
    TEST(MPHTXTIO, RejectsVertexIndexOutOfRange) {
        GEO::Mesh mesh;
        EXPECT_FALSE(load_mphtxt(
            "RejectsVertexIndexOutOfRange",
            mphtxt_header(3,
                "4 # number of mesh vertices\n"
                "# Mesh vertex coordinates\n"
                "0 0 0 \n"
                "1 0 0 \n"
                "0 1 0 \n"
                "0 0 1 \n"
                "\n"
                "1 # number of element types\n"
                "\n"
                "# Type #0\n"
                "\n"
                "3 tet # type name\n"
                "\n"
                "4 # number of vertices per element\n"
                "1 # number of elements\n"
                "# Elements\n"
                "0 1 2 4 \n"),
            mesh));
        EXPECT_EQ(mesh.vertices.nb(), 0);
        EXPECT_EQ(mesh.cells.nb(), 0);
    }
}
