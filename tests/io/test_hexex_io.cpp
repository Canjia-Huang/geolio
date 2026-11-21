//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/11/22.
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
#include <geolio/io/hexex_io.h>

namespace geolio::test
{
    namespace
    {
        /**
         * @brief A corner coordinate of a hexahedral integer grid mapping.
         * @details This is the value the handler used to choke on: the
         *          rational has 327 digits over 326 digits, so neither of
         *          its parts fits in a long double on platforms where
         *          long double is 64 bit, while the quotient is the ordinary
         *          number 11.076418692685206.
         */
        constexpr const char* HUGE_UVW_X =
            "1425173472268483144267180947737288640901874841623393279987864502"
            "9632683240884252360915430823549920228894501129729260180176992389"
            "4668231474543476831520961272815397127860332278400434674135207840"
            "4264330809008728555902887003820023427659623674577407304679125764"
            "3782789108274178876923118897600106989967699411616725938357413135"
            "6397073/12866735285202411939836957653277711141392059613965316925"
            "0828734582535512807190688151108914654442029306751551438820450299"
            "7951419987647572045026202717320939314606756497241942268248037760"
            "0866064660568692753862766992247693152004809073988834777315393469"
            "7131601207988236327891105055206240468337088426875845944723146284"
            "58012886433792";

        /**
         * @brief Another corner coordinate, with 356 digits over 354 digits.
         * @details Its quotient is the ordinary number -27.085881990351517.
         */
        constexpr const char* HUGE_UVW_Y =
            "-191591334514829728609460348953326721534900502413086791177169492"
            "6423839887233384479089171438781112194289078259648701628379133854"
            "3014253756964561860007264112490652418474980902668956823878881173"
            "2199718112515059954936617074313377605462545893283455936578297074"
            "7999000211680390506343962622839514710732081757863390005538053167"
            "5054846859596380293738912653793236759/70734759378733926348684226"
            "6647920562685038963755986682421881105665854424667650154263370098"
            "8365698619244962653720737882956963924515296399348062880016372718"
            "6131489133102910124560111858866127912859886504056398248815693798"
            "1948268285931992689069081158088865089118078497249929614158918103"
            "8119305830946835697656687580179930280670463628975710995805816924"
            "89859072";

        /**
         * @brief Writes a .hexex file and returns its path.
         * @details The file is written in the temporary directory, so that a
         *          failing test does not pollute the build tree.
         * @param[in] test_name the name of the test, used to make the path unique
         * @param[in] contents the contents of the file
         */
        std::string write_hexex_file(const std::string& test_name, const std::string& contents) {
            const std::string filepath = (
                std::filesystem::temp_directory_path() /
                ("geolio_" + test_name + ".hexex")
            ).string();

            std::ofstream out(filepath);
            out << contents;
            return filepath;
        }

        /**
         * @brief Writes and loads a .hexex file, then removes it.
         * @param[in] test_name the name of the test, used to make the path unique
         * @param[in] contents the contents of the file
         * @param[out] mesh the loaded mesh
         * @return true if the file could be loaded
         */
        bool load_hexex(const std::string& test_name, const std::string& contents, GEO::Mesh& mesh) {
            const std::string filepath = write_hexex_file(test_name, contents);
            const bool result = GEO::mesh_load(filepath, mesh);
            std::filesystem::remove(filepath);
            return result;
        }

        /**
         * @brief The .hexex header of a single tetrahedron with the given uvw.
         * @details One tetrahedron whose four corners carry the integer grid
         *          coordinates given by \p uvw, written as uvw.x uvw.y uvw.z
         *          triples separated by spaces.
         */
        std::string single_tet_hexex(const std::string& uvw) {
            return
                "4\n"
                "0 0 0\n"
                "1 0 0\n"
                "0 1 0\n"
                "0 0 1\n"
                "1\n"
                "0 1 2 3 " + uvw + "\n";
        }
    }

    /**
     * @brief Corner coordinates written as rationals with more than 308 digits
     *        are loaded instead of failing.
     * @details The quoted coordinates come from
     *          https://github.com/HendrikBrueckler/MC3D-samples style files:
     *          a hexahedral mesh with its integer grid mapping, where the
     *          uvw of a corner is written as an exact rational. Since
     *          long double is 64 bit on Apple Silicon, the numerator and the
     *          denominator of these rationals overflow when they are
     *          converted one by one, although their quotient does not. The
     *          handler must read them anyway.
     */
    TEST(HEXEXIO, LoadsCornerUvwBeyondDoubleRange) {
        GEO::Mesh mesh;
        ASSERT_TRUE(load_hexex(
            "LoadsCornerUvwBeyondDoubleRange",
            single_tet_hexex(
                "0 0 0 "
                "1 2 3 " +
                std::string(HUGE_UVW_X) + " 0 1 " +
                std::string(HUGE_UVW_Y) + " 5 -2"),
            mesh));

        ASSERT_EQ(mesh.vertices.nb(), 4);
        ASSERT_EQ(mesh.cells.nb(), 1);
        ASSERT_TRUE(mesh.cell_corners.attributes().is_defined("uvw"));

        GEO::Attribute<GEO::vec3> uvw(mesh.cell_corners.attributes(), "uvw");
        ASSERT_TRUE(uvw.is_bound());
        EXPECT_EQ(uvw.size(), 4);

        // The plain coordinates are read as is.
        EXPECT_DOUBLE_EQ(uvw[mesh.cells.corner(0, 0)].x, 0.0);
        EXPECT_DOUBLE_EQ(uvw[mesh.cells.corner(0, 1)].z, 3.0);

        // The huge rationals keep their quotient, up to the accuracy of a double.
        EXPECT_NEAR(uvw[mesh.cells.corner(0, 2)].x, 11.076418692685206, 1e-12);
        EXPECT_DOUBLE_EQ(uvw[mesh.cells.corner(0, 2)].y, 0.0);
        EXPECT_DOUBLE_EQ(uvw[mesh.cells.corner(0, 2)].z, 1.0);
        EXPECT_NEAR(uvw[mesh.cells.corner(0, 3)].x, -27.085881990351517, 1e-12);
        EXPECT_DOUBLE_EQ(uvw[mesh.cells.corner(0, 3)].y, 5.0);
        EXPECT_DOUBLE_EQ(uvw[mesh.cells.corner(0, 3)].z, -2.0);

        // The vertices are unaffected by the parameterization.
        EXPECT_DOUBLE_EQ(mesh.vertices.point(3).z, 1.0);
    }

    /**
     * @brief A vertex coordinate written as a subnormal double is loaded.
     * @details The file i13u_s6_MC.hexex has a vertex whose x coordinate is
     *          the smallest positive subnormal, written 4.9406564584124654e-324
     *          (numerical noise of the mapping that should be zero). Reading
     *          it through strtod() sets errno to ERANGE, which used to be
     *          reported as a conversion failure and made the whole file
     *          unreadable.
     */
    TEST(HEXEXIO, LoadsSubnormalVertexCoordinate) {
        GEO::Mesh mesh;
        ASSERT_TRUE(load_hexex(
            "LoadsSubnormalVertexCoordinate",
            "4\n"
            "4.9406564584124654e-324 47.754007982527192 -0.10158590171159046\n"
            "1 0 0\n"
            "0 1 0\n"
            "0 0 1\n"
            "1\n"
            "0 1 2 3 0 0 0 1 0 0 0 1 0 0 0 1\n",
            mesh));

        ASSERT_EQ(mesh.vertices.nb(), 4);
        ASSERT_EQ(mesh.cells.nb(), 1);

        // The subnormal keeps its exact value instead of being zeroed or
        // rejected.
        EXPECT_DOUBLE_EQ(mesh.vertices.point(0).x, 4.9406564584124654e-324);
        EXPECT_DOUBLE_EQ(mesh.vertices.point(0).y, 47.754007982527192);
        EXPECT_DOUBLE_EQ(mesh.vertices.point(0).z, -0.10158590171159046);
    }

    /**
     * @brief The optional MC3D-samples trailer is loaded: wall triangles and
     *        the feature vertices, edges and facets.
     */
    TEST(HEXEXIO, LoadsWallTrianglesAndFeatures) {
        GEO::Mesh mesh;
        ASSERT_TRUE(load_hexex(
            "LoadsWallTrianglesAndFeatures",
            single_tet_hexex("0 0 0 1 0 0 0 1 0 0 0 1") +
            // One wall triangle with its distance to the origin.
            "1\n"
            "0 1 2 0.5\n"
            // 1 feature vertex, 1 feature edge, 1 feature facet.
            "1 1 1\n"
            "0\n"
            "0 1\n"
            "0 1 2\n",
            mesh));

        ASSERT_EQ(mesh.vertices.nb(), 4);
        ASSERT_EQ(mesh.cells.nb(), 1);
        ASSERT_EQ(mesh.facets.nb(), 1);
        EXPECT_EQ(mesh.facets.nb_vertices(0), 3);
        EXPECT_EQ(mesh.facets.vertex(0, 0), 0);
        EXPECT_EQ(mesh.facets.vertex(0, 1), 1);
        EXPECT_EQ(mesh.facets.vertex(0, 2), 2);

        GEO::Attribute<double> dist_to_origin(mesh.facets.attributes(), "dist_to_origin");
        ASSERT_TRUE(dist_to_origin.is_bound());
        EXPECT_DOUBLE_EQ(dist_to_origin[0], 0.5);

        // The feature facet already exists as a wall triangle, so no facet is
        // added and the existing one is flagged.
        GEO::Attribute<bool> f_feature(mesh.facets.attributes(), "feature");
        ASSERT_TRUE(f_feature.is_bound());
        EXPECT_TRUE(f_feature[0]);

        ASSERT_EQ(mesh.edges.nb(), 1);
        EXPECT_EQ(mesh.edges.vertex(0, 0), 0);
        EXPECT_EQ(mesh.edges.vertex(0, 1), 1);
        GEO::Attribute<bool> e_feature(mesh.edges.attributes(), "feature");
        ASSERT_TRUE(e_feature.is_bound());
        EXPECT_TRUE(e_feature[0]);

        GEO::Attribute<bool> v_feature(mesh.vertices.attributes(), "feature");
        ASSERT_TRUE(v_feature.is_bound());
        EXPECT_TRUE(v_feature[0]);
        EXPECT_FALSE(v_feature[1]);
    }

    /**
     * @brief A feature facet that is not a wall triangle is added to the facets.
     */
    TEST(HEXEXIO, AddsMissingFeatureFacets) {
        GEO::Mesh mesh;
        ASSERT_TRUE(load_hexex(
            "AddsMissingFeatureFacets",
            single_tet_hexex("0 0 0 1 0 0 0 1 0 0 0 1") +
            "1\n"
            "0 1 2 0.5\n"
            // 0 feature vertices, 0 feature edges, 1 feature facet that is
            // not among the wall triangles.
            "0 0 1\n"
            "1 2 3\n",
            mesh));

        ASSERT_EQ(mesh.facets.nb(), 2);
        EXPECT_EQ(mesh.facets.vertex(1, 0), 1);
        EXPECT_EQ(mesh.facets.vertex(1, 1), 2);
        EXPECT_EQ(mesh.facets.vertex(1, 2), 3);
        GEO::Attribute<bool> f_feature(mesh.facets.attributes(), "feature");
        ASSERT_TRUE(f_feature.is_bound());
        EXPECT_FALSE(f_feature[0]);
        EXPECT_TRUE(f_feature[1]);
    }
}
