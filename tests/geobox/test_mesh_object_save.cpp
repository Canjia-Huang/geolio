//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/31.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <gtest/gtest.h>
#include <filesystem>
#include <memory>
#include <string>
#include <geogram/basic/attributes.h>
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_io.h>
#include <geolio/geobox/object/mesh_object.h>

namespace geolio::test
{
    namespace
    {
        /// A single triangle.
        void make_triangle_mesh(GEO::Mesh& mesh) {
            mesh.vertices.set_dimension(3);
            mesh.vertices.create_vertices(3);
            mesh.vertices.point(0) = GEO::vec3(0.0, 0.0, 0.0);
            mesh.vertices.point(1) = GEO::vec3(1.0, 0.0, 0.0);
            mesh.vertices.point(2) = GEO::vec3(0.0, 1.0, 0.0);
            mesh.facets.create_triangle(0, 1, 2);
        }

        /// Path of a file unique to \p test_name, removed by the tests.
        std::string save_test_file(const std::string& test_name) {
            return (
                std::filesystem::temp_directory_path() /
                ("geolio_" + test_name + ".geogram")
            ).string();
        }
    }

    /**
     * @brief Saving an object writes the mesh and the attributes it carries.
     * @details Saving is what the "Save as..." entry of an object's context menu
     *          does: the data an application computed lives in the attributes of
     *          the mesh (the sizing field of GEO::compute_sizing_field() is
     *          stored in "weight"), so they have to be written too -- the
     *          default IO flags would save none.
     */
    TEST(MeshObjectSaveTest, WritesTheMeshAndItsAttributes) {
        const std::string filepath = save_test_file("WritesTheMeshAndItsAttributes");

        GEO::Mesh mesh;
        make_triangle_mesh(mesh);
        {
            GEO::Attribute<double> weight(
                mesh.vertices.attributes(), "weight");
            weight[0] = 0.5;
            weight[1] = 1.5;
            weight[2] = 2.5;
        }

        geobox::MeshObject object("mesh", "", mesh);
        ASSERT_TRUE(object.save(filepath));

        // The file holds the geometry...
        GEO::Mesh reloaded;
        GEO::MeshIOFlags flags;
        flags.set_attributes(GEO::MESH_ALL_ATTRIBUTES);
        ASSERT_TRUE(GEO::mesh_load(filepath, reloaded, flags));
        EXPECT_EQ(reloaded.vertices.nb(), 3);
        EXPECT_EQ(reloaded.facets.nb(), 1);
        EXPECT_DOUBLE_EQ(reloaded.vertices.point(1)[0], 1.0);

        // ... and the attribute the object carried.
        ASSERT_TRUE(reloaded.vertices.attributes().is_defined("weight"));
        GEO::Attribute<double> reloaded_weight(
            reloaded.vertices.attributes(), "weight");
        EXPECT_DOUBLE_EQ(reloaded_weight[2], 2.5);

        std::filesystem::remove(filepath);
    }

    /**
     * @brief A mesh accepts the formats it can be written in, and refuses the
     *        ones that can only be read.
     * @details The save dialog only offers the extensions
     *          can_write_extension() accepts. GeoLio registers handlers for the
     *          formats of other tools that only load (see
     *          geolio::register_additional_MeshIOHandlers()), so those are
     *          refused instead of being offered and then failing.
     */
    TEST(MeshObjectSaveTest, AcceptsOnlyTheFormatsItCanBeWrittenIn) {
        GEO::Mesh mesh;
        make_triangle_mesh(mesh);
        geobox::MeshObject object("mesh", "", mesh);

        // Written by GeoGram's mesh IO handlers, and by the OFF and PTS handlers
        // of GeoLio.
        EXPECT_TRUE(object.can_write_extension("geogram"));
        EXPECT_TRUE(object.can_write_extension("obj"));
        EXPECT_TRUE(object.can_write_extension("off"));
        EXPECT_TRUE(object.can_write_extension("pts"));

        // Read-only handlers.
        EXPECT_FALSE(object.can_write_extension("aedtplt"));
        EXPECT_FALSE(object.can_write_extension("hexex"));
        EXPECT_FALSE(object.can_write_extension("ovm"));

        // A format nothing is registered for is not written either.
        EXPECT_FALSE(object.save(save_test_file("not_a_mesh_format") + ".unknown"));
    }

    /**
     * @brief An object that was not read from a file can be saved too.
     * @details The object names the values of its mesh, which is what the file
     *          dialog proposes as a file name (see
     *          GeoBoxApplication::open_object_save_dialog()).
     */
    TEST(MeshObjectSaveTest, NamesObjectsThatWereNotReadFromAFile) {
        GEO::Mesh mesh;
        make_triangle_mesh(mesh);

        geobox::MeshObject object("samples", "", mesh);
        EXPECT_EQ(object.filepath(), "");

        const std::string filepath = save_test_file("NamesObjectsThatWereNotReadFromAFile");
        EXPECT_TRUE(object.save(filepath));
        EXPECT_TRUE(std::filesystem::exists(filepath));

        std::filesystem::remove(filepath);
    }
}
