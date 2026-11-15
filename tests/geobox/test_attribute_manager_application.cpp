//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/11/1.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <vector>
#include <geogram/basic/attributes.h>
#include <geogram/mesh/mesh.h>
#include <geogram_gfx/third_party/imgui/imgui.h>
#include <geolio/geobox/application/attributes/attribute_manager_application.h>
#include <geolio/geobox/object/mesh_object.h>

namespace geolio::test
{
    namespace
    {
        /**
         * @brief AttributeManagerApplication with the helpers it uses exposed.
         * @details They are what the rows of the table do (see
         *          AttributeManagerApplication); the window itself is drawn by
         *          the tests below, but nothing there can be clicked.
         */
        class TestAttributeManagerApplication
            : public geobox::AttributeManagerApplication {
        public:
            explicit TestAttributeManagerApplication(
                geobox::ApplicationObjects& objects)
                : AttributeManagerApplication(objects) {
            }

            /// Name of the type of the elements of an attribute, as the table
            /// shows it.
            static std::string type_name(
                const geobox::MeshObject::AttributeInfo& attribute
                ) {
                return element_type_name(attribute);
            }

            /// Changes the dimension of an attribute, as the field of its row
            /// does.
            static bool redim(
                geobox::MeshObject& mesh_object,
                const geobox::MeshObject::AttributeInfo& attribute,
                const GEO::index_t dimension
                ) {
                return redim_attribute(mesh_object, attribute, dimension);
            }

            /// Removes an attribute, as the row of a removed one does.
            static bool remove(
                geobox::MeshObject& mesh_object,
                const geobox::MeshObject::AttributeInfo& attribute
                ) {
                return remove_attribute(mesh_object, attribute);
            }

            /// Records that the row of \p attribute is not kept.
            void uncheck(const geobox::MeshObject::AttributeInfo& attribute) {
                keep_[attribute.full_name()] = false;
            }

            /// Records the dimension typed in the row of \p attribute.
            void set_dimension(
                const geobox::MeshObject::AttributeInfo& attribute,
                const GEO::index_t dimension
                ) {
                dimensions_[attribute.full_name()] = dimension;
            }

            /// Presses the reset button: drops what the rows asked for.
            void reset() {
                reset_pending_changes();
            }

            /// Presses the button: applies what the rows asked for.
            void perform(
                geobox::MeshObject& mesh_object,
                const std::vector<geobox::MeshObject::AttributeInfo>& attributes
                ) {
                apply_pending_changes(mesh_object, attributes);
            }
        };

        /// A triangle, i.e. a mesh with one vertex, facet and corner store.
        void make_triangle_mesh(GEO::Mesh& mesh) {
            mesh.vertices.set_dimension(3);
            mesh.vertices.create_vertices(3);
            mesh.vertices.point(0) = GEO::vec3(0.0, 0.0, 0.0);
            mesh.vertices.point(1) = GEO::vec3(1.0, 0.0, 0.0);
            mesh.vertices.point(2) = GEO::vec3(0.0, 1.0, 0.0);
            mesh.facets.create_triangle(0, 1, 2);
        }

        /// The attribute of \p mesh_object whose full name is \p name.
        geobox::MeshObject::AttributeInfo find_attribute(
            const geobox::MeshObject& mesh_object,
            const std::string& name
            ) {
            for (const auto& attribute : mesh_object.attribute_infos(true)) {
                if (attribute.full_name() == name)
                    return attribute;
            }
            return {};
        }
    }

    /**
     * @brief The attribute a row names is removed from the mesh.
     * @details The store is destroyed, so the attribute is gone from the mesh
     *          and from what the object lists -- and the row's button can be
     *          pressed twice only if the attribute is still there.
     */
    TEST(AttributeManagerApplicationTest, RemovesTheAttributeOfARow) {
        GEO::Mesh mesh;
        make_triangle_mesh(mesh);
        {
            GEO::Attribute<double> weight(
                mesh.vertices.attributes(), "weight");
            for (GEO::index_t v = 0; v < mesh.vertices.nb(); ++v)
                weight[v] = double(v);
        }
        {
            GEO::Attribute<GEO::vec3> normal(
                mesh.facets.attributes(), "normal");
            for (GEO::index_t f = 0; f < mesh.facets.nb(); ++f)
                normal[f] = GEO::vec3(0.0, 0.0, 1.0);
        }

        geobox::MeshObject mesh_object("mesh", "", mesh);
        const auto weight = find_attribute(mesh_object, "vertices.weight");
        ASSERT_EQ(weight.full_name(), "vertices.weight");
        EXPECT_EQ(weight.subelements, GEO::MESH_VERTICES);
        EXPECT_EQ(weight.dimension, 1u);

        // The table shows the C++ name of the element type: GeoGram stores an
        // attribute of vectors as an attribute of scalars with a dimension, so
        // both of them are made of doubles here, and the vector one has 3 of
        // them per facet.
        EXPECT_EQ(
            TestAttributeManagerApplication::type_name(weight), "double");
        const auto normal = find_attribute(mesh_object, "facets.normal");
        ASSERT_EQ(normal.full_name(), "facets.normal");
        EXPECT_EQ(TestAttributeManagerApplication::type_name(normal), "double");
        EXPECT_EQ(normal.dimension, 3u);

        EXPECT_TRUE(TestAttributeManagerApplication::remove(
            mesh_object, weight));
        EXPECT_FALSE(
            mesh_object.mesh().vertices.attributes().is_defined("weight"));
        EXPECT_EQ(
            find_attribute(mesh_object, "vertices.weight").full_name(), "");

        // The other attributes are untouched.
        EXPECT_TRUE(
            mesh_object.mesh().facets.attributes().is_defined("normal"));

        // Removing it again reports that there is nothing left to remove.
        EXPECT_FALSE(TestAttributeManagerApplication::remove(
            mesh_object, weight));
    }

    /**
     * @brief The reset button puts the table back to the mesh as it is.
     * @details Nothing is applied to the mesh until the button below is
     *          pressed: dropping what the rows asked for is enough to show the
     *          mesh again, and the next "Perform" then changes nothing.
     */
    TEST(AttributeManagerApplicationTest, ResetsWhatTheRowsAskedFor) {
        GEO::Mesh mesh;
        make_triangle_mesh(mesh);
        {
            GEO::Attribute<double> weight(
                mesh.vertices.attributes(), "weight");
            for (GEO::index_t v = 0; v < mesh.vertices.nb(); ++v)
                weight[v] = double(v);
            GEO::Attribute<float> label(mesh.facets.attributes(), "label");
            for (GEO::index_t f = 0; f < mesh.facets.nb(); ++f)
                label[f] = float(f);
        }

        geobox::MeshObject mesh_object("mesh", "", mesh);
        geobox::ApplicationObjects objects;
        TestAttributeManagerApplication application(objects);

        const auto attributes = mesh_object.attribute_infos(true);
        ASSERT_EQ(attributes.size(), 3u);
        for (const auto& attribute : attributes) {
            if (attribute.full_name() == "facets.label")
                application.uncheck(attribute);
            else if (attribute.full_name() == "vertices.weight")
                application.set_dimension(attribute, 4);
        }

        application.reset();

        // Nothing is pending any more: the mesh is left as it is.
        application.perform(mesh_object, attributes);
        EXPECT_TRUE(mesh_object.mesh().facets.attributes().is_defined("label"));
        EXPECT_EQ(
            find_attribute(mesh_object, "vertices.weight").dimension, 1u);
    }

    /**
     * @brief The coordinates of the vertices are listed, and cannot be removed.
     * @details They are an attribute of the vertices store, so the manager
     *          shows them like the others -- but they are the geometry of the
     *          mesh: removing them would leave the mesh without a shape. Their
     *          dimension, however, is the dimension of the mesh, which GeoGram
     *          does change (GEO::MeshVertices::set_dimension() is this very
     *          resize).
     */
    TEST(AttributeManagerApplicationTest, KeepsTheGeometryButResizesIt) {
        GEO::Mesh mesh;
        make_triangle_mesh(mesh);
        {
            GEO::Attribute<double> weight(
                mesh.vertices.attributes(), "weight");
            for (GEO::index_t v = 0; v < mesh.vertices.nb(); ++v)
                weight[v] = double(v);
        }

        geobox::MeshObject mesh_object("mesh", "", mesh);

        // The coordinates are there, and flagged as the geometry.
        const auto point = find_attribute(
            mesh_object,
            mesh_object.mesh().vertices.single_precision()
                ? "vertices.point_fp32"
                : "vertices.point");
        ASSERT_FALSE(point.full_name().empty());
        EXPECT_TRUE(point.geometry);
        EXPECT_EQ(point.dimension, 3u);

        // The other attributes are not.
        EXPECT_FALSE(find_attribute(mesh_object, "vertices.weight").geometry);

        // Removing them is refused, and changes nothing.
        EXPECT_FALSE(
            TestAttributeManagerApplication::remove(mesh_object, point));
        EXPECT_TRUE(
            mesh_object.mesh().vertices.attributes().is_defined(point.name));

        // Resizing them changes the dimension of the mesh: it keeps the
        // coordinates it has, and zeroes the ones it gains.
        ASSERT_TRUE(
            TestAttributeManagerApplication::redim(mesh_object, point, 4));
        EXPECT_EQ(mesh_object.mesh().vertices.dimension(), 4u);
        EXPECT_EQ(
            find_attribute(mesh_object, point.full_name()).dimension, 4u);
        EXPECT_DOUBLE_EQ(mesh_object.mesh().vertices.point(0)[0], 0.0);
        // The coordinates are read through the pointer of the vertex: point()
        // returns the three first ones, whatever the dimension of the mesh.
        EXPECT_DOUBLE_EQ(mesh_object.mesh().vertices.point_ptr(0)[3], 0.0);

        // A row that asks for its removal anyway is ignored, but its dimension
        // is applied: that is what the window does (see
        // apply_pending_changes()).
        geobox::ApplicationObjects objects;
        TestAttributeManagerApplication application(objects);
        application.uncheck(point);
        application.set_dimension(point, 3);
        application.perform(mesh_object, mesh_object.attribute_infos(true));
        EXPECT_TRUE(
            mesh_object.mesh().vertices.attributes().is_defined(point.name));
        EXPECT_EQ(mesh_object.mesh().vertices.dimension(), 3u);
    }

    /**
     * @brief The dimension of an attribute can be changed from its row.
     * @details GeoGram resizes the attribute in place, keeping the values it
     *          already holds (see GEO::AttributeStore::redim()), so a scalar
     *          attribute can become a vector one, and the other way around.
     */
    TEST(AttributeManagerApplicationTest, ChangesTheDimensionOfAnAttribute) {
        GEO::Mesh mesh;
        make_triangle_mesh(mesh);
        {
            GEO::Attribute<double> weight(
                mesh.vertices.attributes(), "weight");
            weight[0] = 1.0;
            weight[1] = 2.0;
            weight[2] = 3.0;
        }

        geobox::MeshObject mesh_object("mesh", "", mesh);
        const auto weight = find_attribute(mesh_object, "vertices.weight");
        ASSERT_EQ(weight.dimension, 1u);

        ASSERT_TRUE(TestAttributeManagerApplication::redim(
            mesh_object, weight, 3));

        const auto resized = find_attribute(mesh_object, "vertices.weight");
        ASSERT_EQ(resized.full_name(), "vertices.weight");
        EXPECT_EQ(resized.dimension, 3u);

        // The values it already held are still there, one component each.
        const GEO::Attribute<double> read_back(
            mesh_object.mesh().vertices.attributes(), "weight");
        EXPECT_DOUBLE_EQ(read_back[0], 1.0);
        EXPECT_DOUBLE_EQ(read_back[3], 2.0);
        EXPECT_DOUBLE_EQ(read_back[6], 3.0);

        // Back to a scalar, and to the dimension it already has: the second
        // call has nothing to do.
        EXPECT_TRUE(TestAttributeManagerApplication::redim(
            mesh_object, resized, 1));
        EXPECT_FALSE(TestAttributeManagerApplication::redim(
            mesh_object, find_attribute(mesh_object, "vertices.weight"), 1));
        EXPECT_FALSE(TestAttributeManagerApplication::redim(
            mesh_object, find_attribute(mesh_object, "vertices.weight"), 0));
    }

    /**
     * @brief The button applies what the rows asked for, and only that.
     * @details An attribute whose box is unchecked is removed, an attribute
     *          whose dimension was typed is resized, and an attribute the user
     *          did not touch is left alone. Pressing the button again has
     *          nothing left to do.
     */
    TEST(AttributeManagerApplicationTest, AppliesWhatTheRowsAskFor) {
        GEO::Mesh mesh;
        make_triangle_mesh(mesh);
        {
            GEO::Attribute<double> weight(
                mesh.vertices.attributes(), "weight");
            for (GEO::index_t v = 0; v < mesh.vertices.nb(); ++v)
                weight[v] = double(v);
            GEO::Attribute<float> label(mesh.facets.attributes(), "label");
            for (GEO::index_t f = 0; f < mesh.facets.nb(); ++f)
                label[f] = float(f);
        }

        geobox::MeshObject mesh_object("mesh", "", mesh);
        geobox::ApplicationObjects objects;
        TestAttributeManagerApplication application(objects);

        const auto attributes = mesh_object.attribute_infos(true);
        // The coordinates of the vertices are listed too, and the "label" and
        // "weight" attributes are the ones the rows work on.
        ASSERT_EQ(attributes.size(), 3u);
        for (const auto& attribute : attributes) {
            if (attribute.full_name() == "facets.label")
                application.uncheck(attribute);
            else if (attribute.full_name() == "vertices.weight")
                application.set_dimension(attribute, 3);
        }

        application.perform(mesh_object, attributes);

        EXPECT_FALSE(
            mesh_object.mesh().facets.attributes().is_defined("label"));
        const auto weight = find_attribute(mesh_object, "vertices.weight");
        ASSERT_EQ(weight.full_name(), "vertices.weight");
        EXPECT_EQ(weight.dimension, 3u);

        // Nothing is pending any more: the next run leaves the mesh as it is.
        const auto remaining = mesh_object.attribute_infos(true);
        ASSERT_EQ(remaining.size(), 2u);
        application.perform(mesh_object, remaining);
        EXPECT_EQ(
            find_attribute(mesh_object, "vertices.weight").dimension, 3u);
    }

    /**
     * @brief The window lists the attributes of the selected mesh, and draws
     *        one remove button per row.
     * @details Drawn headlessly (no renderer): what matters is that the rows
     *          are laid out, for the mesh the picker selected, without a crash.
     */
    TEST(AttributeManagerApplicationTest, DrawsTheAttributesOfTheSelectedMesh) {
        ImGui::CreateContext();
        ImGuiIO& io = ImGui::GetIO();
        io.DisplaySize = ImVec2(1280.0f, 720.0f);
        io.DeltaTime = 1.0f / 60.0f;
        io.IniFilename = nullptr;
        io.LogFilename = nullptr;
        io.BackendFlags |= ImGuiBackendFlags_RendererHasTextures;

        GEO::Mesh mesh;
        make_triangle_mesh(mesh);
        {
            GEO::Attribute<double> weight(
                mesh.vertices.attributes(), "weight");
            for (GEO::index_t v = 0; v < mesh.vertices.nb(); ++v)
                weight[v] = double(v);
        }

        geobox::ApplicationObjects objects;
        objects.push_back(
            std::make_shared<geobox::MeshObject>("mesh", "", mesh));
        TestAttributeManagerApplication application(objects);

        // The single mesh of the scene is picked automatically, and its
        // attributes are listed.
        ImGui::NewFrame();
        application.draw_window();
        ImGui::EndFrame();

        ImGui::DestroyContext();
    }
}
