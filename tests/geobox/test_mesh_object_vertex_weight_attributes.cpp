//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <vector>
#include <geogram/basic/attributes.h>
#include <geogram/mesh/mesh.h>
#include <geolio/geobox/object/mesh_object.h>

namespace geolio::test
{
    namespace
    {
        /**
         * @brief MeshObject with the name of the display-only attributes the
         *        listing under test has to hide made observable.
         */
        class TestWeightAttributeMeshObject : public geobox::MeshObject {
        public:
            TestWeightAttributeMeshObject(
                const std::string& name, const GEO::Mesh& mesh)
                : MeshObject(name, "", mesh) {
            }

            /// Name of the display copy of the displayed attribute.
            static std::string display_attribute_name() {
                return DISPLAY_ATTRIBUTE_NAME;
            }

            /// Name of the display copy of the scaled texture coordinates.
            static std::string display_tex_coord_name() {
                return DISPLAY_TEX_COORD_NAME;
            }

            /// Whether an attribute can weight the sampling: one double per
            /// vertex (the filter of MeshRandomSamplingApplication).
            static bool is_weight_attribute(
                const geobox::MeshObject::AttributeInfo& attribute
                ) {
                return attribute.subelements == GEO::MESH_VERTICES &&
                       attribute.holds<double>() &&
                       attribute.dimension == 1;
            }

            /// Full names ("vertices.weight") of the attributes that can weight
            /// the sampling, i.e. the ones a selector with that filter offers.
            std::vector<std::string> weight_attributes() const {
                std::vector<std::string> result;
                for (const auto& attribute : attribute_infos()) {
                    if (is_weight_attribute(attribute))
                        result.push_back(attribute.full_name());
                }
                return result;
            }
        };

        /**
         * @brief Fills \p mesh with a single triangle, i.e. a surface the
         *        sampler accepts.
         */
        void make_triangle_mesh(GEO::Mesh& mesh) {
            mesh.vertices.set_dimension(3);
            mesh.vertices.create_vertices(3);
            mesh.vertices.point(0) = GEO::vec3(0.0, 0.0, 0.0);
            mesh.vertices.point(1) = GEO::vec3(1.0, 0.0, 0.0);
            mesh.vertices.point(2) = GEO::vec3(0.0, 1.0, 0.0);
            mesh.facets.create_triangle(0, 1, 2);
        }

        /**
         * @brief Creates a per-vertex double attribute holding \p values.
         */
        void create_weight(
            GEO::Mesh& mesh,
            const std::string& name,
            const std::vector<double>& values
            ) {
            GEO::Attribute<double> weight(
                mesh.vertices.attributes(), name);
            for (GEO::index_t v = 0; v < mesh.vertices.nb(); ++v)
                weight[v] = values[static_cast<std::size_t>(v)];
        }
    }

    /**
     * @brief The weight selector only offers the per-vertex attributes holding
     *        one double per vertex.
     * @details Those are the only ones that can be bound to the
     *          GEO::Attribute<double> the sampler takes: an int attribute is
     *          rejected by the attribute type check, a vector attribute holds
     *          several components per vertex, and an attribute on another
     *          sub-element (facets, cells) has one value per facet or per cell
     *          instead of one per vertex.
     */
    TEST(MeshObjectVertexWeightAttributesTest, ListsOnlyVertexDoubleAttributes) {
        GEO::Mesh mesh;
        make_triangle_mesh(mesh);
        // The one the selector has to offer.
        create_weight(mesh, "weight", {0.0, 1.0, 2.0});
        // The ones it has to leave out.
        GEO::Attribute<int> id(mesh.vertices.attributes(), "v_id");
        for (GEO::index_t v = 0; v < mesh.vertices.nb(); ++v)
            id[v] = static_cast<int>(v);
        GEO::Attribute<GEO::vec3> direction(mesh.vertices.attributes(), "v_dir");
        for (GEO::index_t v = 0; v < mesh.vertices.nb(); ++v)
            direction[v] = GEO::vec3(mesh.vertices.point(v));
        // A double attribute with three components per vertex.
        GEO::Attribute<double> displacements;
        displacements.create_vector_attribute(
            mesh.vertices.attributes(), "v_disp", 3);
        // A double attribute on another sub-element (one value per facet).
        GEO::Attribute<double> facet_area(mesh.facets.attributes(), "f_area");
        facet_area[0] = 0.5;

        const TestWeightAttributeMeshObject object("mesh", mesh);
        EXPECT_EQ(
            object.weight_attributes(),
            std::vector<std::string>({"vertices.weight"}));
    }

    /**
     * @brief The listing describes where an attribute lives and what it holds.
     * @details This is what an application filters on: the sub-element, the
     *          element type and the number of components per element.
     */
    TEST(MeshObjectVertexWeightAttributesTest, DescribesAttributes) {
        GEO::Mesh mesh;
        make_triangle_mesh(mesh);
        create_weight(mesh, "weight", {0.0, 1.0, 2.0});
        GEO::Attribute<double> facet_area(mesh.facets.attributes(), "f_area");
        facet_area[0] = 0.5;

        const TestWeightAttributeMeshObject object("mesh", mesh);
        const std::vector<geobox::MeshObject::AttributeInfo> attributes =
            object.attribute_infos();
        ASSERT_EQ(attributes.size(), 2);

        // The attributes are grouped by sub-element, in the order of the mesh
        // stores: the vertices come before the facets.
        EXPECT_EQ(attributes[0].subelements, GEO::MESH_VERTICES);
        EXPECT_EQ(attributes[0].name, "weight");
        EXPECT_EQ(attributes[0].full_name(), "vertices.weight");
        EXPECT_TRUE(attributes[0].holds<double>());
        EXPECT_FALSE(attributes[0].holds<GEO::vec3>());
        EXPECT_EQ(attributes[0].dimension, 1);
        EXPECT_EQ(attributes[0].nb_elements, mesh.vertices.nb());

        EXPECT_EQ(attributes[1].subelements, GEO::MESH_FACETS);
        EXPECT_EQ(attributes[1].full_name(), "facets.f_area");
        EXPECT_EQ(attributes[1].nb_elements, mesh.facets.nb());
    }

    /**
     * @brief The display copies the object builds for the renderer are not
     *        listed as attributes of the mesh.
     * @details They hold one double per vertex, but they carry what the current
     *          display settings asked for, not data of the mesh: the GUI hides
     *          them everywhere else too (see MeshObject::attribute_names()).
     */
    TEST(MeshObjectVertexWeightAttributesTest, HidesDisplayCopies) {
        GEO::Mesh mesh;
        make_triangle_mesh(mesh);
        create_weight(mesh, "weight", {0.0, 1.0, 2.0});
        create_weight(
            mesh, TestWeightAttributeMeshObject::display_attribute_name(),
            {0.0, 0.5, 1.0});
        create_weight(
            mesh, TestWeightAttributeMeshObject::display_tex_coord_name(),
            {0.0, 0.5, 1.0});

        const TestWeightAttributeMeshObject object("mesh", mesh);
        EXPECT_EQ(
            object.weight_attributes(),
            std::vector<std::string>({"vertices.weight"}));
    }

    /**
     * @brief A mesh without attributes offers no weight at all.
     * @details The selector of the sampling application then shows its "no
     *          attribute available" message instead of a combo box.
     */
    TEST(MeshObjectVertexWeightAttributesTest, EmptyWithoutAttributes) {
        GEO::Mesh mesh;
        make_triangle_mesh(mesh);
        const TestWeightAttributeMeshObject object("mesh", mesh);
        EXPECT_TRUE(object.weight_attributes().empty());
        EXPECT_TRUE(object.attribute_infos().empty());
    }

    /**
     * @brief The attribute the selector offers can be bound to the double
     *        attribute the sampler weights the sampling with.
     * @details This is what MeshRandomSamplingApplication::perform() does
     *          before sampling: binding fails (and the sampling is then
     *          unweighted) only for the attributes the selector does not offer
     *          either.
     */
    TEST(MeshObjectVertexWeightAttributesTest, SelectedAttributeBinds) {
        GEO::Mesh mesh;
        make_triangle_mesh(mesh);
        create_weight(mesh, "weight", {0.5, 1.5, 2.5});
        GEO::Attribute<int> id(mesh.vertices.attributes(), "v_id");
        for (GEO::index_t v = 0; v < mesh.vertices.nb(); ++v)
            id[v] = static_cast<int>(v);

        const TestWeightAttributeMeshObject object("mesh", mesh);
        ASSERT_EQ(
            object.weight_attributes(),
            std::vector<std::string>({"vertices.weight"}));

        GEO::Attribute<double> vertex_weight;
        ASSERT_TRUE(vertex_weight.bind_if_is_compatible(
            mesh.vertices.attributes(), "weight"));
        EXPECT_TRUE(vertex_weight.is_bound());
        EXPECT_DOUBLE_EQ(vertex_weight[1], 1.5);

        // The attributes that are not offered cannot be bound: the sampling
        // would then fall back to area-proportional sampling.
        GEO::Attribute<double> not_a_weight;
        EXPECT_FALSE(not_a_weight.bind_if_is_compatible(
            mesh.vertices.attributes(), "v_id"));
        EXPECT_FALSE(not_a_weight.is_bound());

        vertex_weight.unbind();
    }
}
