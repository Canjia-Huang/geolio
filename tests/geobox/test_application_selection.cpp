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
#include <geogram_gfx/third_party/imgui/imgui.h>
#include <geolio/geobox/application/base_application.h>
#include <geolio/geobox/object/directional_object.h>
#include <geolio/geobox/object/mesh_object.h>

namespace geolio::test
{
    namespace
    {
        /**
         * @brief BaseApplication with the selection helpers exposed.
         * @details The helpers are what the applications share (see
         *          BaseApplication); the window body is irrelevant here.
         */
        class TestSelectionApplication : public geobox::BaseApplication {
        public:
            explicit TestSelectionApplication(geobox::ApplicationObjects& objects)
                : BaseApplication("test", objects) {
            }

            /// Mesh filter of an application that needs surface facets.
            static bool has_facets(const GEO::Mesh& mesh) {
                return mesh.facets.nb() > 0;
            }

            /// Draws the picker of \p selection, i.e. one of the meshes this
            /// application works on.
            bool draw_mesh(
                std::weak_ptr<geobox::MeshObject>& selection,
                const MeshFilter& filter,
                const char* label = "Mesh"
                ) {
                return draw_mesh_selector(selection, filter, label);
            }

            /// Draws the picker of \p selection, i.e. one of the directional
            /// fields this application works on.
            bool draw_directional(
                std::weak_ptr<geobox::DirectionalObject>& selection,
                const DirectionalFilter& filter,
                const char* label = "Field"
                ) {
                return draw_directional_selector(selection, filter, label);
            }

            bool draw_attribute(
                geobox::MeshObject::AttributeInfo& selection,
                const geobox::MeshObject& mesh_object,
                const AttributeFilter& filter
                ) {
                return draw_attribute_selector(selection, mesh_object, filter);
            }

        protected:
            void draw_window_contents() override {}
        };

        /**
         * @brief A triangle, i.e. a mesh an application accepting surfaces
         *        works on.
         */
        void make_triangle_mesh(GEO::Mesh& mesh) {
            mesh.vertices.set_dimension(3);
            mesh.vertices.create_vertices(3);
            mesh.vertices.point(0) = GEO::vec3(0.0, 0.0, 0.0);
            mesh.vertices.point(1) = GEO::vec3(1.0, 0.0, 0.0);
            mesh.vertices.point(2) = GEO::vec3(0.0, 1.0, 0.0);
            mesh.facets.create_triangle(0, 1, 2);
        }

        /// A mesh with vertices only, i.e. one an application that needs
        /// surface facets rejects.
        void make_point_cloud_mesh(GEO::Mesh& mesh) {
            mesh.vertices.set_dimension(3);
            mesh.vertices.create_vertices(1);
            mesh.vertices.point(0) = GEO::vec3(0.0, 0.0, 0.0);
        }

        std::shared_ptr<geobox::MeshObject> make_object(
            const std::string& name, const GEO::Mesh& mesh) {
            return std::make_shared<geobox::MeshObject>(name, "", mesh);
        }

        /// A directional field of \p nb_frames frames of three unit vectors,
        /// i.e. what an application that works on fields picks.
        std::shared_ptr<geobox::DirectionalObject> make_field(
            const std::string& name,
            const GEO::index_t nb_frames = 1) {
            std::vector<double> vectors;
            for (GEO::index_t f = 0; f < nb_frames; ++f) {
                const double frame[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
                vectors.insert(vectors.end(), frame, frame + 9);
            }
            return std::make_shared<geobox::DirectionalObject>(
                name, "", 3, vectors);
        }

        /// Only the per-vertex double attributes, i.e. the weight of the
        /// sampling application.
        bool is_vertex_double(const geobox::MeshObject::AttributeInfo& attribute) {
            return attribute.subelements == GEO::MESH_VERTICES &&
                   attribute.holds<double>() &&
                   attribute.dimension == 1;
        }
    }

    /**
     * @brief The selection helpers draw ImGui widgets, so the tests run them
     *        inside a frame of a (backend-less) ImGui context: the widgets are
     *        laid out, but nothing is rendered.
     * @details What matters here is the state the applications keep: which
     *          mesh object each picker selects, and which attribute.
     */
    class ApplicationSelectionTest : public ::testing::Test {
    protected:
        void SetUp() override {
            ImGui::CreateContext();
            ImGuiIO& io = ImGui::GetIO();
            io.DisplaySize = ImVec2(1280.0f, 720.0f);
            io.DeltaTime = 1.0f / 60.0f;
            io.IniFilename = nullptr;
            io.LogFilename = nullptr;
            // No renderer backend here: tell ImGui it may request the font
            // textures lazily, which is what it needs to lay the widgets out
            // without an atlas built by a backend (see ImGui 1.92 fonts).
            io.BackendFlags |= ImGuiBackendFlags_RendererHasTextures;
        }

        void TearDown() override {
            ImGui::DestroyContext();
        }

        /**
         * @brief Runs one frame with @p draw called inside a window, and
         *        returns what it returned.
         */
        template <class Draw>
        static bool frame(Draw draw) {
            ImGui::NewFrame();
            ImGui::Begin("test");
            const bool result = draw();
            ImGui::End();
            ImGui::EndFrame();
            return result;
        }

    };

    /**
     * @brief A single candidate is selected automatically, so that an
     *        application whose scene holds one mesh is usable right away.
     * @details The candidates are the meshes the filter accepts: the point
     *          cloud is not selected even though it is the only other object.
     */
    TEST_F(ApplicationSelectionTest, SelectsSingleCandidate) {
        geobox::ApplicationObjects objects;
        GEO::Mesh cloud_mesh;
        make_point_cloud_mesh(cloud_mesh);
        objects.push_back(make_object("cloud", cloud_mesh));

        TestSelectionApplication application(objects);
        std::weak_ptr<geobox::MeshObject> selection;

        EXPECT_FALSE(frame([&] {
            return application.draw_mesh(
                selection, &TestSelectionApplication::has_facets);
        }));
        // No candidate: nothing to select.
        EXPECT_EQ(selection.lock(), nullptr);

        GEO::Mesh surface_mesh;
        make_triangle_mesh(surface_mesh);
        objects.push_back(make_object("surface", surface_mesh));

        EXPECT_TRUE(frame([&] {
            return application.draw_mesh(
                selection, &TestSelectionApplication::has_facets);
        }));
        ASSERT_NE(selection.lock(), nullptr);
        EXPECT_EQ(selection.lock()->name(), "surface");

        // The selection is kept, and reported as unchanged, on the next frame.
        EXPECT_FALSE(frame([&] {
            return application.draw_mesh(
                selection, &TestSelectionApplication::has_facets);
        }));
        ASSERT_NE(selection.lock(), nullptr);
        EXPECT_EQ(selection.lock()->name(), "surface");
    }

    /**
     * @brief Two pickers of the same application keep two independent
     *        selections.
     * @details The reason the selections belong to the applications
     *          (CVTApplication picks a base surface and, separately, the mesh
     *          holding its samples): drawing the second picker must not touch
     *          what the first one selected.
     */
    TEST_F(ApplicationSelectionTest, KeepsOneSelectionPerPicker) {
        geobox::ApplicationObjects objects;
        GEO::Mesh surface_mesh;
        make_triangle_mesh(surface_mesh);
        objects.push_back(make_object("first", surface_mesh));
        objects.push_back(make_object("second", surface_mesh));

        TestSelectionApplication application(objects);
        std::weak_ptr<geobox::MeshObject> first;
        std::weak_ptr<geobox::MeshObject> second;

        // Two candidates each: the user has to pick, nothing is automatic.
        EXPECT_FALSE(frame([&] {
            return application.draw_mesh(first, {}, "First");
        }));
        EXPECT_FALSE(frame([&] {
            return application.draw_mesh(second, {}, "Second");
        }));
        EXPECT_EQ(first.lock(), nullptr);
        EXPECT_EQ(second.lock(), nullptr);
    }

    /**
     * @brief A selected mesh that stops matching the filter drops the
     *        selection.
     * @details The reported scenario: the operation of an application changes
     *          the mesh (here: it loses its facets), so the mesh must not stay
     *          selected and the parameters of the application, which are only
     *          drawn for a selected mesh, must be disabled.
     */
    TEST_F(ApplicationSelectionTest, DropsSelectionWhenMeshStopsMatching) {
        geobox::ApplicationObjects objects;
        GEO::Mesh surface_mesh;
        make_triangle_mesh(surface_mesh);
        objects.push_back(make_object("surface", surface_mesh));

        TestSelectionApplication application(objects);
        std::weak_ptr<geobox::MeshObject> selection;
        ASSERT_TRUE(frame([&] {
            return application.draw_mesh(
                selection, &TestSelectionApplication::has_facets);
        }));
        ASSERT_NE(selection.lock(), nullptr);

        // The mesh becomes one the application cannot work on.
        const std::shared_ptr<geobox::MeshObject> mesh_object = selection.lock();
        mesh_object->mesh().facets.clear();
        EXPECT_EQ(mesh_object->mesh().facets.nb(), 0);

        EXPECT_TRUE(frame([&] {
            return application.draw_mesh(
                selection, &TestSelectionApplication::has_facets);
        }));
        EXPECT_EQ(selection.lock(), nullptr);
    }

    /**
     * @brief The attribute picker offers the attributes the filter keeps, and
     *        selects the first one of them until the user picks another.
     */
    TEST_F(ApplicationSelectionTest, AttributeSelectorUsesTheFilter) {
        GEO::Mesh mesh;
        make_triangle_mesh(mesh);
        GEO::Attribute<double> weight(mesh.vertices.attributes(), "weight");
        weight[0] = 1.0;
        GEO::Attribute<int> id(mesh.vertices.attributes(), "v_id");
        id[0] = 0;
        GEO::Attribute<double> facet_area(mesh.facets.attributes(), "f_area");
        facet_area[0] = 0.5;
        const geobox::MeshObject mesh_object("mesh", "", mesh);

        geobox::ApplicationObjects objects;
        TestSelectionApplication application(objects);
        geobox::MeshObject::AttributeInfo selection;

        // The first attribute the filter keeps is selected: "vertices.weight",
        // neither the integer "v_id" (wrong element type) nor the facet
        // attribute (wrong sub-element).
        EXPECT_TRUE(frame([&] {
            return application.draw_attribute(
                selection, mesh_object, &is_vertex_double);
        }));
        EXPECT_EQ(selection.full_name(), "vertices.weight");

        // The selection is kept, and reported as unchanged, on the next frame.
        EXPECT_FALSE(frame([&] {
            return application.draw_attribute(
                selection, mesh_object, &is_vertex_double);
        }));
        EXPECT_EQ(selection.full_name(), "vertices.weight");

        // Two attributes matched: a second picker keeps its own selection.
        geobox::MeshObject::AttributeInfo other_selection;
        EXPECT_TRUE(frame([&] {
            return application.draw_attribute(
                other_selection, mesh_object, &is_vertex_double);
        }));
        EXPECT_EQ(other_selection.full_name(), "vertices.weight");
        EXPECT_EQ(selection.full_name(), "vertices.weight");
    }

    /**
     * @brief A selected attribute that the mesh does not offer any more falls
     *        back to the first attribute the filter keeps.
     * @details The reported case: another mesh was picked, or the mesh was
     *          reloaded without that attribute; the application must not keep
     *          working with an attribute that is gone.
     */
    TEST_F(ApplicationSelectionTest, AttributeSelectorFallsBackWhenGone) {
        geobox::ApplicationObjects objects;
        GEO::Mesh mesh;
        make_triangle_mesh(mesh);
        const auto mesh_object = make_object("mesh", mesh);
        objects.push_back(mesh_object);

        TestSelectionApplication application(objects);
        geobox::MeshObject::AttributeInfo selection;

        EXPECT_FALSE(frame([&] {
            return application.draw_attribute(
                selection, *mesh_object, &is_vertex_double);
        }));
        // No attribute at all in the mesh: the picker reports nothing to pick,
        // and keeps the selection empty.
        EXPECT_EQ(selection.full_name(), "");

        {
            GEO::Attribute<double> weight(
                mesh_object->mesh().vertices.attributes(), "weight");
            weight[0] = 1.0;
        }
        EXPECT_TRUE(frame([&] {
            return application.draw_attribute(
                selection, *mesh_object, &is_vertex_double);
        }));
        EXPECT_EQ(selection.full_name(), "vertices.weight");

        // The attribute disappears from the mesh: the application must not keep
        // working with an attribute that is gone.
        mesh_object->mesh().vertices.attributes().delete_attribute_store("weight");
        EXPECT_TRUE(frame([&] {
            return application.draw_attribute(
                selection, *mesh_object, &is_vertex_double);
        }));
        EXPECT_EQ(selection.full_name(), "");
    }

    /**
     * @brief A single directional field is selected automatically, like a single
     *        mesh.
     * @details The fields of the scene are listed by name; the selection is kept
     *          from one frame to the next, so the parameters of the application
     *          that depend on it do not reset (see draw_directional_selector()).
     */
    TEST_F(ApplicationSelectionTest, SelectsTheSingleDirectionalField) {
        geobox::ApplicationObjects objects;
        GEO::Mesh surface_mesh;
        make_triangle_mesh(surface_mesh);
        objects.push_back(make_object("surface", surface_mesh));

        TestSelectionApplication application(objects);
        std::weak_ptr<geobox::DirectionalObject> selection;

        // No field in the scene yet: nothing to select.
        EXPECT_FALSE(frame([&] {
            return application.draw_directional(selection, {}, "Field");
        }));
        EXPECT_EQ(selection.lock(), nullptr);

        const auto field = make_field("frames");
        objects.push_back(field);

        EXPECT_TRUE(frame([&] {
            return application.draw_directional(selection, {}, "Field");
        }));
        ASSERT_NE(selection.lock(), nullptr);
        EXPECT_EQ(selection.lock(), field);

        // The selection is kept, and reported as unchanged, on the next frame.
        EXPECT_FALSE(frame([&] {
            return application.draw_directional(selection, {}, "Field");
        }));
        EXPECT_EQ(selection.lock(), field);
    }

    /**
     * @brief Two objects that share a name are still two different objects.
     * @details Running an operation twice gives two objects with the same name,
     *          and so does loading the same file twice. The pickers label their
     *          items with that name, so they identify them with
     *          BaseObject::unique_id() (see draw_mesh_selector()); what the
     *          selectors keep is the object itself, not its name, so a selection
     *          of the second one survives the frames that follow.
     */
    TEST_F(ApplicationSelectionTest, KeepsObjectsThatShareANameApart) {
        geobox::ApplicationObjects objects;
        GEO::Mesh surface_mesh;
        make_triangle_mesh(surface_mesh);
        const auto first = make_object("same name", surface_mesh);
        const auto second = make_object("same name", surface_mesh);
        objects.push_back(first);
        objects.push_back(second);

        // The name is not an identifier: the unique id is.
        EXPECT_EQ(first->name(), second->name());
        EXPECT_NE(first->unique_id(), second->unique_id());

        TestSelectionApplication application(objects);
        std::weak_ptr<geobox::MeshObject> selection;

        // Two candidates of the same name: the user has to pick one, and the
        // picker offers both of them.
        EXPECT_FALSE(frame([&] {
            return application.draw_mesh(selection, {}, "Mesh");
        }));
        EXPECT_EQ(selection.lock(), nullptr);

        // Whatever was picked is kept, even though another object has the same
        // name: the selection is the object, not the name it was looked up by.
        selection = second;
        EXPECT_FALSE(frame([&] {
            return application.draw_mesh(selection, {}, "Mesh");
        }));
        EXPECT_EQ(selection.lock(), second);
        EXPECT_NE(selection.lock(), first);

        // And when that object leaves the scene, the selection goes with it
        // instead of falling back to the object of the same name.
        std::erase(objects, second);
        EXPECT_TRUE(frame([&] {
            return application.draw_mesh(selection, {}, "Mesh");
        }));
        EXPECT_EQ(selection.lock(), nullptr);
    }

    /**
     * @brief The field picker only offers the fields the filter accepts, and
     *        drops a selection whose field it does not accept any more.
     * @details That is how an application that needs, say, a field of a given
     *          number of frames shows only the ones it can work on.
     */
    TEST_F(ApplicationSelectionTest, FiltersTheDirectionalFieldsItOffers) {
        geobox::ApplicationObjects objects;

        // Only the fields of one frame can be picked.
        const auto single = make_field("single", 1);
        const auto many = make_field("many", 2);
        objects.push_back(single);
        objects.push_back(many);

        TestSelectionApplication application(objects);
        std::weak_ptr<geobox::DirectionalObject> selection;
        const auto one_frame = [](const geobox::DirectionalObject& field) {
            return field.nb_frames() == 1;
        };

        EXPECT_TRUE(frame([&] {
            return application.draw_directional(selection, one_frame, "Field");
        }));
        ASSERT_NE(selection.lock(), nullptr);
        EXPECT_EQ(selection.lock(), single);

        // A second field of one frame is a second candidate: the selection is
        // kept, and the user has to pick between the two.
        const auto other = make_field("other", 1);
        objects.push_back(other);
        EXPECT_FALSE(frame([&] {
            return application.draw_directional(selection, one_frame, "Field");
        }));
        EXPECT_EQ(selection.lock(), single);

        // The selected field leaves the scene: the selection is dropped.
        std::erase(objects, single);
        EXPECT_TRUE(frame([&] {
            return application.draw_directional(selection, one_frame, "Field");
        }));
        EXPECT_EQ(selection.lock(), nullptr);

        // And when no field at all matches, the picker says so and stays empty.
        EXPECT_FALSE(frame([&] {
            return application.draw_directional(
                selection,
                [](const geobox::DirectionalObject& field) {
                    return field.nb_frames() == 7;
                },
                "Field");
        }));
        EXPECT_EQ(selection.lock(), nullptr);
    }
}
