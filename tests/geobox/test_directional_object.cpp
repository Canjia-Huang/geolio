//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/31.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <gtest/gtest.h>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>
#include <geogram/mesh/mesh.h>
#include <geogram_gfx/third_party/imgui/imgui.h>
#include <geolio/directional/frame_field.h>
#include <geolio/geobox/object/directional_object.h>
#include <geolio/geobox/object/mesh_object.h>

namespace geolio::test
{
    namespace
    {
        /// A square made of two triangles in the z = 0 plane.
        void make_square_mesh(GEO::Mesh& mesh) {
            mesh.vertices.set_dimension(3);
            mesh.vertices.create_vertices(4);
            mesh.vertices.point(0) = GEO::vec3(0.0, 0.0, 0.0);
            mesh.vertices.point(1) = GEO::vec3(1.0, 0.0, 0.0);
            mesh.vertices.point(2) = GEO::vec3(1.0, 1.0, 0.0);
            mesh.vertices.point(3) = GEO::vec3(0.0, 1.0, 0.0);
            mesh.facets.create_triangle(0, 1, 2);
            mesh.facets.create_triangle(0, 2, 3);
        }

        /**
         * @brief The frame of element @p element: three unit vectors along the
         *        axes, which is what a frame field stores.
         */
        void append_frame(std::vector<double>& vectors) {
            const double frame[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
            vectors.insert(vectors.end(), frame, frame + 9);
        }

        /// A scene holding one mesh object made from \p mesh, for the fields the
        /// tests attach to it.
        struct TestScene {
            explicit TestScene(const GEO::Mesh& mesh) {
                mesh_object = std::make_shared<geobox::MeshObject>(
                    "mesh", "", mesh);
                objects.push_back(mesh_object);
            }

            geobox::ApplicationObjects objects;
            std::shared_ptr<geobox::MeshObject> mesh_object;
        };

        /**
         * @brief The object, with the scaling it draws with exposed.
         * @details The scale is protected because it is an implementation
         *          detail of the drawing; the tests need it to check the lengths
         *          the vectors are drawn with rather than their directions.
         */
        class ScaledDirectionalObject : public geobox::DirectionalObject {
        public:
            using DirectionalObject::DirectionalObject;

            [[nodiscard]] double scale() const {
                return vector_scale_in_model_units();
            }

            [[nodiscard]] double longest() const {
                return length_in_model_units();
            }
        };

        /// Path of a file unique to \p test_name, removed by the tests.
        std::string field_test_file(const std::string& test_name) {
            return (
                std::filesystem::temp_directory_path() /
                ("geolio_" + test_name + ".fra")
            ).string();
        }
    }

    /**
     * @brief The object draws one edge per vector, from the center of the element
     *        the vector belongs to, along its direction.
     * @details A frame field stores three vectors per element, and the object
     *          shows each of them as an edge of its visualization mesh, as long as
     *          the vector: the overall scale is applied while drawing, so that it
     *          can be changed without rebuilding the mesh.
     */
    TEST(DirectionalObjectTest, VisualizesOneEdgePerVector) {
        GEO::Mesh source_mesh;
        make_square_mesh(source_mesh);

        constexpr GEO::index_t NB_VECTORS_PER_ELEMENT = 3;
        std::vector<double> vectors;
        for (GEO::index_t f = 0; f < source_mesh.facets.nb(); ++f)
            append_frame(vectors);

        TestScene scene(source_mesh);
        geobox::DirectionalObject object(
            "field", "", NB_VECTORS_PER_ELEMENT, vectors);
        ASSERT_TRUE(object.attach_to(scene.mesh_object, GEO::MESH_FACETS));

        EXPECT_EQ(object.nb_frames(), source_mesh.facets.nb());
        EXPECT_EQ(object.nb_elements(), source_mesh.facets.nb());
        EXPECT_EQ(object.vectors_nb_per_element(), NB_VECTORS_PER_ELEMENT);
        EXPECT_EQ(
            object.mesh().edges.nb(),
            source_mesh.facets.nb() * NB_VECTORS_PER_ELEMENT);
        EXPECT_EQ(
            object.mesh().vertices.nb(),
            2 * source_mesh.facets.nb() * NB_VECTORS_PER_ELEMENT);

        // The first edge of the first element starts at its center -- the first
        // triangle is (0, 0), (1, 0), (1, 1) -- and points along the first
        // vector of its frame (the x axis).
        const double* start = object.mesh().vertices.point_ptr(0);
        const double* direction = object.mesh().vertices.point_ptr(1);
        EXPECT_DOUBLE_EQ(start[0], 2.0 / 3.0);
        EXPECT_DOUBLE_EQ(start[1], 1.0 / 3.0);
        EXPECT_DOUBLE_EQ(start[2], 0.0);
        EXPECT_NEAR(direction[0] - start[0], 1.0, 1e-12);
        EXPECT_NEAR(direction[1] - start[1], 0.0, 1e-12);

        // Every axis has its own style.
        EXPECT_FLOAT_EQ(object.axis_style(0).color.x, 0.90f);
        EXPECT_NE(object.axis_style(0).color.x, object.axis_style(1).color.x);
        EXPECT_GT(object.length(), 0.0f);
        EXPECT_GT(object.width(), 0.0f);

        // The length of the vectors is a fraction of the diagonal of the mesh the
        // field is defined on: the unit square below has a diagonal of sqrt(2).
        EXPECT_NEAR(object.bbox_diagonal(), std::sqrt(2.0), 1e-6);

        // The bounding box holds the vectors, whose length is a fraction of that
        // diagonal.
        double xyzmin[3];
        double xyzmax[3];
        object.get_bbox(xyzmin, xyzmax);
        EXPECT_LE(xyzmin[0], start[0]);
        EXPECT_GE(xyzmax[0], start[0]);
        EXPECT_GT(xyzmax[2] - xyzmin[2], 0.0);
    }

    /**
     * @brief Each axis reads the vector it should from the input.
     * @details The input lists the N vectors of an element consecutively, one
     *          element after the other, while the visualization stores its edges
     *          axis by axis (see mesh()). Reading the input axis by axis would
     *          take the vectors of other elements, which point somewhere else.
     */
    TEST(DirectionalObjectTest, ReadsTheVectorsOfEachAxisOfEachElement) {
        GEO::Mesh source_mesh;
        make_square_mesh(source_mesh);

        // Facet 0: x, y, z. Facet 1: -x, -y, -z.
        std::vector<double> vectors = {
            1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
            -1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0
        };

        TestScene scene(source_mesh);
        geobox::DirectionalObject object("field", "", 3, vectors);
        ASSERT_TRUE(object.attach_to(scene.mesh_object, GEO::MESH_FACETS));

        ASSERT_EQ(object.mesh().edges.nb(), 6);
        // Edge of axis 0 of element 1: the -x vector of that element.
        const GEO::index_t edge = 0 * 2 + 1;
        const double* start =
            object.mesh().vertices.point_ptr(object.mesh().edges.vertex(edge, 0));
        const double* tip =
            object.mesh().vertices.point_ptr(object.mesh().edges.vertex(edge, 1));
        EXPECT_NEAR(tip[0] - start[0], -1.0, 1e-12);
        EXPECT_NEAR(tip[1] - start[1], 0.0, 1e-12);
        EXPECT_NEAR(tip[2] - start[2], 0.0, 1e-12);

        // And the three axes of that element are the three negative directions.
        for (GEO::index_t axis = 0; axis < 3; ++axis) {
            const GEO::index_t a = axis * 2 + 1;
            const double* from = object.mesh().vertices.point_ptr(
                object.mesh().edges.vertex(a, 0));
            const double* to = object.mesh().vertices.point_ptr(
                object.mesh().edges.vertex(a, 1));
            EXPECT_NEAR(to[axis] - from[axis], -1.0, 1e-12);
        }
    }

    /**
     * @brief The field is written to, and read back from, a .fra file.
     * @details That is what the "Save as..." of the object does, and what reload()
     *          reads: the format holds the vectors only, so the geometry is kept.
     */
    TEST(DirectionalObjectTest, SavesAndReloadsAFraFile) {
        const std::string filepath = field_test_file("SavesAndReloadsAFraFile");

        GEO::Mesh source_mesh;
        make_square_mesh(source_mesh);

        std::vector<double> vectors;
        append_frame(vectors);
        append_frame(vectors);

        TestScene scene(source_mesh);
        geobox::DirectionalObject object("field", filepath, 3, vectors);
        ASSERT_TRUE(object.attach_to(scene.mesh_object, GEO::MESH_FACETS));

        // The .fra format is the only one the object writes.
        EXPECT_TRUE(object.can_write_extension("fra"));
        EXPECT_FALSE(object.can_write_extension("obj"));
        ASSERT_TRUE(object.save(filepath));
        ASSERT_TRUE(std::filesystem::exists(filepath));

        // A second object, built without vectors, reads them back from the file.
        geobox::DirectionalObject reloaded(
            "field", filepath, 3, std::vector<double>());
        // Built without vectors: nothing to draw yet.
        EXPECT_EQ(reloaded.mesh().edges.nb(), 0);
        reloaded.reload();
        ASSERT_TRUE(reloaded.attach_to(scene.mesh_object, GEO::MESH_FACETS));
        EXPECT_EQ(reloaded.vectors(), vectors);
        EXPECT_EQ(reloaded.mesh().edges.nb(), 6);

        std::filesystem::remove(filepath);
    }

    /**
     * @brief The properties panel draws one block of controls per axis.
     * @details Drawn headlessly (no renderer): what matters is that the widgets
     *          of every axis are laid out, and that what they change is what the
     *          object draws with.
     */
    TEST(DirectionalObjectTest, DrawsThePropertiesOfEachAxis) {
        ImGui::CreateContext();
        ImGuiIO& io = ImGui::GetIO();
        io.DisplaySize = ImVec2(1280.0f, 720.0f);
        io.DeltaTime = 1.0f / 60.0f;
        io.IniFilename = nullptr;
        io.LogFilename = nullptr;
        // No backend: let ImGui request the font textures lazily.
        io.BackendFlags |= ImGuiBackendFlags_RendererHasTextures;

        GEO::Mesh source_mesh;
        make_square_mesh(source_mesh);
        constexpr GEO::index_t NB_VECTORS_PER_ELEMENT = 3;
        std::vector<double> vectors;
        for (GEO::index_t f = 0; f < source_mesh.facets.nb(); ++f)
            append_frame(vectors);

        TestScene scene(source_mesh);
        geobox::DirectionalObject object(
            "field", "", NB_VECTORS_PER_ELEMENT, vectors);

        ImGui::NewFrame();
        ImGui::Begin("properties");
        // The single mesh object of the scene has as many facets as the field has
        // frames: the pickers attach the field to it on their own.
        object.draw_object_properties(scene.objects);
        ImGui::End();
        ImGui::EndFrame();

        EXPECT_EQ(object.mesh_object().lock(), scene.mesh_object);
        EXPECT_EQ(object.subelements(), GEO::MESH_FACETS);

        // Drawn with a length of 1, the vectors are as long as the diagonal of
        // the mesh; the bounding box grows accordingly (only one direction is
        // drawn here, see set_draw_opposite() below).
        double xyzmin[3];
        double xyzmax[3];
        object.set_draw_opposite(false);
        object.set_length(0.03f);
        object.get_bbox(xyzmin, xyzmax);
        const double short_extent = xyzmax[0] - xyzmin[0];
        object.set_length(1.0f);
        object.get_bbox(xyzmin, xyzmax);
        EXPECT_GT(xyzmax[0] - xyzmin[0], short_extent);
        EXPECT_LE(xyzmax[0] - xyzmin[0], short_extent + object.bbox_diagonal());

        // The length and the width are shared by all the axes, and the vectors
        // can be drawn in both directions; the controls write what is drawn.
        object.set_length(0.25f);
        object.set_width(1.0f);
        object.set_draw_opposite(true);
        EXPECT_FLOAT_EQ(object.length(), 0.25f);
        EXPECT_FLOAT_EQ(object.width(), 1.0f);
        EXPECT_TRUE(object.draw_opposite());

        ImGui::DestroyContext();
    }

    /**
     * @brief A field defined on the cells of a mesh is drawn from the center of
     *        each cell.
     * @details That is the case of a volumetric frame field, which the frame
     *          field application computes with volumetric_ set (see
     *          FrameFieldApplication::perform()).
     */
    TEST(DirectionalObjectTest, VisualizesTheFieldOfCells) {
        // Two tetrahedra sharing the (1, 2, 3) face.
        GEO::Mesh source_mesh;
        source_mesh.vertices.set_dimension(3);
        source_mesh.vertices.create_vertices(5);
        source_mesh.vertices.point(0) = GEO::vec3(0.0, 0.0, 0.0);
        source_mesh.vertices.point(1) = GEO::vec3(1.0, 0.0, 0.0);
        source_mesh.vertices.point(2) = GEO::vec3(0.0, 1.0, 0.0);
        source_mesh.vertices.point(3) = GEO::vec3(0.0, 0.0, 1.0);
        source_mesh.vertices.point(4) = GEO::vec3(1.0, 1.0, 1.0);
        source_mesh.cells.create_tet(0, 1, 2, 3);
        source_mesh.cells.create_tet(1, 2, 3, 4);
        ASSERT_EQ(source_mesh.cells.nb(), 2);

        // One vector per cell, along +x.
        std::vector<double> vectors = {1.0, 0.0, 0.0, 1.0, 0.0, 0.0};
        TestScene scene(source_mesh);
        geobox::DirectionalObject object("field", "", 1, vectors);
        ASSERT_TRUE(object.attach_to(scene.mesh_object, GEO::MESH_CELLS));

        EXPECT_EQ(object.nb_frames(), 2);
        EXPECT_EQ(object.nb_elements(), 2);
        EXPECT_EQ(object.mesh().edges.nb(), 2);

        // The first edge starts at the center of the first tetrahedron.
        const double* start = object.mesh().vertices.point_ptr(0);
        const double* tip = object.mesh().vertices.point_ptr(1);
        EXPECT_DOUBLE_EQ(start[0], 0.25);
        EXPECT_DOUBLE_EQ(start[1], 0.25);
        EXPECT_DOUBLE_EQ(start[2], 0.25);
        EXPECT_NEAR(tip[0] - start[0], 1.0, 1e-12);
    }

    /**
     * @brief A field whose mesh object left the scene is not drawn any more.
     * @details The scene owns the objects, so the attachment never keeps a mesh
     *          alive: once the mesh object is gone, the field has nowhere to be
     *          drawn, and is out of the bounding box of the scene until another
     *          mesh is picked.
     */
    TEST(DirectionalObjectTest, StopsDrawingWhenTheMeshObjectIsGone) {
        GEO::Mesh source_mesh;
        make_square_mesh(source_mesh);

        std::vector<double> vectors;
        append_frame(vectors);
        append_frame(vectors);

        TestScene scene(source_mesh);
        geobox::DirectionalObject object("field", "", 3, vectors);
        ASSERT_TRUE(object.attach_to(scene.mesh_object, GEO::MESH_FACETS));
        EXPECT_TRUE(object.attached());
        EXPECT_EQ(object.mesh().edges.nb(), 6);

        // The mesh object is dropped from the scene and destroyed.
        scene.objects.clear();
        scene.mesh_object.reset();

        EXPECT_FALSE(object.attached());

        double xyzmin[3];
        double xyzmax[3];
        object.get_bbox(xyzmin, xyzmax);
        for (GEO::coord_index_t c = 0; c < 3; ++c) {
            EXPECT_DOUBLE_EQ(xyzmin[c], 0.0);
            EXPECT_DOUBLE_EQ(xyzmax[c], 0.0);
        }

        // A new mesh object with the right number of facets takes over.
        const auto other = std::make_shared<geobox::MeshObject>(
            "other", "", source_mesh);
        scene.objects.push_back(other);
        ASSERT_TRUE(object.attach_to(other, GEO::MESH_FACETS));
        EXPECT_TRUE(object.attached());
    }

    /**
     * @brief When several kinds of element have as many elements as the field has
     *        frames, the field can be attached to any of them.
     * @details A tetrahedron has four vertices and four facets: a field of four
     *          frames can sit on either, which is the case the properties offer
     *          the "attach to" picker for.
     */
    TEST(DirectionalObjectTest, AttachesToAnyKindThatHasEnoughElements) {
        GEO::Mesh source_mesh;
        source_mesh.vertices.set_dimension(3);
        source_mesh.vertices.create_vertices(4);
        source_mesh.vertices.point(0) = GEO::vec3(0.0, 0.0, 0.0);
        source_mesh.vertices.point(1) = GEO::vec3(1.0, 0.0, 0.0);
        source_mesh.vertices.point(2) = GEO::vec3(0.0, 1.0, 0.0);
        source_mesh.vertices.point(3) = GEO::vec3(0.0, 0.0, 1.0);
        source_mesh.facets.create_triangle(0, 1, 2);
        source_mesh.facets.create_triangle(0, 1, 3);
        source_mesh.facets.create_triangle(0, 2, 3);
        source_mesh.facets.create_triangle(1, 2, 3);

        // One vector per element, along +x.
        std::vector<double> vectors;
        for (GEO::index_t i = 0; i < 4; ++i)
            vectors.insert(vectors.end(), {1.0, 0.0, 0.0});

        TestScene scene(source_mesh);
        geobox::DirectionalObject object("field", "", 1, vectors);

        const auto kinds = object.attachable_subelements(*scene.mesh_object);
        ASSERT_EQ(kinds.size(), 2u);
        EXPECT_EQ(kinds[0], GEO::MESH_VERTICES);
        EXPECT_EQ(kinds[1], GEO::MESH_FACETS);

        // On the vertices, the frames start at the vertices themselves.
        ASSERT_TRUE(object.attach_to(scene.mesh_object, GEO::MESH_VERTICES));
        EXPECT_EQ(object.subelements(), GEO::MESH_VERTICES);
        EXPECT_DOUBLE_EQ(object.mesh().vertices.point_ptr(0)[0], 0.0);

        // On the facets, they start at the center of the facets.
        ASSERT_TRUE(object.attach_to(scene.mesh_object, GEO::MESH_FACETS));
        EXPECT_EQ(object.subelements(), GEO::MESH_FACETS);
        EXPECT_DOUBLE_EQ(object.mesh().vertices.point_ptr(0)[0], 1.0 / 3.0);
    }

    /**
     * @brief A field that does not describe the elements of a mesh is not
     *        attached to it.
     * @details Building the visualization from it would read past the end of the
     *          vector array; the field stays unattached and draws nothing.
     */
    TEST(DirectionalObjectTest, RefusesAMeshThatDoesNotMatchTheField) {
        GEO::Mesh source_mesh;
        make_square_mesh(source_mesh);

        // A single frame: the two triangles of the square cannot hold it, and the
        // four vertices cannot either.
        std::vector<double> vectors;
        append_frame(vectors);

        TestScene scene(source_mesh);
        geobox::DirectionalObject object("field", "", 3, vectors);

        EXPECT_EQ(object.nb_frames(), 1);
        EXPECT_TRUE(object.attachable_subelements(*scene.mesh_object).empty());
        EXPECT_FALSE(object.attach_to(scene.mesh_object, GEO::MESH_FACETS));
        EXPECT_EQ(object.subelements(), GEO::MESH_NONE);
        EXPECT_EQ(object.mesh().edges.nb(), 0);
        EXPECT_EQ(object.mesh().vertices.nb(), 0);
        EXPECT_EQ(object.vectors().size(), 9);
    }

    /**
     * @brief The field can be attached to the vertices of a mesh.
     * @details A field whose frames are as many as the vertices of a mesh (e.g.
     *          one vector per point) is drawn from the vertices themselves.
     */
    TEST(DirectionalObjectTest, AttachesToTheVerticesOfAMesh) {
        GEO::Mesh source_mesh;
        make_square_mesh(source_mesh);

        // One vector per vertex, along +x.
        const GEO::index_t nb_vertices = source_mesh.vertices.nb();
        std::vector<double> vectors;
        for (GEO::index_t v = 0; v < nb_vertices; ++v)
            vectors.insert(vectors.end(), {1.0, 0.0, 0.0});

        TestScene scene(source_mesh);
        geobox::DirectionalObject object("field", "", 1, vectors);

        // Both the vertices and the facets could hold the field only if the
        // counts matched; here only the vertices do.
        EXPECT_EQ(object.attachable_subelements(*scene.mesh_object).size(), 1u);
        ASSERT_TRUE(object.attach_to(scene.mesh_object, GEO::MESH_VERTICES));

        EXPECT_EQ(object.nb_elements(), nb_vertices);
        EXPECT_EQ(object.mesh().edges.nb(), nb_vertices);

        // The first edge starts at the first vertex of the mesh.
        const double* start = object.mesh().vertices.point_ptr(0);
        EXPECT_DOUBLE_EQ(start[0], 0.0);
        EXPECT_DOUBLE_EQ(start[1], 0.0);
    }

    /**
     * @brief The vectors are drawn with the lengths they carry, not one common
     *        length.
     * @details A curvature-weighted frame field stores the curvature magnitudes in
     *          the vectors -- axis 0 along the maximum curvature, axis 1 along the
     *          minimum one -- so drawing them all at the same length would hide
     *          the information that tells a ridge from a flat direction. The
     *          longest vector of the field is length() * bbox_diagonal() long and
     *          every other one keeps its ratio to it.
     */
    TEST(DirectionalObjectTest, DrawsTheVectorsWithTheLengthsTheyCarry) {
        GEO::Mesh source_mesh;
        make_square_mesh(source_mesh);

        // Three axes with magnitudes 2, 1 and 0.5, i.e. ratios 4:2:1.
        std::vector<double> vectors;
        for (GEO::index_t f = 0; f < source_mesh.facets.nb(); ++f)
            vectors.insert(
                vectors.end(),
                {2.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.5});

        TestScene scene(source_mesh);
        ScaledDirectionalObject object("field", "", 3, vectors);
        ASSERT_TRUE(object.attach_to(scene.mesh_object, GEO::MESH_FACETS));

        // The visualization mesh holds the vectors themselves: their lengths are
        // the ones of the field, in model units.
        constexpr GEO::index_t NB_VECTORS_PER_ELEMENT = 3;
        for (GEO::index_t f = 0; f < source_mesh.facets.nb(); ++f) {
            for (GEO::index_t axis = 0; axis < NB_VECTORS_PER_ELEMENT; ++axis) {
                const GEO::index_t edge = axis * object.nb_elements() + f;
                const double* start = object.mesh().vertices.point_ptr(
                    object.mesh().edges.vertex(edge, 0));
                const double* end = object.mesh().vertices.point_ptr(
                    object.mesh().edges.vertex(edge, 1));
                const double drawn = std::sqrt(
                    (end[0] - start[0]) * (end[0] - start[0]) +
                    (end[1] - start[1]) * (end[1] - start[1]) +
                    (end[2] - start[2]) * (end[2] - start[2]));
                const double expected = axis == 0 ? 2.0 : (axis == 1 ? 1.0 : 0.5);
                EXPECT_NEAR(drawn, expected, 1e-12) << "axis " << axis;
            }
        }

        // What the drawing applies: the longest vector, of magnitude 2, comes out
        // length() * bbox_diagonal() long, and the others keep the ratio.
        object.set_length(0.5f);
        const double longest = object.longest();
        EXPECT_NEAR(longest, 0.5 * std::sqrt(2.0), 1e-6);
        EXPECT_NEAR(object.scale() * 2.0, longest, 1e-12);
        EXPECT_NEAR(object.scale() * 1.0, longest / 2.0, 1e-12);
        EXPECT_NEAR(object.scale() * 0.5, longest / 4.0, 1e-12);
    }

    /**
     * @brief The frames are exposed as the flat array a frame field algorithm
     *        reads.
     * @details The same storage as vectors(), in the layout the frame field
     *          algorithms expect: 3 coordinates per vector, three vectors per
     *          frame, in element order, so that the frame of element e starts at
     *          9*e. A caller that takes a raw pointer - the anisotropic metric of
     *          the LpCVT, for instance - then reads the field without a copy. A
     *          field holding no vector has nothing to point at, hence nullptr.
     */
    TEST(DirectionalObjectTest, ExposesItsFramesAsAFlatArray) {
        GEO::Mesh source_mesh;
        make_square_mesh(source_mesh);

        constexpr GEO::index_t NB_VECTORS_PER_ELEMENT = 3;
        std::vector<double> vectors;
        for (GEO::index_t f = 0; f < source_mesh.facets.nb(); ++f)
            append_frame(vectors);
        // Give the second frame its own coefficients, so that reading it proves
        // the 9 per element offset rather than repeating the same frame.
        if (source_mesh.facets.nb() > 1)
            for (GEO::index_t c = 0; c < 9; ++c)
                vectors[9 + c] = -vectors[9 + c];

        TestScene scene(source_mesh);
        geobox::DirectionalObject object(
            "field", "", NB_VECTORS_PER_ELEMENT, vectors);

        // The frames are the storage of the object itself, not a copy.
        ASSERT_NE(object.frames(), nullptr);
        EXPECT_EQ(object.frames(), object.vectors().data());
        EXPECT_EQ(object.nb_frames(), source_mesh.facets.nb());

        // The frame of element f is the one that was read for it, in element
        // order.
        for (GEO::index_t f = 0; f < object.nb_frames(); ++f) {
            for (GEO::index_t c = 0; c < 9; ++c)
                EXPECT_DOUBLE_EQ(object.frames()[9 * f + c], vectors[9 * f + c]);
        }
        ASSERT_EQ(object.nb_frames(), 2);
        EXPECT_DOUBLE_EQ(object.frames()[0], 1.0);
        EXPECT_DOUBLE_EQ(object.frames()[9], -1.0);
        EXPECT_DOUBLE_EQ(object.frames()[10], 0.0);
        EXPECT_DOUBLE_EQ(object.frames()[13], -1.0);

        // What it is exposed for: the frames are the ones the frame field
        // algorithms read, one matrix per element.
        const FrameField no_certainty;
        std::vector<double> matrices;
        no_certainty.compute_anisotropy_matrices(
            source_mesh, false, object.frames(), 2.0, matrices);
        ASSERT_EQ(matrices.size(), 9 * source_mesh.facets.nb());
        for (const double coefficient : matrices)
            EXPECT_TRUE(std::isfinite(coefficient));

        // An empty field has no frame to expose.
        const std::vector<double> nothing;
        geobox::DirectionalObject empty(
            "empty", "", NB_VECTORS_PER_ELEMENT, nothing);
        EXPECT_EQ(empty.nb_frames(), 0);
        EXPECT_EQ(empty.frames(), nullptr);
    }
}

