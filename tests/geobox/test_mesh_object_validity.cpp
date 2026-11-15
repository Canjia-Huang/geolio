//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <geogram/mesh/mesh.h>
#include <geogram_gfx/third_party/imgui/imgui.h>
#include <geolio/geobox/object/mesh_object.h>

namespace geolio::test
{
    namespace
    {
        /**
         * @brief MeshObject with the state of its validity check made
         *        observable.
         */
        class TestValidityMeshObject : public geobox::MeshObject {
        public:
            explicit TestValidityMeshObject(const GEO::Mesh& mesh)
                : MeshObject("mesh", "", mesh) {
            }

            /// Number of invalid elements of each kind.
            [[nodiscard]] const InvalidElementCounts& counts() const {
                return invalid_counts_;
            }

            /// Whether the warning dialog is still due.
            [[nodiscard]] bool dialog_pending() const {
                return validity_dialog_pending_;
            }

            void set_remove_invalid_vertices(const bool remove) {
                remove_invalid_vertices_ = remove;
            }

            void set_remove_invalid_elements(const bool remove) {
                remove_invalid_elements_ = remove;
            }

            /// Applies what the dialog's "repair" button applies.
            void repair() { repair_invalid_vertices_and_elements(); }
        };

        /**
         * @brief Fills \p mesh with a triangle whose data cannot be rendered.
         * @details Three vertices, one of which has NaN coordinates, plus a
         *          facet and an edge that reference vertices the mesh does not
         *          have -- i.e. what a corrupted file looks like, and what makes
         *          the drawing passes read outside the vertex array.
         */
        void make_invalid_mesh(GEO::Mesh& mesh) {
            mesh.vertices.set_dimension(3);
            mesh.vertices.create_vertices(3);
            mesh.vertices.point(0) = GEO::vec3(0.0, 0.0, 0.0);
            mesh.vertices.point(1) = GEO::vec3(1.0, 0.0, 0.0);
            const double nan = std::numeric_limits<double>::quiet_NaN();
            mesh.vertices.point(2) = GEO::vec3(nan, nan, nan);

            mesh.facets.create_triangle(0, 1, 2);
            // The facet references a vertex that does not exist.
            mesh.facets.set_vertex(0, 0, 99);

            // So does the edge.
            mesh.edges.create_edge(0, 42);
        }

        /**
         * @brief Fills \p mesh with a valid triangle.
         */
        void make_valid_mesh(GEO::Mesh& mesh) {
            mesh.vertices.set_dimension(3);
            mesh.vertices.create_vertices(3);
            mesh.vertices.point(0) = GEO::vec3(0.0, 0.0, 0.0);
            mesh.vertices.point(1) = GEO::vec3(1.0, 0.0, 0.0);
            mesh.vertices.point(2) = GEO::vec3(0.0, 1.0, 0.0);
            mesh.facets.create_triangle(0, 1, 2);
        }
    }

    /**
     * @brief An object created from valid data validates, and has nothing to
     *        warn about.
     */
    TEST(MeshObjectValidityTest, ValidMeshHasNothingToReport) {
        GEO::Mesh mesh;
        make_valid_mesh(mesh);

        TestValidityMeshObject object(mesh);
        EXPECT_TRUE(object.check_validity());
        EXPECT_FALSE(object.counts().any());
        EXPECT_FALSE(object.dialog_pending());
    }

    /**
     * @brief An object created from invalid data reports the invalid elements
     *        of each kind, as the warning dialog shows them.
     * @details The check runs when the object is created (the constructor calls
     *          it), because data that cannot be rendered has to be dealt with
     *          before the object is drawn, not when the next frame crashes.
     */
    TEST(MeshObjectValidityTest, ReportsInvalidElements) {
        GEO::Mesh mesh;
        make_invalid_mesh(mesh);

        TestValidityMeshObject object(mesh);

        // Counted when the object was created.
        EXPECT_TRUE(object.dialog_pending());
        EXPECT_EQ(object.counts().vertices, 1);
        EXPECT_EQ(object.counts().edges, 1);
        EXPECT_EQ(object.counts().facets, 1);
        EXPECT_EQ(object.counts().cells, 0);
        EXPECT_FALSE(object.check_validity());
        // Data that cannot be rendered keeps the object out of the scene: the
        // drawing passes would read outside of the mesh.
        EXPECT_FALSE(object.renderable());
    }

    /**
     * @brief Repairing removes what the dialog's checkboxes ask for.
     * @details "remove invalid elements" removes the edges, facets and cells
     *          that reference missing vertices, and "remove invalid vertices"
     *          removes the vertices whose coordinates are not finite.
     */
    TEST(MeshObjectValidityTest, RepairsInvalidElements) {
        GEO::Mesh mesh;
        make_invalid_mesh(mesh);

        TestValidityMeshObject object(mesh);
        ASSERT_TRUE(object.dialog_pending());

        object.set_remove_invalid_elements(true);
        object.set_remove_invalid_vertices(true);
        object.repair();

        // The invalid facet and edge are gone, and so are the vertices whose
        // coordinates were not finite.
        EXPECT_EQ(object.mesh().facets.nb(), 0);
        EXPECT_EQ(object.mesh().edges.nb(), 0);
        EXPECT_EQ(object.mesh().vertices.nb(), 2);
        // Nothing is left to warn about: repairing the mesh also re-checks it.
        EXPECT_FALSE(object.dialog_pending());
        EXPECT_TRUE(object.check_validity());
        // The repaired mesh is drawn again.
        EXPECT_TRUE(object.renderable());
    }

    /**
     * @brief Repairing with only "remove invalid vertices" keeps the elements,
     *        and reports them again, since removing a vertex leaves the
     *        elements that reference it with no vertex at all.
     */
    TEST(MeshObjectValidityTest, RemovingVerticesLeavesElementsToFix) {
        GEO::Mesh mesh;
        make_valid_mesh(mesh);
        const double nan = std::numeric_limits<double>::quiet_NaN();
        mesh.vertices.point(1) = GEO::vec3(nan, nan, nan);

        TestValidityMeshObject object(mesh);
        ASSERT_EQ(object.counts().vertices, 1);
        ASSERT_TRUE(object.dialog_pending());

        object.set_remove_invalid_vertices(true);
        object.set_remove_invalid_elements(false);
        object.repair();

        // The vertex is gone, but the facet that referenced it is now the
        // invalid one: the dialog comes back with the new counts.
        EXPECT_EQ(object.mesh().vertices.nb(), 2);
        EXPECT_EQ(object.counts().vertices, 0);
        EXPECT_EQ(object.counts().facets, 1);
        EXPECT_TRUE(object.dialog_pending());
        // Still invalid, so still left out of the scene.
        EXPECT_FALSE(object.renderable());
    }

    /**
     * @brief The warning dialog can be drawn, and does not dismiss itself.
     * @details Drawn headlessly (no renderer): what matters is that the widgets
     *          are laid out and that the dialog stays due until the user
     *          clicks its button, i.e. that it cannot be skipped.
     */
    TEST(MeshObjectValidityTest, DialogStaysUntilAnswered) {
        ImGui::CreateContext();
        ImGuiIO& io = ImGui::GetIO();
        io.DisplaySize = ImVec2(1280.0f, 720.0f);
        io.DeltaTime = 1.0f / 60.0f;
        io.IniFilename = nullptr;
        io.LogFilename = nullptr;
        // No backend: tell ImGui it may request the font textures lazily.
        io.BackendFlags |= ImGuiBackendFlags_RendererHasTextures;

        GEO::Mesh mesh;
        make_invalid_mesh(mesh);
        TestValidityMeshObject object(mesh);
        ASSERT_TRUE(object.dialog_pending());

        for (int frame = 0; frame < 3; ++frame) {
            ImGui::NewFrame();
            object.draw_validity_dialog();
            ImGui::EndFrame();
            // No click: the dialog is still due on the next frame.
            EXPECT_TRUE(object.dialog_pending());
        }

        ImGui::DestroyContext();
    }
}
