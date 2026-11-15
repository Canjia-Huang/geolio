//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/10/17.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <gtest/gtest.h>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_io.h>
#include <geolio/geobox/object/mesh_object.h>

namespace geolio::test
{
    /**
     * @brief MeshObject with its reload path and its cell-facet drawing
     *        decision made observable.
     * @details The reported scenario: a mesh file carries an attribute on the
     *          cell facets, the user displays it, then the file is edited so
     *          that the attribute is not in it any more and the Reload button
     *          is clicked. reload() replaces the mesh, which therefore loses
     *          the attribute the drawing passes sample.
     */
    class TestReloadMeshObject : public geobox::MeshObject {
    public:
        /**
         * @brief Builds the object under test from the file it was loaded from,
         *        as GeoBoxApplication::load() does.
         * @param[in] filepath File the mesh was read from.
         * @param[in] mesh Mesh read from that file.
         */
        TestReloadMeshObject(const std::string& filepath, const GEO::Mesh& mesh)
            : MeshObject(get_filename(filepath), filepath, mesh) {
        }

        /// Selects \p attribute for display, as the attribute popup does.
        void visualize(const std::string& attribute) {
            painting_mode_ = geobox::PaintingMode::ATTRIBUTE;
            set_attribute(attribute);
        }

        void set_field(const float lo, const float hi) {
            attribute_min_ = lo;
            attribute_max_ = hi;
        }

        /// Name of the attribute the colormap is sampled with.
        std::string sampled_attribute() { return colormap_attribute_name(); }

        /// Value of \p element in the attribute the colormap is sampled with.
        double sampled_value(const GEO::index_t element) {
            const GEO::MeshSubElementsStore& subelements =
                mesh().get_subelements_by_type(attribute_subelements_);
            GEO::ReadOnlyScalarAttributeAdapter attribute(
                subelements.attributes(), colormap_attribute_name());
            return attribute[element];
        }

        /// Whether the volume is drawn through the cell-facet pass.
        bool cell_facets_active() { return cell_facets_attribute_active(); }

        /// Whether the surface has border edges to draw.
        bool has_surface_borders() const { return surface_has_borders_; }

        /// Number of border edges the surface pass draws (see SurfaceBorders).
        /// @details Enumerates them if the object was told the mesh changed,
        ///          which is what the pass itself does before drawing them.
        GEO::index_t border_edge_count() {
            surface_borders_.update(mesh());
            return surface_borders_.edge_count();
        }

        /// Recomputes what the object caches about the mesh, as the drawing
        /// passes and the properties panel do on the next frame (see
        /// update_mesh_caches()).
        void refresh() { update_mesh_caches(); }

        /**
         * @brief Feeds the colormap with the attribute of \p element, as the
         *        cell-facet pass does for every vertex of every cell facet.
         * @details The scalar attribute is unbound outside of a drawing pass
         *          (it is bound by begin_attributes() and unbound by
         *          end_attributes()), so this is what the pass does when the
         *          mesh does not have the attribute: it must be a no-op, not the
         *          dereference of an unbound attribute (which aborts).
         */
        void feed_colormap(const GEO::index_t element) {
            draw_attribute_as_tex_coord(element);
        }

        /// Name of the display-only attribute the colormap samples.
        static std::string display_attribute() {
            return DISPLAY_ATTRIBUTE_NAME;
        }
    };

    namespace
    {
        /**
         * @brief Colormaps the tests select from.
         * @details Only the names matter here: the textures are bound while
         *          drawing only. The fixture below installs this table into the
         *          process-wide colormap library, which is where the objects
         *          read the colormaps from.
         */
        const std::vector<geobox::ColormapInfo>& reload_test_colormaps() {
            static const std::vector<geobox::ColormapInfo> colormaps = [] {
                std::vector<geobox::ColormapInfo> result(1);
                result[0].name = "viridis";
                return result;
            }();
            return colormaps;
        }

        /**
         * @brief Path of the file the tests reload, unique per test name.
         * @param[in] test_name Name of the test that uses the file.
         */
        std::string reload_test_file(const std::string& test_name) {
            return (
                std::filesystem::temp_directory_path() /
                ("geolio_" + test_name + ".geogram")
            ).string();
        }

        /**
         * @brief Writes the scenario of the tests to \p filepath.
         * @details A single tetrahedron, with (or without) the index attribute
         *          "v_cell" on the cell facets, holding the cell facet indices.
         *          The attribute is written to, and read back from, the geogram
         *          file format, so that removing it from the file is what
         *          reload() observes.
         * @param[in] filepath File to write.
         * @param[in] with_cell_facet_attribute Whether to write "v_cell".
         * @return true if the file was written.
         */
        bool save_tet_mesh(
            const std::string& filepath,
            const bool with_cell_facet_attribute
            ) {
            GEO::Mesh mesh;
            mesh.vertices.set_dimension(3);
            mesh.vertices.create_vertices(4);
            mesh.vertices.point(0) = GEO::vec3(0.0, 0.0, 0.0);
            mesh.vertices.point(1) = GEO::vec3(1.0, 0.0, 0.0);
            mesh.vertices.point(2) = GEO::vec3(0.0, 1.0, 0.0);
            mesh.vertices.point(3) = GEO::vec3(0.0, 0.0, 1.0);
            mesh.cells.create_tet(0, 1, 2, 3);

            if (with_cell_facet_attribute) {
                GEO::Attribute<GEO::index_t> v_cell(
                    mesh.cell_facets.attributes(), "v_cell"
                );
                for (GEO::index_t cell_facet = 0;
                     cell_facet < mesh.cell_facets.nb();
                     ++cell_facet)
                    v_cell[cell_facet] = cell_facet;
            }

            return GEO::mesh_save(mesh, filepath);
        }

        /**
         * @brief Writes a closed surface (the boundary of a tetrahedron) to
         *        \p filepath.
         * @details Every edge is shared by two facets, so the mesh has no
         *          border edge and the border pass has nothing to draw.
         * @param[in] filepath File to write.
         * @return true if the file was written.
         */
        bool save_closed_surface_mesh(const std::string& filepath) {
            GEO::Mesh mesh;
            mesh.vertices.set_dimension(3);
            mesh.vertices.create_vertices(4);
            mesh.vertices.point(0) = GEO::vec3(0.0, 0.0, 0.0);
            mesh.vertices.point(1) = GEO::vec3(1.0, 0.0, 0.0);
            mesh.vertices.point(2) = GEO::vec3(0.0, 1.0, 0.0);
            mesh.vertices.point(3) = GEO::vec3(0.0, 0.0, 1.0);

            // Outward-oriented triangles, so that connect() pairs them.
            mesh.facets.create_triangle(0, 2, 1);
            mesh.facets.create_triangle(0, 1, 3);
            mesh.facets.create_triangle(0, 3, 2);
            mesh.facets.create_triangle(1, 2, 3);
            // The geogram file format stores the corner adjacency, so it has
            // to be computed before saving for the loaded mesh to have it.
            mesh.facets.connect();

            return GEO::mesh_save(mesh, filepath);
        }

        /**
         * @brief Writes an open surface (a single triangle) to \p filepath.
         * @details All three edges are on the border, so the border pass has
         *          them to draw.
         * @param[in] filepath File to write.
         * @return true if the file was written.
         */
        bool save_open_surface_mesh(const std::string& filepath) {
            GEO::Mesh mesh;
            mesh.vertices.set_dimension(3);
            mesh.vertices.create_vertices(3);
            mesh.vertices.point(0) = GEO::vec3(0.0, 0.0, 0.0);
            mesh.vertices.point(1) = GEO::vec3(1.0, 0.0, 0.0);
            mesh.vertices.point(2) = GEO::vec3(0.0, 1.0, 0.0);
            mesh.facets.create_triangle(0, 1, 2);
            mesh.facets.connect();
            return GEO::mesh_save(mesh, filepath);
        }

        /**
         * @brief Loads \p filepath and builds the object under test from it, as
         *        GeoBoxApplication::load() does.
         */
        std::unique_ptr<TestReloadMeshObject> open(const std::string& filepath) {
            GEO::Mesh mesh;
            if (!mesh.load(filepath))
                return nullptr;
            return std::make_unique<TestReloadMeshObject>(filepath, mesh);
        }
    }

    /**
     * @brief Installs the colormap table the tests select from.
     * @details A MeshObject reads the process-wide colormap library instead of
     *          being handed a table (see ColormapLibrary). The tests run without
     *          a GL context, so the fixture installs a table of names directly:
     *          nothing is uploaded, and the entries only have to tell the
     *          colormaps apart. It is cleared afterwards so that no test depends
     *          on the table of the one before it.
     */
    class MeshObjectReloadTest : public ::testing::Test {
    protected:
        void SetUp() override {
            geobox::ColormapLibrary::instance().set_colormaps(
                reload_test_colormaps());
        }

        void TearDown() override {
            geobox::ColormapLibrary::instance().set_colormaps(
                std::vector<geobox::ColormapInfo>());
        }
    };

    /**
     * @brief Reloading a mesh whose file lost the displayed cell facet attribute
     *        displays the volume without it, instead of sampling an attribute
     *        the mesh does not have any more.
     * @details The reported crash: reload() copies the new mesh, which drops the
     *          attribute stores of the old one, and the drawing passes then look
     *          the attribute up by the name the user selected. The cell-facet
     *          pass reads it with MeshObject's own attribute adapter (see
     *          draw_attribute_as_tex_coord()), which is unbound when the
     *          attribute is missing: dereferencing it aborts on the ET_NONE
     *          element type of an unbound attribute.
     */
    TEST_F(MeshObjectReloadTest, RemovedCellFacetAttributeIsNotSampled) {
        const std::string filepath =
            reload_test_file("RemovedCellFacetAttributeIsNotSampled");
        ASSERT_TRUE(save_tet_mesh(filepath, true));

        const std::unique_ptr<TestReloadMeshObject> object = open(filepath);
        ASSERT_NE(object, nullptr);
        object->visualize("cell_facets.v_cell");
        object->set_field(0.0f, 3.0f);

        // The attribute is displayed through the display copy, and the volume is
        // drawn facet by facet to color each cell facet with it.
        EXPECT_EQ(object->sampled_attribute(),
                  TestReloadMeshObject::display_attribute());
        EXPECT_TRUE(object->cell_facets_active());

        // The file is edited: the attribute is not in it any more.
        ASSERT_TRUE(save_tet_mesh(filepath, false));
        object->reload();

        // The mesh no longer has the attribute, so nothing can be sampled...
        EXPECT_EQ(object->sampled_attribute(), "v_cell");
        // ... and the cell-facet pass is not taken: it would read the unbound
        // attribute, and the volume is drawn through MeshGfx, uncolored, as
        // GeoGram draws it when the attribute is missing.
        EXPECT_FALSE(object->cell_facets_active());
        // Feeding the colormap from the missing attribute is a no-op, even if a
        // facet pass were to run.
        object->feed_colormap(0);

        std::filesystem::remove(filepath);
    }

    /**
     * @brief The attribute the file still has keeps being displayed after a
     *        reload.
     * @details The counterpart of the test above: the availability test must not
     *          disable the cell-facet pass of a mesh that does have the
     *          attribute, and the display copy must be rebuilt on the reloaded
     *          attribute (the old one was destroyed with the old mesh).
     */
    TEST_F(MeshObjectReloadTest, KeptCellFacetAttributeIsStillSampled) {
        const std::string filepath =
            reload_test_file("KeptCellFacetAttributeIsStillSampled");
        ASSERT_TRUE(save_tet_mesh(filepath, true));

        const std::unique_ptr<TestReloadMeshObject> object = open(filepath);
        ASSERT_NE(object, nullptr);
        object->visualize("cell_facets.v_cell");
        object->set_field(0.0f, 3.0f);
        ASSERT_TRUE(object->cell_facets_active());

        // The file is rewritten with the attribute, as a file the user only
        // edited the geometry of would be.
        ASSERT_TRUE(save_tet_mesh(filepath, true));
        object->reload();

        EXPECT_EQ(object->sampled_attribute(),
                  TestReloadMeshObject::display_attribute());
        EXPECT_TRUE(object->cell_facets_active());
        // The values are the ones of the reloaded attribute: cell facet 1 holds
        // the value 1, clamped to the range the fields read.
        EXPECT_DOUBLE_EQ(object->sampled_value(1), 1.0);

        std::filesystem::remove(filepath);
    }

    /**
     * @brief The border controls are only offered when the surface has border
     *        edges to draw.
     * @details A closed surface has no edge without an adjacent facet, so
     *          MeshGfx::draw_surface_borders() would draw nothing and the GUI
     *          hides the border controls. The flag is refreshed by reload(), so
     *          replacing the file by a surface that has borders brings them
     *          back.
     */
    TEST_F(MeshObjectReloadTest, BorderControlsFollowTheSurface) {
        const std::string filepath =
            reload_test_file("BorderControlsFollowTheSurface");

        ASSERT_TRUE(save_closed_surface_mesh(filepath));
        const std::unique_ptr<TestReloadMeshObject> object = open(filepath);
        ASSERT_NE(object, nullptr);
        // The tetrahedron surface is closed: no border edge to draw.
        EXPECT_FALSE(object->has_surface_borders());

        // The file is replaced by an open surface: the borders are back.
        ASSERT_TRUE(save_open_surface_mesh(filepath));
        object->reload();
        EXPECT_TRUE(object->has_surface_borders());

        // Closing it again hides the controls again.
        ASSERT_TRUE(save_closed_surface_mesh(filepath));
        object->reload();
        EXPECT_FALSE(object->has_surface_borders());

        std::filesystem::remove(filepath);
    }

    /**
     * @brief Disconnecting the facets of a mesh in place makes the object show
     *        the border edges that this creates.
     * @details The reported bug: the "disconnect facets" operation of
     *          MeshConnectApplication sets GEO::NO_FACET as the adjacent facet
     *          of every facet corner, which turns every facet edge into a
     *          border edge. The object had computed "no border edge to draw"
     *          when the (closed) mesh was set, and kept that answer, so the
     *          border pass stayed off and the borders were never drawn.
     *          An operation that modifies the mesh must report it
     *          (MeshObject::mesh_changed()), which is what
     *          BaseApplication::perform_on_mesh() does.
     */
    TEST_F(MeshObjectReloadTest, DisconnectingFacetsShowsBorders) {
        const std::string filepath =
            reload_test_file("DisconnectingFacetsShowsBorders");

        ASSERT_TRUE(save_closed_surface_mesh(filepath));
        const std::unique_ptr<TestReloadMeshObject> object = open(filepath);
        ASSERT_NE(object, nullptr);
        ASSERT_FALSE(object->has_surface_borders());
        ASSERT_EQ(object->border_edge_count(), 0u);

        // Disconnect the facets, as the application does.
        for (const GEO::index_t f : object->mesh().facets) {
            for (const GEO::index_t c : object->mesh().facets.corners(f))
                object->mesh().facet_corners.set_adjacent_facet(
                    c, GEO::NO_FACET);
        }
        // Without telling the object, what it cached about the mesh is stale.
        EXPECT_FALSE(object->has_surface_borders());

        object->mesh_changed();
        object->refresh();
        // Every facet edge is a border edge now: the border pass has to run,
        // and to draw the edges of the mesh it was told about.
        EXPECT_TRUE(object->has_surface_borders());
        EXPECT_EQ(
            object->border_edge_count(),
            3 * object->mesh().facets.nb());

        // Connecting the facets back closes the surface again.
        object->mesh().facets.connect();
        object->mesh_changed();
        object->refresh();
        EXPECT_FALSE(object->has_surface_borders());
        EXPECT_EQ(object->border_edge_count(), 0u);

        std::filesystem::remove(filepath);
    }
}
