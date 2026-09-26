//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/14.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <gtest/gtest.h>
#include <geolio/common/octahedral_rotations.h>
#include <memory>
#include <random>
#include <utility>
#include "../utils.h"

namespace
{
    template <GEO::index_t DIM>
    bool equal(
        const GEO::Matrix<DIM, double>& A,
        const GEO::Matrix<DIM, double>& B
        ) {
        for (GEO::index_t i = 0; i < DIM; ++i) {
            for (GEO::index_t j = 0; j < DIM; ++j) {
                if (std::fabs(A(i,j)-B(i,j)) > 1e-10)
                    return false;
            }
        }
        return true;
    }

    /**
     * @brief Whether two parameter values agree component-wise.
     *
     * Transitions only involve rotations that are signed axis permutations and integer
     * translations, so a correct propagation reproduces the expected lattice point
     * exactly; the tolerance is here to keep the comparison meaningful if the
     * parameterization is ever moved off the integer lattice.
     */
    template <GEO::index_t DIM>
    bool nearly_equal(
        const GEO::vecng<DIM, double>& a,
        const GEO::vecng<DIM, double>& b,
        const double eps = 1e-10
        ) {
        for (GEO::index_t d = 0; d < DIM; ++d) {
            if (std::fabs(a[d]-b[d]) > eps)
                return false;
        }
        return true;
    }

    /**
     * @brief Marker stored in every vertex before a propagation, so that a vertex the
     * traversal was not supposed to reach is easy to detect: it can only still hold
     * this value.
     */
    constexpr double UNREACHED_UV = -12345.0;
}

namespace geolio::test
{
    template <GEO::index_t DIM>
    class OctahedralRotationsTest : public ::testing::Test {
        static_assert(DIM == 2 || DIM == 3);
    protected:
        /** @brief Parameter type of the dimension under test: `vec2` in 2D, `vec3` in 3D. */
        using vec = GEO::vecng<DIM, double>;

        void SetUp(
            ) override {
            if constexpr (DIM == 2) {
                for (const auto& R : OCTAHEDRAL_ROTATIONS_2D)
                    rotations_.push_back(R);
            }
            else {
                for (const auto& R : OCTAHEDRAL_ROTATIONS_3D)
                    rotations_.push_back(R);
            }
        }

        void test_determinant(
            ) const {
            for (const auto& R : rotations_)
                EXPECT_EQ(GEO::det(R), 1);
        }

        void test_orthogonal(
            ) const {
            for (const auto& R : rotations_) {
                const auto I = R * R.transpose();
                for (GEO::index_t i = 0; i < DIM; ++i) {
                    for (GEO::index_t j = 0; j < DIM; ++j) {
                        if (i == j)
                            EXPECT_EQ(I(i, j), 1);
                        else
                            EXPECT_EQ(I(i, j), 0);
                    }
                }
            }
        }

        void test_uniqueness(
            ) const {
            for (GEO::index_t i = 0, i_end = rotations_.size(); i < i_end; ++i) {
                const auto& Ri = rotations_[i];
                for (GEO::index_t j = i+1; j < i_end; ++j) {
                    const auto& Rj = rotations_[j];
                    EXPECT_FALSE(equal(Ri, Rj));
                }
            }
        }

        void test_closure(
            ) const {
            for (const auto& R0 : rotations_) {
                for (const auto& R1 : rotations_) {
                    const auto R = R0*R1;

                    bool found = false;
                    for (const auto& R2 : rotations_) {
                        if (equal(R, R2)) {
                            found = true;
                            break;
                        }
                    }
                    EXPECT_TRUE(found);
                }
            }
        }

        void test_contains_identity(
            ) const {
            GEO::Matrix<DIM, double> I;
            I.load_identity();
            for (const auto& R : rotations_) {
                if (equal(R, I))
                    return;
            }
            FAIL();
        }

        std::vector<GEO::Matrix<DIM, double>> rotations_;

        /* == helpers of the transition-propagation tests ============================================ */

        /**
         * @brief Number of elements of the mesh under test: facets in 2D, cells in 3D.
         * @return The number of triangles (2D) or tetrahedra (3D) of `mesh_`.
         */
        [[nodiscard]] GEO::index_t elements_nb(
            ) const {
            if constexpr (DIM == 2)
                return mesh_.facets.nb();
            else
                return mesh_.cells.nb();
        }

        /**
         * @brief Number of vertices of one element.
         * @return 3 for a triangle, 4 for a tetrahedron.
         */
        [[nodiscard]] static constexpr GEO::index_t element_vertices_nb(
            ) {
            return (DIM == 2) ? 3 : 4;
        }

        /**
         * @brief Global index of a vertex of an element.
         * @param[in] e Element index, i.e. a facet in 2D and a cell in 3D.
         * @param[in] lv Local vertex index, in `0..element_vertices_nb()-1`.
         * @return The global vertex index.
         */
        [[nodiscard]] GEO::index_t element_vertex(
            const GEO::index_t e,
            const GEO::index_t lv
            ) const {
            if constexpr (DIM == 2)
                return mesh_.facets.vertex(e, lv);
            else
                return mesh_.cells.vertex(e, lv);
        }

        /**
         * @brief Global index of the corner of a vertex of an element.
         * @param[in] e Element index, i.e. a facet in 2D and a cell in 3D.
         * @param[in] lv Local vertex index, in `0..element_vertices_nb()-1`.
         * @return The global corner index of that element.
         */
        [[nodiscard]] GEO::index_t element_corner(
            const GEO::index_t e,
            const GEO::index_t lv
            ) const {
            if constexpr (DIM == 2)
                return mesh_.facets.corner(e, lv);
            else
                return mesh_.cells.corner(e, lv);
        }

        /**
         * @brief Build the structured mesh used by the propagation tests, together with its
         * ground-truth lattice parameterization.
         *
         * The mesh is an `n` by `n` grid of quads split into triangles (2D) or an `n` by `n`
         * by `n` grid of cubes split into tetrahedra (3D). Both are connected, have a
         * boundary, and are consistently oriented, which is what `connect()` needs to pair
         * the shared edges and facets. The 3D grid uses Kuhn's subdivision, i.e. the six
         * tetrahedra obtained by walking the three axes in every possible order, a
         * subdivision whose restriction to a face only depends on the two axes that face
         * spans, hence conforming on the whole grid.
         *
         * `gt_uv_` is the ground-truth parameterization: the grid vertex `(i, j, k)` is
         * mapped to the lattice point `(3i, 5j)` in 2D and `(3i, 5j, 7k)` in 3D. The three
         * strides are distinct primes, so two vertices of one element differ by a vector
         * whose components are distinct multiples of distinct primes, and two such vectors
         * are parallel only if they are equal up to sign, which three distinct vertices of
         * an element never are. No element corner, shared edge or shared face is therefore
         * degenerate in the ground-truth parameterization.
         *
         * @param[in] n Number of cells of the grid along each axis.
         */
        void build_grid(
            const GEO::index_t n
            ) {
            mesh_.clear(false, false);

            if constexpr (DIM == 2) {
                const auto grid_vertex = [n](
                    const GEO::index_t i,
                    const GEO::index_t j
                    ) {
                    return i*(n+1) + j;
                };

                const auto vertices_nb = (n+1)*(n+1);
                mesh_.vertices.create_vertices(vertices_nb);
                mesh_.facets.create_triangles(2*n*n);
                gt_uv_.resize(vertices_nb);

                for (GEO::index_t i = 0; i <= n; ++i) {
                    for (GEO::index_t j = 0; j <= n; ++j) {
                        const auto v = grid_vertex(i, j);
                        mesh_.vertices.point(v) = GEO::vec3(double(i), double(j), 0);
                        gt_uv_[v] = vec(3*double(i), 5*double(j));
                    }
                }

                GEO::index_t f = 0;
                for (GEO::index_t i = 0; i < n; ++i) {
                    for (GEO::index_t j = 0; j < n; ++j) {
                        const auto v00 = grid_vertex(i, j);
                        const auto v10 = grid_vertex(i+1, j);
                        const auto v11 = grid_vertex(i+1, j+1);
                        const auto v01 = grid_vertex(i, j+1);

                        /* Counter-clockwise, i.e. the two triangles of the quad are oriented
                         * consistently with their neighbors. */
                        mesh_.facets.set_vertex(f, 0, v00);
                        mesh_.facets.set_vertex(f, 1, v10);
                        mesh_.facets.set_vertex(f, 2, v11);
                        ++f;
                        mesh_.facets.set_vertex(f, 0, v00);
                        mesh_.facets.set_vertex(f, 1, v11);
                        mesh_.facets.set_vertex(f, 2, v01);
                        ++f;
                    }
                }

                mesh_.facets.connect();
            }
            else {
                const auto grid_vertex = [n](
                    const GEO::index_t i,
                    const GEO::index_t j,
                    const GEO::index_t k
                    ) {
                    return (i*(n+1) + j)*(n+1) + k;
                };

                const auto vertices_nb = (n+1)*(n+1)*(n+1);
                mesh_.vertices.create_vertices(vertices_nb);
                gt_uv_.resize(vertices_nb);

                for (GEO::index_t i = 0; i <= n; ++i) {
                    for (GEO::index_t j = 0; j <= n; ++j) {
                        for (GEO::index_t k = 0; k <= n; ++k) {
                            const auto v = grid_vertex(i, j, k);
                            mesh_.vertices.point(v) = GEO::vec3(double(i), double(j), double(k));
                            gt_uv_[v] = vec(3*double(i), 5*double(j), 7*double(k));
                        }
                    }
                }

                static constexpr int AXES_ORDERS[6][3] = {
                    {0,1,2}, {0,2,1}, {1,0,2}, {1,2,0}, {2,0,1}, {2,1,0}
                };

                const auto cells_nb = 6*n*n*n;
                mesh_.cells.create_tets(cells_nb);
                std::vector<GEO::index_t> tet(4);
                GEO::index_t c = 0;
                for (GEO::index_t i = 0; i < n; ++i) {
                    for (GEO::index_t j = 0; j < n; ++j) {
                        for (GEO::index_t k = 0; k < n; ++k) {
                            for (const auto& order : AXES_ORDERS) {
                                /* The tetrahedron that walks the axes in this order, from one
                                 * corner of the cube to the opposite one. */
                                int offset[4][3] = {{0,0,0}, {0,0,0}, {0,0,0}, {1,1,1}};
                                offset[1][order[0]] = 1;
                                offset[2][order[0]] = 1;
                                offset[2][order[1]] = 1;

                                for (GEO::index_t lv = 0; lv < 4; ++lv)
                                    tet[lv] = grid_vertex(
                                        i+offset[lv][0], j+offset[lv][1], k+offset[lv][2]);

                                /* `cells.connect()` only pairs two facets that appear reversed
                                 * in their cells, so every tetrahedron is given the same
                                 * (positive) orientation. */
                                const auto& p0 = mesh_.vertices.point(tet[0]);
                                const auto& p1 = mesh_.vertices.point(tet[1]);
                                const auto& p2 = mesh_.vertices.point(tet[2]);
                                const auto& p3 = mesh_.vertices.point(tet[3]);
                                if (GEO::dot(GEO::cross(p1-p0, p2-p0), p3-p0) < 0)
                                    std::swap(tet[2], tet[3]);

                                for (GEO::index_t lv = 0; lv < 4; ++lv)
                                    mesh_.cells.set_vertex(c, lv, tet[lv]);
                                ++c;
                            }
                        }
                    }
                }

                mesh_.cells.connect();
            }
        }

        /**
         * @brief Append one vertex that no element is incident to.
         *
         * A traversal only reaches vertices through the elements it visits, so this vertex
         * must keep whatever the attribute holds when a propagation is run.
         */
        void append_isolated_vertex(
            ) {
            const auto v = mesh_.vertices.nb();
            mesh_.vertices.create_vertices(1);
            mesh_.vertices.point(v) = GEO::vec3(1000, 1000, 1000);
            gt_uv_.resize(mesh_.vertices.nb());
        }

        /**
         * @brief Append one element that shares no vertex with the current mesh, i.e. that
         * starts a second connected component.
         *
         * The new vertices are appended after the existing ones and extend `gt_uv_` with
         * further lattice points, so that `randomize_charts()` gives the new element a chart
         * like any other.
         *
         * @return Index of the new element.
         */
        [[nodiscard]] GEO::index_t append_isolated_element(
            ) {
            const auto v0 = mesh_.vertices.nb();
            const GEO::vec3 origin(1000, 1000, 1000);

            if constexpr (DIM == 2) {
                static constexpr int OFFSETS[3][2] = {{0,0}, {1,0}, {1,1}};

                mesh_.vertices.create_vertices(3);
                mesh_.facets.create_triangles(1);
                gt_uv_.resize(mesh_.vertices.nb());

                for (GEO::index_t lv = 0; lv < 3; ++lv) {
                    const auto i = OFFSETS[lv][0];
                    const auto j = OFFSETS[lv][1];
                    mesh_.vertices.point(v0+lv) = origin + GEO::vec3(double(i), double(j), 0);
                    gt_uv_[v0+lv] = vec(8900 + 3*double(i), 8900 + 5*double(j));
                }

                const auto f = mesh_.facets.nb()-1;
                for (GEO::index_t lv = 0; lv < 3; ++lv)
                    mesh_.facets.set_vertex(f, lv, v0+lv);

                /* Rebuild the adjacency of the whole surface, including that of the grid. */
                mesh_.facets.connect();
                return f;
            }
            else {
                static constexpr int OFFSETS[4][3] = {{0,0,0}, {1,0,0}, {0,1,0}, {0,0,1}};

                mesh_.vertices.create_vertices(4);
                mesh_.cells.create_tets(1);
                gt_uv_.resize(mesh_.vertices.nb());

                for (GEO::index_t lv = 0; lv < 4; ++lv) {
                    const auto i = OFFSETS[lv][0];
                    const auto j = OFFSETS[lv][1];
                    const auto k = OFFSETS[lv][2];
                    /* Positively oriented. */
                    mesh_.vertices.point(v0+lv) = origin + GEO::vec3(double(i), double(j), double(k));
                    gt_uv_[v0+lv] = vec(8900 + 3*double(i), 8900 + 5*double(j), 8900 + 7*double(k));
                }

                const auto c = mesh_.cells.nb()-1;
                for (GEO::index_t lv = 0; lv < 4; ++lv)
                    mesh_.cells.set_vertex(c, lv, v0+lv);

                /* Rebuild the adjacency of the whole tetrahedralization, including that of
                 * the grid. */
                mesh_.cells.connect();
                return c;
            }
        }

        /**
         * @brief Bind the two attributes of a parameterization on the current mesh.
         *
         * The element-corner attribute holds the input, i.e. the parameterization of every
         * facet (2D) or cell (3D) in its own chart, and the vertex attribute receives the
         * propagated parameterization.
         */
        void create_uv_attributes(
            ) {
            if constexpr (DIM == 2)
                element_uv_ = std::make_unique<GEO::Attribute<vec>>(mesh_.facet_corners.attributes(), "uv");
            else
                element_uv_ = std::make_unique<GEO::Attribute<vec>>(mesh_.cell_corners.attributes(), "uvw");

            vertex_uv_ = std::make_unique<GEO::Attribute<vec>>(mesh_.vertices.attributes(), "vertex_uv");
        }

        /**
         * @brief Give every element its own chart, drawn at random, and fill the per-corner
         * parameterization accordingly.
         *
         * Element `e` receives a rotation `R` of the group and an integer translation `t`,
         * and the corner of its local vertex `lv` is set to
         * `R*gt_uv_[element_vertex(e, lv)] + t`. The input is then transition-consistent by
         * construction: those charts are all rigid images of the same lattice
         * parameterization, which is precisely what a propagation has to recover.
         *
         * @param[in] seed Seed of the draw. It is local to this function, so that a test can
         *                 make several independent draws and still be reproducible.
         */
        void randomize_charts(
            const GEO::Numeric::uint64 seed
            ) {
            std::mt19937_64 rng(seed);

            chart_ri_.resize(elements_nb());
            chart_t_.resize(elements_nb());

            for (GEO::index_t e = 0, e_end = elements_nb(); e < e_end; ++e) {
                chart_ri_[e] = GEO::index_t(rng() % rotations_.size());
                for (GEO::index_t d = 0; d < DIM; ++d)
                    chart_t_[e][d] = double(rng() % 128) - 64.0;

                const auto& R = rotations_[chart_ri_[e]];
                const auto& t = chart_t_[e];
                for (GEO::index_t lv = 0; lv < element_vertices_nb(); ++lv)
                    element_uv()[element_corner(e, lv)] = R*gt_uv_[element_vertex(e, lv)] + t;
            }
        }

        /**
         * @brief Mark every vertex as not reached, so that a propagation that misses one of
         * them is detected.
         */
        void reset_vertex_uvs(
            ) {
            vertex_uv().fill(unreached_uv());
        }

        /**
         * @brief Run the propagation from one element.
         *
         * The two overloads of `propagate_consistent_transitions()` are told apart by the
         * dimension of the attributes, i.e. by the dimension under test.
         *
         * @param[in] start Element that seeds the traversal and defines the reference frame.
         */
        void propagate(
            const GEO::index_t start
            ) {
            propagate_consistent_transitions(mesh_, start, element_uv(), vertex_uv());
        }

        /**
         * @brief Parameterization a propagation started from `start` must produce on one
         * vertex.
         *
         * The chart of `start` becomes the reference frame of the traversal, so the expected
         * per-vertex parameterization is the ground truth expressed in that chart.
         *
         * @param[in] start Element the propagation was started from.
         * @param[in] v Vertex index.
         * @return The expected parameter of `v`.
         */
        [[nodiscard]] vec expected_uv(
            const GEO::index_t start,
            const GEO::index_t v
            ) const {
            return rotations_[chart_ri_[start]]*gt_uv_[v] + chart_t_[start];
        }

        /**
         * @brief Parameter stored in a vertex that a traversal must not reach.
         */
        [[nodiscard]] static vec unreached_uv(
            ) {
            vec uv;
            for (GEO::index_t d = 0; d < DIM; ++d)
                uv[d] = UNREACHED_UV;
            return uv;
        }

        /**
         * @brief Whether the propagated per-vertex parameterization reproduces the corner
         * parameterization of one element, up to a single rotation and translation.
         *
         * This is the contract of a propagation: the result must be one single chart, so the
         * chart of an element must be recovered from the propagated vertices by one rigid
         * motion of the group.
         *
         * @param[in] e Element index, i.e. a facet in 2D and a cell in 3D.
         * @return true if some rotation of the group maps the propagated vertices of that
         *         element onto its corners.
         */
        [[nodiscard]] bool element_is_consistent(
            const GEO::index_t e
            ) const {
            const auto p0 = vertex_uv()[element_vertex(e, 0)];
            const auto q0 = element_uv()[element_corner(e, 0)];

            for (const auto& R : rotations_) {
                const auto t = q0 - R*p0;

                bool consistent = true;
                for (GEO::index_t lv = 1; lv < element_vertices_nb(); ++lv) {
                    if (!nearly_equal(R*vertex_uv()[element_vertex(e, lv)] + t,
                                      element_uv()[element_corner(e, lv)])) {
                        consistent = false;
                        break;
                    }
                }
                if (consistent)
                    return true;
            }
            return false;
        }

        GEO::Attribute<vec>& element_uv(
            ) {
            return *element_uv_;
        }

        [[nodiscard]] const GEO::Attribute<vec>& element_uv(
            ) const {
            return *element_uv_;
        }

        GEO::Attribute<vec>& vertex_uv(
            ) {
            return *vertex_uv_;
        }

        [[nodiscard]] const GEO::Attribute<vec>& vertex_uv(
            ) const {
            return *vertex_uv_;
        }

        /* == data ==================================================================================== */

        GEO::Mesh mesh_;
        /** @brief Ground-truth parameterization, one lattice point per vertex. */
        std::vector<vec> gt_uv_;
        /** @brief Index, in `rotations_`, of the ground-truth chart rotation of each element. */
        std::vector<GEO::index_t> chart_ri_;
        /** @brief Ground-truth chart translation of each element. */
        std::vector<vec> chart_t_;
        std::unique_ptr<GEO::Attribute<vec>> element_uv_;
        std::unique_ptr<GEO::Attribute<vec>> vertex_uv_;
    };

    template <typename DimType>
    class OctahedralRotationsDimTest : public OctahedralRotationsTest<DimType::value> {};

    TYPED_TEST_SUITE(OctahedralRotationsDimTest, DimTypes);

    TYPED_TEST(OctahedralRotationsDimTest, validity) {
        this->test_determinant();
        this->test_orthogonal();
        this->test_uniqueness();
        this->test_closure();
        this->test_contains_identity();
    }

    TYPED_TEST(OctahedralRotationsDimTest, compute_transition_function) {
        constexpr GEO::index_t DIM = TypeParam::value;
        constexpr GEO::index_t N = 100;

        for (GEO::index_t i = 0; i < N; ++i) {
            const auto gt_ri = static_cast<GEO::index_t>(std::round((this->rotations_.size()-1) * GEO::Numeric::random_float32()));
            const auto& R = this->rotations_[gt_ri];
            GEO::vecng<DIM, double> gt_t;
            for (GEO::index_t d = 0; d < DIM; ++d)
                gt_t[d] = GEO::Numeric::random_int32();

            GEO::vecng<DIM, double> p0;
            GEO::vecng<DIM, double> p1;
            GEO::vecng<DIM, double> p2;
            for (GEO::index_t d = 0; d < DIM; ++d) {
                p0[d] = GEO::Numeric::random_float32();
                p1[d] = GEO::Numeric::random_float32();
                p2[d] = GEO::Numeric::random_float32();
            }
            const auto q0 = R*p0 + gt_t;
            const auto q1 = R*p1 + gt_t;
            const auto q2 = R*p2 + gt_t;

            GEO::index_t ri;
            GEO::vecng<DIM, double> t;
            if constexpr (DIM == 2)
                compute_transition_function(p0, p1, q0, q1, ri, t);
            else
                compute_transition_function(p0, p1, p2, q0, q1, q2, ri, t);
            EXPECT_EQ(ri, gt_ri);
            EXPECT_NEAR(t.x, gt_t.x, 1e-10);
            EXPECT_NEAR(t.y, gt_t.y, 1e-10);
            if constexpr (DIM == 3)
                EXPECT_NEAR(t.z, gt_t.z, 1e-10);
        }
    }

    /**
     * Every element of one connected component carries its own chart, and all those charts
     * are rigid images of the same lattice parameterization. Propagating from any of them
     * must therefore recover that parameterization, expressed in the chart of the start
     * element, on every vertex of the component, whatever the order in which the traversal
     * visits the elements.
     */
    TYPED_TEST(OctahedralRotationsDimTest, propagate_consistent_transitions) {
        constexpr GEO::index_t DIM = TypeParam::value;
        constexpr GEO::index_t N = (DIM == 2) ? 3 : 2;

        this->build_grid(N);
        this->create_uv_attributes();

        /* Several draws of the per-element charts: a single draw could happen to hide a
         * transition that is not composed correctly. */
        for (const GEO::Numeric::uint64 seed : {0u, 1u, 2u, 3u}) {
            this->randomize_charts(seed);

            for (GEO::index_t start = 0, start_end = this->elements_nb(); start < start_end; ++start) {
                this->reset_vertex_uvs();
                this->propagate(start);

                for (GEO::index_t v = 0, v_end = this->mesh_.vertices.nb(); v < v_end; ++v) {
                    EXPECT_TRUE(nearly_equal(this->vertex_uv()[v], this->expected_uv(start, v)))
                        << "seed " << seed << ", start element " << start << ", vertex " << v;
                }

                /* And every element must agree with the propagated parameterization as a
                 * whole, i.e. the transition of that element must be a rotation of the
                 * group. */
                for (GEO::index_t e = 0, e_end = this->elements_nb(); e < e_end; ++e) {
                    EXPECT_TRUE(this->element_is_consistent(e))
                        << "seed " << seed << ", start element " << start << ", element " << e;
                }
            }
        }

        EXPECT_TRUE(this->mesh_.save(get_current_test_name()+".geogram"));
    }

    /**
     * A propagation only walks the dual graph of the mesh, so it must leave everything that
     * is not part of the component of its start element untouched: the vertices of another
     * component, and the vertices that no element is incident to.
     */
    TYPED_TEST(OctahedralRotationsDimTest, propagate_consistent_transitions_keeps_other_components_untouched) {
        constexpr GEO::index_t DIM = TypeParam::value;
        constexpr GEO::index_t N = (DIM == 2) ? 2 : 1;

        this->build_grid(N);
        const auto grid_vertices_nb = this->mesh_.vertices.nb();
        const auto grid_elements_nb = this->elements_nb();

        this->append_isolated_vertex();
        const auto isolated_v0 = this->mesh_.vertices.nb();
        const auto isolated_e = this->append_isolated_element();

        this->create_uv_attributes();
        this->randomize_charts(0);

        /* From the grid: the grid receives its parameterization, nothing else is written. */
        this->reset_vertex_uvs();
        this->propagate(0);

        for (GEO::index_t v = 0; v < grid_vertices_nb; ++v) {
            EXPECT_TRUE(nearly_equal(this->vertex_uv()[v], this->expected_uv(0, v)))
                << "vertex " << v << " of the grid";
        }
        for (GEO::index_t v = grid_vertices_nb, v_end = this->mesh_.vertices.nb(); v < v_end; ++v) {
            EXPECT_TRUE(nearly_equal(this->vertex_uv()[v], this->unreached_uv()))
                << "vertex " << v << " outside of the grid";
        }

        /* And symmetrically: the component of the isolated element is the only one a
         * propagation started from it may touch. */
        this->reset_vertex_uvs();
        this->propagate(isolated_e);

        for (GEO::index_t v = 0; v < isolated_v0; ++v) {
            EXPECT_TRUE(nearly_equal(this->vertex_uv()[v], this->unreached_uv()))
                << "vertex " << v << " outside of the isolated element";
        }
        for (GEO::index_t v = isolated_v0, v_end = this->mesh_.vertices.nb(); v < v_end; ++v) {
            EXPECT_TRUE(nearly_equal(this->vertex_uv()[v], this->expected_uv(isolated_e, v)))
                << "vertex " << v << " of the isolated element";
        }

        /* The two components of that mesh are the grid and the isolated element. */
        EXPECT_EQ(grid_elements_nb+1, this->elements_nb());

        EXPECT_TRUE(this->mesh_.save(get_current_test_name()+".geogram"));
    }
}
