//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/14.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_OCTAHEDRAL_ROTATIONS_H
#define GEOLIO_OCTAHEDRAL_ROTATIONS_H
#include <geogram/basic/geometry.h>
#include <geogram/mesh/mesh.h>
#include <cassert>
#include <array>
#include <cmath>
#include <limits>

namespace geolio
{
    /**
     * The 4 orientation-preserving symmetry matrices of the square: the cyclic
     * group C4 of rotations by multiples of 90 degrees, which is the 2D analogue
     * of the octahedral rotation group (there is no octahedral group in 2D).
     * @details Each 2x2 matrix is a signed axis permutation with det=+1, used to
     * enumerate orientation candidates for frame/element alignment. The
     * reflections of the full square symmetry group D4 (det=-1) are deliberately
     * excluded, because a chart transition must preserve orientation.
     */
    const std::array<GEO::mat2, 4> OCTAHEDRAL_ROTATIONS_2D = {
        {
            {{1, 0}, {0, 1}},
            {{0, -1}, {1, 0}},
            {{-1, 0}, {0, -1}},
            {{0, 1}, {-1, 0}}
        }
    };

    /**
     * Compute the 2D transition function between two parameterized triangles
     * that share an edge.
     *
     * @details The transition function is the rigid change of frame between the
     * two charts across their shared edge, i.e.
     * `uj = R * ui + t`, with a rotation `R` of `OCTAHEDRAL_ROTATIONS_2D` and an
     * integer translation `t`. `R` is recovered as the minimizer of the squared
     * distance between the source edge vector and its rotated image,
     * `|(uj2 - uj1) - R*(ui2 - ui1)|^2`, which is well defined and unique as soon
     * as the shared edge has non-zero length.
     *
     * @param[in] ui1 First endpoint of the shared edge, in the parameter domain
     *                 of the source chart.
     * @param[in] ui2 Second endpoint of the shared edge, in the parameter domain
     *                 of the source chart.
     * @param[in] uj1 Image of `ui1` in the parameter domain of the target chart.
     * @param[in] uj2 Image of `ui2` in the parameter domain of the target chart.
     * @param[out] ri Index in `OCTAHEDRAL_ROTATIONS_2D` of the selected rotation `R`.
     * @param[out] t Integer translation `t = uj1 - R*ui1`, rounded component-wise.
     *
     * @see compute_transition_function(const GEO::vec3&, const GEO::vec3&, const GEO::vec3&,
     *      const GEO::vec3&, const GEO::vec3&, const GEO::vec3&, GEO::index_t&, GEO::vec3&)
     *      for the tetrahedron-based 3D variant.
     */
    inline void compute_transition_function(
        const GEO::vec2& ui1, const GEO::vec2& ui2,
        const GEO::vec2& uj1, const GEO::vec2& uj2,
        GEO::index_t& ri,
        GEO::vec2& t
        ) {
        const auto ui2_vec = ui2-ui1;
        const auto uj2_vec = uj2-uj1;

        ri = GEO::NO_INDEX;
        double min_value = std::numeric_limits<double>::max();
        for (GEO::index_t i = 0; i < 4; ++i) {
            const auto& R = OCTAHEDRAL_ROTATIONS_2D[i];
            if (const double value = GEO::length2(uj2_vec - R*ui2_vec);
                value < min_value
                ) {
                ri = i;
                min_value = value;
            }
        }
        assert(ri != GEO::NO_INDEX);

        t = uj1 - OCTAHEDRAL_ROTATIONS_2D[ri]*ui1;
        t.x = std::round(t.x);
        t.y = std::round(t.y);
    }

    /**
     * Propagate the parameterization of one triangle to its whole connected
     * component.
     *
     * @details The input `mesh_fc_uv` is a per-corner parameterization: each
     * facet is free to use its own chart, so the UVs that two facets assign to a
     * shared vertex generally differ by a transition function `uj = R*ui + t`,
     * with a rotation `R` of `OCTAHEDRAL_ROTATIONS_2D` and an integer
     * translation `t`. This routine converts that per-facet data into a single
     * per-vertex parameterization `mesh_v_uv`, i.e. one integer-lattice chart
     * shared by the whole component, by composing those transitions along the
     * dual graph of the mesh.
     *
     * @param[in] mesh Triangulated surface mesh. Its facet adjacency must be up
     *                 to date (`mesh.facets.connect()`), otherwise every facet
     *                 looks like a boundary facet and nothing is propagated.
     * @param[in] start_f Facet that seeds the traversal and defines the reference
     *                 frame. Its connected component is the one that gets
     *                 parameterized.
     * @param[in] mesh_fc_uv Per-corner UVs, one `vec2` per facet corner, each
     *                 expressed in the chart of its own facet.
     * @param[in,out] mesh_v_uv Per-vertex UVs receiving the propagated
     *                 parameterization. Its incoming content is not read:
     *                 every vertex of a visited facet is written before it is
     *                 used. Vertices outside the component of `start_f` are left
     *                 untouched.
     *
     * @see compute_transition_function(const GEO::vec2&, const GEO::vec2&,
     *      const GEO::vec2&, const GEO::vec2&, GEO::index_t&, GEO::vec2&)
     *      for the transition composed between two adjacent facets.
     * @see propagate_consistent_transitions(const GEO::Mesh&, GEO::index_t,
     *      const GEO::Attribute<GEO::vec3>&, GEO::Attribute<GEO::vec3>&)
     *      for the tetrahedron-based 3D variant.
     */
    inline void propagate_consistent_transitions(
        const GEO::Mesh& mesh,
        const GEO::index_t start_f,
        const GEO::Attribute<GEO::vec2>& mesh_fc_uv,
        GEO::Attribute<GEO::vec2>& mesh_v_uv
        ) {
        assert(start_f < mesh.facets.nb());
        assert(mesh_fc_uv.is_bound());
        assert(mesh_fc_uv.size() == mesh.facet_corners.nb());
        assert(mesh_fc_uv.dimension() == 1);
        assert(mesh_v_uv.is_bound());
        assert(mesh_v_uv.size() == mesh.vertices.nb());
        assert(mesh_v_uv.dimension() == 1);

        std::vector<bool> processed_facets(mesh.facets.nb(), false);

        /* Init */
        mesh_v_uv[mesh.facets.vertex(start_f, 0)] = mesh_fc_uv[mesh.facets.corner(start_f, 0)];
        mesh_v_uv[mesh.facets.vertex(start_f, 1)] = mesh_fc_uv[mesh.facets.corner(start_f, 1)];
        mesh_v_uv[mesh.facets.vertex(start_f, 2)] = mesh_fc_uv[mesh.facets.corner(start_f, 2)];

        std::vector<GEO::index_t> stack;
        stack.push_back(start_f);
        processed_facets[start_f] = true;

        /* Propagate */
        while (!stack.empty()) {
            const auto f = stack.back();
            stack.pop_back();

            for (GEO::index_t lv = 0; lv < 3; ++lv) {
                const auto& nf = mesh.facets.adjacent(f, lv);
                if (nf == GEO::NO_FACET || processed_facets[nf])
                    continue;

                const auto& v0 = mesh.facets.vertex(f, lv);
                const auto& v1 = mesh.facets.vertex(f, (lv+1)%3);
                const auto& nf_lv0 = mesh.facets.find_vertex(nf, v0);
                const auto& nf_lv1 = mesh.facets.find_vertex(nf, v1);
                assert(nf_lv0 != GEO::NO_INDEX);
                assert(nf_lv1 != GEO::NO_INDEX);
                const auto nf_lv2 = 0^1^2^nf_lv0^nf_lv1;
                assert(nf_lv2 < 3 && nf_lv2 != nf_lv0 && nf_lv2 != nf_lv1);

                /* Compute transition function */
                const auto& f_uv0 = mesh_v_uv[v0];
                const auto& f_uv1 = mesh_v_uv[v1];
                const auto& nf_uv0 = mesh_fc_uv[mesh.facets.corner(nf, nf_lv0)];
                const auto& nf_uv1 = mesh_fc_uv[mesh.facets.corner(nf, nf_lv1)];
                const auto& nf_uv2 = mesh_fc_uv[mesh.facets.corner(nf, nf_lv2)];
                const GEO::vec2 nf_uv2_vec = nf_uv2 - nf_uv0;

                GEO::index_t ri;
                GEO::vec2 t;
                compute_transition_function(
                    nf_uv0, nf_uv1,
                    f_uv0, f_uv1,
                    ri, t);
                assert(ri < 4);
                const auto& R = OCTAHEDRAL_ROTATIONS_2D[ri];

                /* Apply transition */
                mesh_v_uv[mesh.facets.vertex(nf, nf_lv2)] = f_uv0 + R*nf_uv2_vec;

                stack.push_back(nf);
                processed_facets[nf] = true;
            }
        }
    }

    /**
     * The 24 proper rotation matrices of the octahedral (cube) symmetry group:
     * all signed axis permutations with det=+1, i.e. the full orientation-preserving
     * automorphism group of the integer lattice Z^3.
     * @details Each 3x3 matrix is a right-handed axis permutation/sign-flip (det=1),
     * used to enumerate orientation candidates for frame/element alignment.
     */
    const std::array<GEO::mat3, 24> OCTAHEDRAL_ROTATIONS_3D = {
        {
            {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
            {{1, 0, 0}, {0, -1, 0}, {0, 0, -1}},
            {{-1, 0, 0}, {0, 1, 0}, {0, 0, -1}},
            {{-1, 0, 0}, {0, -1, 0}, {0, 0, 1}},
            {{1, 0, 0}, {0, 0, 1}, {0, -1, 0}},
            {{1, 0, 0}, {0, 0, -1}, {0, 1, 0}},
            {{-1, 0, 0}, {0, 0, 1}, {0, 1, 0}},
            {{-1, 0, 0}, {0, 0, -1}, {0, -1, 0}},
            {{0, 1, 0}, {1, 0, 0}, {0, 0, -1}},
            {{0, 1, 0}, {-1, 0, 0}, {0, 0, 1}},
            {{0, -1, 0}, {1, 0, 0}, {0, 0, 1}},
            {{0, -1, 0}, {-1, 0, 0}, {0, 0, -1}},
            {{0, 1, 0}, {0, 0, 1}, {1, 0, 0}},
            {{0, 1, 0}, {0, 0, -1}, {-1, 0, 0}},
            {{0, -1, 0}, {0, 0, 1}, {-1, 0, 0}},
            {{0, -1, 0}, {0, 0, -1}, {1, 0, 0}},
            {{0, 0, 1}, {1, 0, 0}, {0, 1, 0}},
            {{0, 0, 1}, {-1, 0, 0}, {0, -1, 0}},
            {{0, 0, -1}, {1, 0, 0}, {0, -1, 0}},
            {{0, 0, -1}, {-1, 0, 0}, {0, 1, 0}},
            {{0, 0, 1}, {0, 1, 0}, {-1, 0, 0}},
            {{0, 0, 1}, {0, -1, 0}, {1, 0, 0}},
            {{0, 0, -1}, {0, 1, 0}, {1, 0, 0}},
            {{0, 0, -1}, {0, -1, 0}, {-1, 0, 0}}
        }
    };

    /**
     * Compute the transition function between two parameterized tetrahedra that
     * share a face.
     *
     * @details The transition function is the rigid change of frame between the
     * two charts across their shared triangle, i.e.
     * `uj = R * ui + t`, with a rotation `R` of `OCTAHEDRAL_ROTATIONS_3D` and an
     * integer translation `t`. `R` is recovered as the minimizer of the summed
     * squared distance between the two source edge vectors and their rotated
     * images, `|(uj2 - uj1) - R*(ui2 - ui1)|^2 + |(uj3 - uj1) - R*(ui3 - ui1)|^2`.
     * Because two edges of a non-degenerate triangle span the plane they define
     * and separate the 24 rotations, the minimizer is unique on valid input.
     *
     * @param[in] ui1 First vertex of the shared triangle, in the parameter domain
     *                 of the source chart.
     * @param[in] ui2 Second vertex of the shared triangle, in the parameter domain
     *                 of the source chart.
     * @param[in] ui3 Third vertex of the shared triangle, in the parameter domain
     *                 of the source chart.
     * @param[in] uj1 Image of `ui1` in the parameter domain of the target chart.
     * @param[in] uj2 Image of `ui2` in the parameter domain of the target chart.
     * @param[in] uj3 Image of `ui3` in the parameter domain of the target chart.
     * @param[out] ri Index in `OCTAHEDRAL_ROTATIONS_3D` of the selected rotation `R`.
     * @param[out] t Integer translation `t = uj1 - R*ui1`, rounded component-wise.
     *
     * @see HexHex: Highspeed Extraction of Hexahedral Meshes, Section 5.1.1.
     */
    inline void compute_transition_function(
        const GEO::vec3& ui1, const GEO::vec3& ui2, const GEO::vec3& ui3,
        const GEO::vec3& uj1, const GEO::vec3& uj2, const GEO::vec3& uj3,
        GEO::index_t& ri,
        GEO::vec3& t
        ) {
        const auto ui2_vec = ui2-ui1;
        const auto ui3_vec = ui3-ui1;
        const auto uj2_vec = uj2-uj1;
        const auto uj3_vec = uj3-uj1;

        ri = GEO::NO_INDEX;
        double min_value = std::numeric_limits<double>::max();
            for (GEO::index_t i = 0; i < 24; ++i) {
                const auto& R = OCTAHEDRAL_ROTATIONS_3D[i];
                if (const double value = GEO::length2(uj2_vec - R*ui2_vec) + GEO::length2(uj3_vec - R*ui3_vec);
                    value < min_value
                    ) {
                    ri = i;
                    min_value = value;
                }
            }
        assert(ri != GEO::NO_INDEX);

        t = uj1 - OCTAHEDRAL_ROTATIONS_3D[ri]*ui1;
        t.x = std::round(t.x);
        t.y = std::round(t.y);
        t.z = std::round(t.z);
    }

    /**
     * Propagate the parameterization of one tetrahedron to its whole connected
     * component.
     *
     * @details The 3D counterpart of
     * `propagate_consistent_transitions(const GEO::Mesh&, GEO::index_t, const GEO::Attribute<GEO::vec2>&, GEO::Attribute<GEO::vec2>&)`.
     * The input `mesh_cc_uvw` is a per-corner parameterization: each cell is free
     * to use its own chart, so the UVWs that two cells assign to a shared vertex
     * generally differ by a transition function `uj = R*ui + t`, with a rotation
     * `R` of `OCTAHEDRAL_ROTATIONS_3D` and an integer translation `t`. This
     * routine converts that per-cell data into a single per-vertex
     * parameterization `mesh_v_uvw`, i.e. one integer-lattice chart shared by the
     * whole component, by composing those transitions along the dual graph of the
     * mesh.
     *
     * @param[in] mesh Tetrahedral mesh. Its cell adjacency must be up to date
     *                 (`mesh.cells.connect()`), otherwise every cell looks like a
     *                 boundary cell and nothing is propagated.
     * @param[in] start_c Cell that seeds the traversal and defines the reference
     *                 frame. Its connected component is the one that gets
     *                 parameterized.
     * @param[in] mesh_cc_uvw Per-corner UVWs, one `vec3` per cell corner, each
     *                 expressed in the chart of its own cell.
     * @param[in,out] mesh_v_uvw Per-vertex UVWs receiving the propagated
     *                 parameterization. Its incoming content is not read: every
     *                 vertex of a visited cell is written before it is used.
     *                 Vertices outside the component of `start_c` are left
     *                 untouched.
     *
     * @see HexHex: Highspeed Extraction of Hexahedral Meshes, Section 5.1.1.
     */
    inline void propagate_consistent_transitions(
        const GEO::Mesh& mesh,
        const GEO::index_t start_c,
        const GEO::Attribute<GEO::vec3>& mesh_cc_uvw,
        GEO::Attribute<GEO::vec3>& mesh_v_uvw
        ) {
        assert(start_c < mesh.cells.nb());
        assert(mesh_cc_uvw.is_bound());
        assert(mesh_cc_uvw.size() == mesh.cell_corners.nb());
        assert(mesh_cc_uvw.dimension() == 1);
        assert(mesh_v_uvw.is_bound());
        assert(mesh_v_uvw.size() == mesh.vertices.nb());
        assert(mesh_v_uvw.dimension() == 1);

        std::vector<bool> processed_cells(mesh.cells.nb(), false);

        /* Init */
        mesh_v_uvw[mesh.cells.vertex(start_c, 0)] = mesh_cc_uvw[mesh.cells.corner(start_c, 0)];
        mesh_v_uvw[mesh.cells.vertex(start_c, 1)] = mesh_cc_uvw[mesh.cells.corner(start_c, 1)];
        mesh_v_uvw[mesh.cells.vertex(start_c, 2)] = mesh_cc_uvw[mesh.cells.corner(start_c, 2)];
        mesh_v_uvw[mesh.cells.vertex(start_c, 3)] = mesh_cc_uvw[mesh.cells.corner(start_c, 3)];

        std::vector<GEO::index_t> stack;
        stack.push_back(start_c);
        processed_cells[start_c] = true;

        /* Propagate */
        while (!stack.empty()) {
            const auto c = stack.back();
            stack.pop_back();

            for (GEO::index_t lf = 0; lf < 4; ++lf) {
                const auto& nc = mesh.cells.adjacent(c, lf);
                if (nc == GEO::NO_CELL || processed_cells[nc])
                    continue;

                const auto& v0 = mesh.cells.facet_vertex(c, lf, 0);
                const auto& v1 = mesh.cells.facet_vertex(c, lf, 1);
                const auto& v2 = mesh.cells.facet_vertex(c, lf, 2);
                const auto& nc_lv0 = mesh.cells.find_tet_vertex(nc, v0);
                const auto& nc_lv1 = mesh.cells.find_tet_vertex(nc, v1);
                const auto& nc_lv2 = mesh.cells.find_tet_vertex(nc, v2);
                assert(nc_lv0 != GEO::NO_INDEX);
                assert(nc_lv1 != GEO::NO_INDEX);
                assert(nc_lv2 != GEO::NO_INDEX);
                const auto nc_lv3 = nc_lv0^nc_lv1^nc_lv2; // 0^1^2^3^ = 0^
                assert(nc_lv3 < 4 && nc_lv3 != nc_lv0 && nc_lv3 != nc_lv1 && nc_lv3 != nc_lv2);

                /* Compute transition function */
                const auto& c_uvw0 = mesh_v_uvw[v0];
                const auto& c_uvw1 = mesh_v_uvw[v1];
                const auto& c_uvw2 = mesh_v_uvw[v2];
                const auto& nc_uvw0 = mesh_cc_uvw[mesh.cells.corner(nc, nc_lv0)];
                const auto& nc_uvw1 = mesh_cc_uvw[mesh.cells.corner(nc, nc_lv1)];
                const auto& nc_uvw2 = mesh_cc_uvw[mesh.cells.corner(nc, nc_lv2)];
                const auto& nc_uvw3 = mesh_cc_uvw[mesh.cells.corner(nc, nc_lv3)];
                const GEO::vec3 nc_uvw3_vec = nc_uvw3 - nc_uvw0;

                GEO::index_t ri;
                GEO::vec3 t;
                compute_transition_function(
                    nc_uvw0, nc_uvw1, nc_uvw2,
                    c_uvw0, c_uvw1, c_uvw2,
                    ri, t);
                assert(ri < 24);
                const auto& R = OCTAHEDRAL_ROTATIONS_3D[ri];

                /* Apply transition */
                mesh_v_uvw[mesh.cells.vertex(nc, nc_lv3)] = c_uvw0 + R*nc_uvw3_vec;

                stack.push_back(nc);
                processed_cells[nc] = true;
            }
        }
    }
}
#endif //GEOLIO_OCTAHEDRAL_ROTATIONS_H
