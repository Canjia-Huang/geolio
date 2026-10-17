//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/5.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_QUAD_MOTORCYCLE_GRAPH_H
#define GEOLIO_QUAD_MOTORCYCLE_GRAPH_H
#include <array>
#include <queue>
#include <utility>
#include <geogram/mesh/mesh.h>
#include "quad_motorcycle_block.h"

namespace geolio
{
    /**
     * Motorcycle graph for quadrilateral meshes (mesh based).
     *
     * This class computes a block decomposition of a quadrilateral mesh using the motorcycle
     * graph / base complex construction similar to the hex motorcycle complex construction
     * method described in the referenced paper. The algorithm propagates "motorcycles" from
     * singular and boundary vertices to build a structured partition of the mesh.
     *
     * @ref Brückler H, Gupta O, Mandad M, et al. The 3D motorcycle complex for structured volume decomposition[C]
     *      Computer Graphics Forum. 2022, 41(2): 221-235. (Section 5.1)
     */
    class QuadMotorCycleGraph {
    public:
        /**
         * Creates a motorcycle graph processor for the given quadrilateral mesh.
         *
         * @param[in] mesh Input quadrilateral mesh. All cells must be quads. The mesh is
         *                 stored by reference and must outlive this object.
         *
         * @pre `mesh` is a valid quad mesh with at least one cell.
         * @post Internal attributes and vertex classification are initialized for graph
         *       construction and propagation.
         */
        explicit QuadMotorCycleGraph(const GEO::Mesh& mesh);

        /**
         * Destroys the motorcycle graph and releases any bound temporary attributes.
         *
         * The destructor deallocates the internally created face-tag attribute if it is still
         * bound.
         */
        ~QuadMotorCycleGraph();

        /**
         * Motorcycle complex variants.
         */
        enum MotorCycleType {
           BASE_COMPLEX,        ///< Compute the base complex.
           MOTORCYCLE_COMPLEX   ///< Compute the motorcycle complex.
        };

        /**
         * Computes the selected complex and assigns block identifiers to the faces.
         *
         * @param[in] complex_type Complex variant to compute. Different variants may use
         *                         different propagation or stopping criteria.
         *
         * @return Total number of blocks.
         *
         * @post Every face receives a block index. The block indices form a contiguous sequence
         *       from 0 to return value - 1.
         */
        GEO::index_t compute(MotorCycleType complex_type = BASE_COMPLEX);

        /**
         * Labels every face in the input mesh with its block index.
         *
         * The block indices correspond to the ordering produced by `compute()`. Each block is
         * assigned a contiguous integer in the range `[0, nb_blocks - 1]`.
         *
         * @param[out] mesh_f_block Output attribute to receive the block id for each face.
         *                          The attribute must already be bound and sized to the mesh.
         *
         * @pre `compute()` has been called and `blocks_` is populated.
         * @post Each face entry in `mesh_f_block` contains the block index of the face.
         */
        void label_blocks(GEO::Attribute<GEO::index_t>& mesh_f_block) const;

        /**
         * Builds a coarse quadrilateral mesh from the computed block decomposition.
         *
         * Each coarse quad corresponds to one block in `blocks_`. The geometry and connectivity
         * are assembled from the block corner vertices inferred during decomposition.
         *
         * @param[out] mesh_out Output coarse quadrilateral mesh. Existing content may be
         *                      overwritten.
         * @param[out] old_fc_to_new_fc Optional map from original face-corner ids (`4*f + lv`)
         *                              to coarse mesh face ids. Pass `nullptr` to skip this
         *                              mapping.
         *
         * @pre `compute()` has been called and `blocks_` is valid.
         * @post `mesh_out` contains the generated coarse mesh.
         */
        void create_coarse_mesh(
           GEO::Mesh& mesh_out,
           std::vector<GEO::index_t>* old_fc_to_new_fc = nullptr) const;

        /**
         * Returns the computed block decomposition.
         */
        [[nodiscard]] const auto& blocks() const { return blocks_; }

    private:
        /**
         * Identifies singular and boundary vertices in the mesh.
         *
         * Singular vertices are detected from their local valence, and boundary vertices are
         * detected from incident face adjacency. These vertices serve as the ignition sources
         * for motorcycle propagation.
         *
         * @post `mesh_v_singular_` and `mesh_v_border_` are filled with the per-vertex status.
         */
        void find_all_singular_and_border_vertices();

        /**
         * Active fire front used during propagation.
         */
        struct Fire {
           GEO::index_t d;
           GEO::index_t f;
           GEO::index_t lv;

           bool operator<(const Fire& other) const {
               if (d != other.d) return d > other.d;
               if (f != other.f) return f > other.f;
               return lv > other.lv;
           }
        };

        /**
         * Inserts initial motorcycle fronts for singular and boundary vertices.
         *
         * @param[in,out] queue Priority queue that will be populated with the initial fire fronts.
         *                      Existing contents are preserved and cleared before insertion.
         *
         * @pre `find_all_singular_and_border_vertices()` has populated the vertex status maps.
         * @post `queue` contains all distance-0 fires that originate from the identified singular
         *       and boundary vertices.
         */
        void ignite(std::priority_queue<Fire>& queue) const;

        /**
         * Decomposes the mesh into connected blocks defined by the motorcycle propagation.
         *
         * @return Total number of blocks created.
         *
         * @pre `mesh_fc_tagged_` has been populated by propagation.
         * @post Each block is built from a connected set of faces and stored in `blocks_`.
         *       `mesh_f_block_` maps every face to the index of its block.
         */
        GEO::index_t decompose_into_blocks();

        /**
         * A directed local edge of a facet: the edge from `facet_corner(f, lv)` to
         * `facet_corner(f, (lv + 1) % 4)`.
         */
        struct LineEdge {
           GEO::index_t f{GEO::NO_INDEX};
           GEO::index_t lv{GEO::NO_INDEX};
        };

        /**
         * Kind of obstacle that terminates a traced grid line.
         */
        enum class LineEnd {
           MESH_BORDER, ///< A vertex on the mesh border was reached.
           WALL,        ///< A tagged (already burnt) edge was reached.
           CLOSED       ///< The line closed on itself.
        };

        /**
         * A whole grid line traced through a facet (both directions) and how it ends.
         */
        struct LineTrace {
           std::vector<LineEdge> edges; ///< Edges of the line, starting at the seed edge.
           LineEnd end_fwd{LineEnd::WALL}; ///< End reached when walking forward from the seed.
           LineEnd end_bwd{LineEnd::WALL}; ///< End reached when walking backward.
           bool closed{false}; ///< The line is a closed loop.
           GEO::index_t score{0}; ///< How well the line cuts a block open; higher is better.
        };

        /**
         * Tags both sides of an edge.
         *
         * @param[in] f Face holding the edge.
         * @param[in] lv Local edge index in `f`.
         * @param[in] tag Tag value to write (only `GEO::NO_INDEX` is interpreted as "untagged").
         * @return Number of facet corners that were tagged by this call (0 or 1 per side).
         */
        GEO::index_t tag_edge(GEO::index_t f, GEO::index_t lv, GEO::index_t tag);

        /**
         * Walks one step along the grid line through the local edge (@p f, @p lv).
         *
         * The step is the same "continue through the opposite edge" rule as the burning: the
         * line leaves the tip vertex of the edge and continues straight across it.
         *
         * @param[in] f Face holding the current edge.
         * @param[in] lv Local edge index of the current edge.
         * @param[in] block_id Block the line is not allowed to leave.
         * @param[in,out] ordered_f_lv Scratch buffer for the one-ring of the tip vertex.
         * @param[out] nf Face holding the next edge.
         * @param[out] nlv Local edge index of the next edge.
         * @param[out] reached_border Set when the line stopped on a mesh border vertex.
         * @return true when the line continues, false when it ends here.
         */
        bool next_line_edge(
           GEO::index_t f,
           GEO::index_t lv,
           GEO::index_t block_id,
           std::vector<std::pair<GEO::index_t, GEO::index_t>>& ordered_f_lv,
           GEO::index_t& nf,
           GEO::index_t& nlv,
           bool& reached_border) const;

        /**
         * Walks a grid line in one direction, collecting its edges.
         *
         * @param[in] f Face holding the first edge.
         * @param[in] lv Local edge index of the first edge.
         * @param[in] block_id Block the line is not allowed to leave.
         * @param[in,out] ordered_f_lv Scratch buffer for the one-rings.
         * @param[out] edges Edges of the line, in walking order. Existing content is kept.
         * @return How the walk ended.
         */
        LineEnd walk_line(
           GEO::index_t f,
           GEO::index_t lv,
           GEO::index_t block_id,
           std::vector<std::pair<GEO::index_t, GEO::index_t>>& ordered_f_lv,
           std::vector<LineEdge>& edges) const;

        /**
         * Traces the whole grid line of family @p k through the facet @p f0.
         *
         * @param[in] f0 Seed facet.
         * @param[in] lv_ref Local edge index that anchors the facet frame in the block.
         * @param[in] k Family: 0 or 1 (the two local edges `lv_ref+k` and `lv_ref+k+2`).
         * @param[in] block_id Block the line is not allowed to leave.
         * @param[in,out] ordered_f_lv Scratch buffer for the one-rings.
         * @param[out] trace The traced line.
         * @return false when the family has no usable edge in @p f0.
         */
        bool trace_line_family(
           GEO::index_t f0,
           GEO::index_t lv_ref,
           GEO::index_t k,
           GEO::index_t block_id,
           std::vector<std::pair<GEO::index_t, GEO::index_t>>& ordered_f_lv,
           LineTrace& trace) const;

        /**
         * Cuts a block open by tagging one of its grid lines as an artificial wall (a seam).
         *
         * @param[in] block Block to cut.
         * @param[in] block_id Index of @p block in `blocks_`.
         * @return true when at least one new edge was tagged.
         */
        bool cut_block(const QuadMotorCycleBlock& block, GEO::index_t block_id);

        /**
         * Checks whether a block is a cuboid with four distinct corner vertices.
         */
        static bool block_is_valid(const QuadMotorCycleBlock& block);

        /**
         * Finds the first block that is not a valid cuboid.
         *
         * @param[out] block_id Index of the offending block in `blocks_`.
         * @return true when an offending block was found.
         */
        bool find_invalid_block(GEO::index_t& block_id) const;

        const std::string attribute_id_;

        const GEO::Mesh& mesh_; // Input quad mesh
        GEO::Attribute<GEO::index_t> mesh_fc_tagged_; // [4*f+lv] -> distance tag or GEO::NO_INDEX
        GEO::Attribute<bool> mesh_v_singular_; // [v] -> singular vertex
        GEO::Attribute<bool> mesh_v_border_; // [v] -> border vertex

        std::vector<QuadMotorCycleBlock> blocks_;
        std::vector<GEO::index_t> mesh_f_block_; // [f] -> block id, or GEO::NO_INDEX
    };
}

#endif //GEOLIO_QUAD_MOTORCYCLE_GRAPH_H
