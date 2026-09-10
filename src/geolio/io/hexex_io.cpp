//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/29.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "hexex_io.h"
#include <ranges>
#include <unordered_set>
#include "line_stream.h"
#include <geolio/common/log.h>
#include "geolio/common/array_hash.h"

namespace geolio
{
    bool HEXEX_IOHandler::load(
        const std::string& filename,
        GEO::Mesh& mesh,
        const GEO::MeshIOFlags& ioflags
        ) {
        mesh.clear();
        mesh.vertices.set_dimension(3);

        LineInput in(filename);
        if (!in.OK()) {
            LOG::ERROR("Cannot load file `{}`!", filename);
            return false;
        }

        /* Load vertices */
        {
            in.get_line();
            in.get_fields();
            if (in.nb_fields() != 1) {
                LOG::ERROR("{}", "Line "+std::to_string(in.line_number())+" :Expect vertices nb!");
                return false;
            }

            const GEO::index_t nb_vertices = in.field_as_uint(0);
            mesh.vertices.create_vertices(nb_vertices);

            for (GEO::index_t v = 0; v < nb_vertices; ++v) {
                in.get_line();
                in.get_fields();
                if (in.nb_fields() != 3) {
                    LOG::ERROR("{}", "Line "+std::to_string(in.line_number())+" :Invalid vertex, expected 3 coordinates!");
                    return false;
                }

                mesh.vertices.point(v).x = in.field_as_double(0);
                mesh.vertices.point(v).y = in.field_as_double(1);
                mesh.vertices.point(v).z = in.field_as_double(2);
            }
        }


        /* Load tetrahedra */
        {
            in.get_line();
            in.get_fields();
            if (in.nb_fields() != 1) {
                LOG::ERROR("{}", "Line "+std::to_string(in.line_number())+" :Expect tetrahedra nb!");
                return false;
            }

            const GEO::index_t nb_cells = in.field_as_uint(0);
            mesh.cells.create_tets(nb_cells);
            GEO::Attribute<GEO::vec3> cc_uvw(mesh.cell_corners.attributes(), "uvw");

            for (GEO::index_t c = 0; c < nb_cells; ++c) {
                in.get_line();
                in.get_fields();
                if(in.nb_fields() != 16) {
                    LOG::ERROR("{}", "Line "+std::to_string(in.line_number())+" :Invalid parameterized cell, expected 4 int + 12 double!");
                    return false;
                }

                for (GEO::index_t lv = 0; lv < 4; ++lv)
                    mesh.cells.set_vertex(c, lv, in.field_as_uint(lv));
                for (GEO::index_t lv = 0; lv < 4; ++lv) {
                    cc_uvw[mesh.cells.corner(c, lv)].x = in.field_as_appro_double(4+3*lv);
                    cc_uvw[mesh.cells.corner(c, lv)].y = in.field_as_appro_double(4+3*lv+1);
                    cc_uvw[mesh.cells.corner(c, lv)].z = in.field_as_appro_double(4+3*lv+2);
                }
            }
        }

        /* For MC3D-samples (https://github.com/HendrikBrueckler/MC3D-samples) */
        {
            while (in.get_line()) {
                in.get_fields();

                if (in.nb_fields() == 1) { /* Load wall triangles */
                    const GEO::index_t nb_wall_triangles = in.field_as_uint(0);
                    mesh.facets.create_triangles(nb_wall_triangles);
                    GEO::Attribute<double> f_dist_to_origin(mesh.facets.attributes(), "dist_to_origin");
                    f_dist_to_origin.fill(0);

                    for (GEO::index_t f = 0; f < nb_wall_triangles; ++f) {
                        in.get_line();
                        in.get_fields();
                        if (in.nb_fields() != 4) {
                            LOG::ERROR("{}", "Line "+std::to_string(in.line_number())+" :Invalid wall triangles, expected 3 coordinates + 1 double!");
                            return false;
                        }

                        mesh.facets.set_vertex(f, 0, in.field_as_uint(0));
                        mesh.facets.set_vertex(f, 1, in.field_as_uint(1));
                        mesh.facets.set_vertex(f, 2, in.field_as_uint(2));
                        f_dist_to_origin[f] = in.field_as_double(3);
                    }
                }
                else if (in.nb_fields() == 3) { /* Load features */
                    const GEO::index_t nb_feature_vertices = in.field_as_uint(0);
                    const GEO::index_t nb_feature_edges = in.field_as_uint(1);
                    const GEO::index_t nb_feature_facets = in.field_as_uint(2);

                    /* Load feature vertices */
                    if (nb_feature_vertices > 0) {
                        GEO::Attribute<bool> v_feature(mesh.vertices.attributes(), "feature");
                        v_feature.fill(false);

                        for (GEO::index_t i = 0; i < nb_feature_vertices; ++i) {
                            in.get_line();
                            in.get_fields();
                            if (in.nb_fields() != 1) {
                                LOG::ERROR("{}", "Line "+std::to_string(in.line_number())+" :Invalid feature vertex index!");
                                return false;
                            }

                            v_feature[in.field_as_uint(0)] = true;
                        }
                    }

                    /* Load feature edges */
                    if (nb_feature_edges > 0) {
                        mesh.edges.create_edges(nb_feature_edges);
                        GEO::Attribute<bool> e_feature(mesh.edges.attributes(), "feature");
                        e_feature.fill(false);

                        for (GEO::index_t e = 0; e < nb_feature_edges; ++e) {
                            in.get_line();
                            in.get_fields();
                            if (in.nb_fields() != 2) {
                                LOG::ERROR("{}", "Line "+std::to_string(in.line_number())+" :Invalid features edge vertices!");
                                return false;
                            }

                            mesh.edges.set_vertex(e, 0, in.field_as_uint(0));
                            mesh.edges.set_vertex(e, 1, in.field_as_uint(1));

                            e_feature[e] = true;
                        }

                    }

                    /* Load feature facets */
                    if (nb_feature_facets > 0) {
                        std::vector<GEO::index_t> feature_facets; // v0, v1, v2, ...
                        feature_facets.reserve(3*nb_feature_facets);

                        for (GEO::index_t f = 0; f < nb_feature_facets; ++f) {
                            in.get_line();
                            in.get_fields();
                            if (in.nb_fields() != 3) {
                                LOG::ERROR("{}", "Line "+std::to_string(in.line_number())+" :Invalid features facet vertices!");
                                return false;
                            }

                            feature_facets.push_back(in.field_as_uint(0));
                            feature_facets.push_back(in.field_as_uint(1));
                            feature_facets.push_back(in.field_as_uint(2));
                        }

                        /* Label feature facets */
                        GEO::Attribute<bool> f_feature(mesh.facets.attributes(), "feature");
                        f_feature.fill(false);

                        std::unordered_map<std::array<GEO::index_t, 3>, GEO::index_t, ArrayHash<GEO::index_t, 3>> feature_facets_set; // (v0, v1, v2) -> f
                        feature_facets_set.reserve(nb_feature_facets);
                        for (GEO::index_t f = 0; f < nb_feature_facets; ++f) {
                            std::array<GEO::index_t, 3> fvs{
                               feature_facets[3*f], feature_facets[3*f+1], feature_facets[3*f+2]
                            };
                            std::ranges::sort(fvs);
                            feature_facets_set.emplace(fvs, f);
                        }
                        {
                            std::vector<bool> feature_facets_found(nb_feature_facets, false);
                            for (const auto& f : mesh.facets) {
                                std::array<GEO::index_t, 3> fvs{
                                    mesh.facets.vertex(f, 0), mesh.facets.vertex(f, 1), mesh.facets.vertex(f, 2)
                                };
                                std::ranges::sort(fvs);
                                if (auto it = feature_facets_set.find(fvs);
                                    it != feature_facets_set.end()) {
                                    f_feature[f] = true;
                                    feature_facets_found[it->second] = true;
                                }
                            }

                            GEO::index_t remain_facets_nb = 0;
                            for (GEO::index_t f = 0; f < nb_feature_facets; ++f) {
                                if (!feature_facets_found[f])
                                    ++remain_facets_nb;
                            }

                            GEO::index_t new_f = mesh.facets.create_triangles(remain_facets_nb);
                            for (GEO::index_t f = 0; f < nb_feature_facets; ++f) {
                                if (!feature_facets_found[f]) {
                                    mesh.facets.set_vertex(new_f, 0, feature_facets[3*f]);
                                    mesh.facets.set_vertex(new_f, 1, feature_facets[3*f+1]);
                                    mesh.facets.set_vertex(new_f, 2, feature_facets[3*f+2]);
                                    f_feature[new_f] = true;
                                    ++new_f;
                                }
                            }
                        }
                    }
                }
                else {
                    LOG::ERROR("{}", "Line "+std::to_string(in.line_number())+" :Invalid keyword!");
                    return false;
                }
            }
        }

        mesh.facets.connect();
        mesh.cells.connect();

        return true;
    }
}
