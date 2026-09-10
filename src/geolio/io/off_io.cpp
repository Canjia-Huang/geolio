//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/5.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "off_io.h"
#include "line_stream.h"
#include <geolio/common/log.h>

namespace geolio
{
    bool OFF_IOHandler::load(
        const std::string& filename,
        GEO::Mesh& mesh,
        const GEO::MeshIOFlags& ioflags
        ) {
        mesh.clear();
        mesh.vertices.set_dimension(3);

        LineInput in(filename);
        if(!in.OK()) {
            LOG::ERROR("Cannot load file `{}`!", filename);
            return false;
        }

        /* Header */
        in.get_line();
        in.get_fields();
        GEO::Attribute<GEO::vec3> v_normal; // bounded when load "N OFF"
        if (in.nb_fields() == 1) {
            if (!in.field_matches(0, "OFF")) {
                LOG::ERROR("Line {} :Unrecognized header, expect `OFF`!", in.line_number());
                return false;
            }
        }
        else {
            if (in.nb_fields() != 2) {
                LOG::ERROR("Line {} :Unrecognized header number!", in.line_number());
                return false;
            }
            if (in.field_matches(0, "N") && in.field_matches(1, "OFF"))
                v_normal.bind(mesh.vertices.attributes(), "normal");
            else {
                LOG::ERROR("Line {} :Unrecognized header, expect `N OFF`!", in.line_number());
                return false;
            }
        }

        /* Elements nb */
        in.get_line();
        in.get_fields();
        if (in.nb_fields() != 3) {
            LOG::ERROR("Line {} :Unrecognized elements nb number, expect 3!", in.line_number());
            return false;
        }

        const GEO::index_t nb_vertices = in.field_as_uint(0);
        const GEO::index_t nb_facets = in.field_as_uint(1);
        const GEO::index_t nb_edges = in.field_as_uint(2); // unused

        /* == Vertices ========================================================================================= */
        if (nb_vertices > 0) {
            mesh.vertices.create_vertices(nb_vertices);
            for (GEO::index_t v = 0; v < nb_vertices; ++v) {
                in.get_line();
                in.get_fields();
                if (v_normal.is_bound()) {
                    if (in.nb_fields() != 6) {
                        LOG::ERROR("Line {} :Invalid number of fields: expect 6!", in.line_number());
                        return false;
                    }
                    mesh.vertices.point(v).x = in.field_as_double(0);
                    mesh.vertices.point(v).y = in.field_as_double(1);
                    mesh.vertices.point(v).z = in.field_as_double(2);
                    v_normal[v] = GEO::vec3(in.field_as_double(3), in.field_as_double(4), in.field_as_double(5));
                }
                else {
                    if (in.nb_fields() != 3) {
                        LOG::ERROR("Line {} :Invalid number of fields: expect 3!", in.line_number());
                        return false;
                    }
                    mesh.vertices.point(v).x = in.field_as_double(0);
                    mesh.vertices.point(v).y = in.field_as_double(1);
                    mesh.vertices.point(v).z = in.field_as_double(2);
                }
            }
        }

        /* == Facets =========================================================================================== */
        if (nb_facets > 0) {
            std::vector<GEO::index_t> facet_ptr; // nb_vertices, v0, v1, ..., vn, nb_vertices, v1, ...
            for (GEO::index_t f = 0; f < nb_facets; ++f) {
                in.get_line();
                in.get_fields();
                if (in.nb_fields() == 0) {
                    LOG::ERROR("Line {} :Invalid facet, empty line!", in.line_number());
                    return false;
                }

                const GEO::index_t fv_nb = in.field_as_uint(0);
                if (in.nb_fields() != fv_nb+1) {
                    LOG::ERROR("Line {} :Facet need to have {} vertices, actual vertices {}!", in.line_number(), fv_nb, in.nb_fields()-1);
                    return false;
                }
                facet_ptr.push_back(fv_nb);

                for (GEO::index_t lv = 0; lv < fv_nb; ++lv)
                    facet_ptr.push_back(in.field_as_uint(1+lv)); // Note: Vertices indexes start by 0 in off format.
            }

            /* Create facets */
            constexpr GEO::index_t FACET_TYPE_TRI = 0;
            constexpr GEO::index_t FACET_TYPE_QUAD = 1;
            constexpr GEO::index_t FACET_TYPE_POLY = 2;
            for (GEO::index_t begin_fi = 0, fi_end = facet_ptr.size(); begin_fi < fi_end;) {
                GEO::index_t end_fi = begin_fi;
                GEO::index_t facets_nb = 0;
                GEO::index_t facet_type; // FACET_TYPE_TRI, FACET_TYPE_QUAD, FACET_TYPE_POLY
                {
                    if (const GEO::index_t fv_nb = facet_ptr[begin_fi];
                        fv_nb == 3)
                        facet_type = FACET_TYPE_TRI;
                    else if (fv_nb == 4)
                        facet_type = FACET_TYPE_QUAD;
                    else
                        facet_type = FACET_TYPE_POLY;
                }

                while (end_fi < fi_end) {
                    if (const GEO::index_t fv_nb = facet_ptr[end_fi];
                        (facet_type == FACET_TYPE_TRI  && fv_nb == 3) ||
                        (facet_type == FACET_TYPE_QUAD && fv_nb == 4) ||
                        (facet_type == FACET_TYPE_POLY && fv_nb > 4)
                        ) {
                        ++facets_nb;
                        end_fi += fv_nb+1;
                        }
                    else
                        break;
                }

                if (facet_type == FACET_TYPE_TRI) {
                    GEO::index_t new_f = mesh.facets.create_triangles(facets_nb);
                    for (; begin_fi < end_fi; ++begin_fi) {
                        assert(facet_ptr[begin_fi] == 3);
                        mesh.facets.set_vertex(new_f, 0, facet_ptr[begin_fi+1]);
                        mesh.facets.set_vertex(new_f, 1, facet_ptr[begin_fi+2]);
                        mesh.facets.set_vertex(new_f, 2, facet_ptr[begin_fi+3]);
                        ++new_f;
                        begin_fi += 3;
                    }
                    assert(new_f == mesh.facets.nb());
                }
                else if (facet_type == FACET_TYPE_QUAD) {
                    GEO::index_t new_f = mesh.facets.create_quads(facets_nb);
                    for (; begin_fi < end_fi; ++begin_fi) {
                        assert(facet_ptr[begin_fi] == 4);
                        mesh.facets.set_vertex(new_f, 0, facet_ptr[begin_fi+1]);
                        mesh.facets.set_vertex(new_f, 1, facet_ptr[begin_fi+2]);
                        mesh.facets.set_vertex(new_f, 2, facet_ptr[begin_fi+3]);
                        mesh.facets.set_vertex(new_f, 3, facet_ptr[begin_fi+4]);
                        ++new_f;
                        begin_fi += 4;
                    }
                    assert(new_f == mesh.facets.nb());
                }
                else {
                    assert(facet_type == FACET_TYPE_POLY);
                    mesh.facets.reserve(facets_nb);
                    for (; begin_fi < end_fi; ++begin_fi) {
                        const GEO::index_t fv_nb = facet_ptr[begin_fi];
                        if (fv_nb > 4) {
                            const GEO::index_t new_f = mesh.facets.create_polygon(fv_nb);
                            for (GEO::index_t lv = 0; lv < fv_nb; ++lv)
                                mesh.facets.set_vertex(new_f, lv, facet_ptr[begin_fi+lv+1]);
                        }
                        begin_fi += fv_nb;
                    }
                }

                assert(begin_fi == end_fi);
            }

            mesh.facets.connect();
        }

        /* == Edges ============================================================================================ */
        if (nb_edges > 0) {
            std::vector<GEO::index_t> edge_ptr; // ev0, ev1, ev0, ...
            edge_ptr.reserve(2*nb_edges);
            for (GEO::index_t e = 0; e < nb_edges; ++e) {
                if (!in.get_line())
                    break;
                in.get_fields();
                if (in.nb_fields() != 3) {
                    LOG::ERROR("Line {} :Invalid number of fields: expect 1+2!", in.line_number());
                    return false;
                }
                if (in.field_as_uint(0) != 2) {
                    LOG::ERROR("Line {} :Invalid edge vertices nb, expect 2!", in.line_number());
                    return false;
                }
                edge_ptr.push_back(in.field_as_uint(1));
                edge_ptr.push_back(in.field_as_uint(2));
            }

            if (!edge_ptr.empty()) {
                mesh.edges.create_edges(edge_ptr.size()/2);
                for (const auto& e : mesh.edges) {
                    mesh.edges.set_vertex(e, 0, edge_ptr[2*e]);
                    mesh.edges.set_vertex(e, 1, edge_ptr[2*e+1]);
                }
            }
        }

        return true;
    }

    bool OFF_IOHandler::save(
        const GEO::Mesh& mesh,
        const std::string& filename,
        const GEO::MeshIOFlags& ioflags
        ) {
        std::ofstream out(filename.c_str());
        if(!out) {
            LOG::ERROR("Cannot save file `{}`!", filename);
            return false;
        }

        /* Header */
        out << "OFF" << std::endl;

        out << mesh.vertices.nb() << " " << mesh.facets.nb() << " " << mesh.edges.nb() << std::endl;

        /* Output vertices */
        for (const auto& p : mesh.vertices.points())
            out << p.x << " " << p.y << " " << p.z << std::endl;

        /* Output facets */
        for (const auto& f : mesh.facets) {
            out << mesh.facets.nb_vertices(f) << " ";
            for (GEO::index_t fc = mesh.facets.corners_begin(f); fc < mesh.facets.corners_end(f); ++fc)
                out << mesh.facet_corners.vertex(fc) << " ";
            out << std::endl;
        }

        /* Output edges */
        for (const auto& e : mesh.edges)
            out << "2 " << mesh.edges.vertex(e, 0) << " " << mesh.edges.vertex(e, 1) << std::endl;

        return true;
    }
}