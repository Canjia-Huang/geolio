//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/3.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "ovm_io.h"

#include <geogram/basic/line_stream.h>

#include "line_stream.h"
#include <geolio/common/log.h>

namespace
{
    void strip_quotes(std::string& s) {
        if (s.size() < 2)
            return;

        const char first = s.front();
        const char last = s.back();
        if ((first == '"' && last == '"') || (first == '\'' && last == '\''))
            s = s.substr(1, s.size()-2);
    }
}

namespace geolio
{
    bool OVM_IOHandler::load(
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

        /* Header */
        in.get_line();
        in.get_fields();
        if(in.nb_fields() != 2 || strcmp(in.field(0),"OVM") != 0 || strcmp(in.field(1),"ASCII") != 0) {
            LOG::ERROR("Unsupported OVM file header `{}` - `{}`!", in.field(0), in.field(1));
            return false;
        }

        while(!in.eof()) {
            if (!in.get_line())
                break; // back empty line
            in.get_fields();
            if (in.nb_fields() == 0) {
                LOG::ERROR("{}", "Line "+std::to_string(in.line_number())+" :Invalid keyword format!");
                return false;
            }

            if (const std::string kw = in.field(0);
                kw == "Vertices") { // == Vertices ================================================================
                in.get_line();
                in.get_fields();
                if (in.nb_fields() != 1) {
                    LOG::ERROR("{}", "Line "+std::to_string(in.line_number())+" :Invalid vertices nb format!");
                    return false;
                }

                const GEO::index_t nb_vertices = in.field_as_uint(0);
                mesh.vertices.create_vertices(nb_vertices);

                for (GEO::index_t v = 0; v < nb_vertices; ++v) {
                    in.get_line();
                    in.get_fields();
                    if(in.nb_fields() != 3) {
                        LOG::ERROR("{}", "Line "+std::to_string(in.line_number())+" :Invalid vertex, expected 3 coordinates!");
                        return false;
                    }

                    auto& p = mesh.vertices.point(v);
                    p.x = in.field_as_double(0);
                    p.y = in.field_as_double(1);
                    p.z = in.field_as_double(2);
                }
            }
            else if (kw == "Edges") { // == Edges =================================================================
                in.get_line();
                in.get_fields();
                if (in.nb_fields() != 1) {
                    LOG::ERROR("{}", "Line "+std::to_string(in.line_number())+" :Invalid edges nb format!");
                    return false;
                }

                const GEO::index_t nb_edges = in.field_as_uint(0);
                mesh.edges.create_edges(nb_edges);

                for (GEO::index_t e = 0; e < nb_edges; ++e) {
                    in.get_line();
                    in.get_fields();
                    if(in.nb_fields() != 2) {
                        LOG::ERROR("{}", "Line "+std::to_string(in.line_number())+" :Invalid edge, expected 2 indices!");
                        return false;
                    }

                    mesh.edges.set_vertex(e, 0, in.field_as_uint(0));
                    mesh.edges.set_vertex(e, 1, in.field_as_uint(1));
                }
            }
            else if (kw == "Faces") { // == Facets =================================================================
                in.get_line();
                in.get_fields();
                if (in.nb_fields() != 1) {
                    LOG::ERROR("{}", "Line "+std::to_string(in.line_number())+" :Invalid facets nb format!");
                    return false;
                }

                const GEO::index_t nb_facets = in.field_as_uint(0);

                std::vector<GEO::index_t> facet_ptr; // nb_vertices, v0, v1, ..., vn, nb_vertices, v1, ...
                for (GEO::index_t f = 0; f < nb_facets; ++f) {
                    in.get_line();
                    in.get_fields();
                    if(in.nb_fields() == 0) {
                        LOG::ERROR("{}", "Line "+std::to_string(in.line_number())+" :Invalid facet, empty line!");
                        return false;
                    }

                    const GEO::index_t fv_nb = in.field_as_uint(0);
                    if (fv_nb < 3) {
                        LOG::ERROR("{}", "Line "+std::to_string(in.line_number())+" :Invalid facet vertices nb "+std::to_string(fv_nb)+"!");
                        return false;
                    }

                    if (in.nb_fields() != fv_nb+1) {
                        LOG::ERROR("{}", "Line "+std::to_string(in.line_number())+" :Invalid facet, wrong number of elements!");
                        return false;
                    }
                    facet_ptr.push_back(fv_nb);

                    for (GEO::index_t lv = 0; lv < fv_nb; ++lv) {
                        const GEO::index_t ie = in.field_as_uint(1+lv);
                        const GEO::index_t e = ie/2;
                        if(e > mesh.edges.nb()) {
                            LOG::ERROR("{}", "Line "+std::to_string(in.line_number())+" :Invalid edge id in facet!");
                            return false;
                        }
                        if (ie%2 == 0)
                            facet_ptr.push_back(mesh.edges.vertex(e, 0));
                        else
                            facet_ptr.push_back(mesh.edges.vertex(e, 1));
                    }
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
            else if (kw == "Polyhedra") { // == Polyhedra =========================================================
                in.get_line();
                in.get_fields();
                if (in.nb_fields() != 1) {
                    LOG::ERROR("{}", "Line "+std::to_string(in.line_number())+" :Invalid polyhedra nb format!");
                    return false;
                }

                const GEO::index_t nb_polyhedra = in.field_as_uint(0);

                std::vector<GEO::index_t> cell_ptr; // nb_vertices, hf0, hf1, ..., hfn, nb_vertices, hf1, ...
                for (GEO::index_t i = 0; i < nb_polyhedra; ++i) {
                    in.get_line();
                    in.get_fields();
                    if(in.nb_fields() == 0) {
                        LOG::ERROR("{}", "Line "+std::to_string(in.line_number())+" :Invalid cell, empty line!");
                        return false;
                    }

                    const GEO::index_t cf_nb = in.field_as_uint(0);
                    if (cf_nb != 4 && cf_nb != 6) {
                        LOG::ERROR("{}", "Line "+std::to_string(in.line_number())+" :Invalid cell vertices nb "+std::to_string(cf_nb)+"!");
                        return false;
                    }

                    if (in.nb_fields() != cf_nb+1) {
                        LOG::ERROR("{}", "Line "+std::to_string(in.line_number())+" :Invalid cell, wrong number of elements!");
                        return false;
                    }
                    cell_ptr.push_back(cf_nb);

                    for (GEO::index_t lf = 0; lf < cf_nb; ++lf) {
                        const GEO::index_t hf = in.field_as_uint(1+lf);
                        if(const GEO::index_t f = hf/2;
                            f > mesh.facets.nb()) {
                            LOG::ERROR("{}", "Line "+std::to_string(in.line_number())+" :Invalid facet id in cell!");
                            return false;
                        }
                        cell_ptr.push_back(hf);
                    }
                }

                /* Create cells */
                constexpr GEO::index_t CELL_TYPE_TET = 0;
                constexpr GEO::index_t CELL_TYPE_HEX = 1;
                for (GEO::index_t begin_ci = 0, ci_end = cell_ptr.size(); begin_ci < ci_end;) {
                    GEO::index_t end_ci = begin_ci;
                    GEO::index_t cells_nb = 0;
                    GEO::index_t cell_type; // CELL_TYPE_TET, CELL_TYPE_HEX
                    {
                        if (const GEO::index_t cf_nb = cell_ptr[begin_ci];
                            cf_nb == 4)
                            cell_type = CELL_TYPE_TET;
                        else {
                            assert(cf_nb == 6);
                            cell_type = CELL_TYPE_HEX;
                        }
                    }

                    while (end_ci < ci_end) {
                        if (const GEO::index_t cf_nb = cell_ptr[end_ci];
                            (cell_type == CELL_TYPE_TET && cf_nb == 4) ||
                            (cell_type == CELL_TYPE_HEX && cf_nb == 6)
                            ) {
                            ++cells_nb;
                            end_ci += cf_nb+1;
                            }
                        else
                            break;
                    }

                    if (cell_type == CELL_TYPE_TET) {
                        LOG::ERROR("{}", "Not support tets yet!");
                        return false;
                    }
                    else if (cell_type == CELL_TYPE_HEX) {
                        GEO::index_t new_c = mesh.cells.create_hexes(cells_nb);
                        for (; begin_ci < end_ci; ++begin_ci) {
                            const GEO::index_t cf_nb = cell_ptr[begin_ci];
                            if (cf_nb == 6) {
                                const auto& hf0 = cell_ptr[begin_ci+1];
                                const auto& hf1 = cell_ptr[begin_ci+2];
                                const auto& hf3 = cell_ptr[begin_ci+4];
                                const auto& hf4 = cell_ptr[begin_ci+5];
                                const auto f0 = hf0/2;
                                const auto f1 = hf1/2;
                                const auto f3 = hf3/2;
                                const auto f4 = hf4/2;
                                assert(f0 < mesh.facets.nb());
                                assert(f1 < mesh.facets.nb());
                                assert(mesh.facets.nb_vertices(f0) == 4); // quad
                                assert(mesh.facets.nb_vertices(f1) == 4); // quad
                                // ref: https://github.com/LIHPC-Computational-Geometry/ovm.io/blob/main/app/ovm.io.cpp
                                // OVM convention is
                                //       5-------6
                                //      /|      /|
                                //     / |     / |
                                //    3-------2  |
                                //    |  4----|--7
                                //    | /     | /
                                //    |/      |/
                                //    0-------1
                                //
                                // and Geogram convention is
                                //        4-------6
                                //       /|      /|
                                //      / |     / |
                                //     0-------2  |
                                //     |  5----|--7
                                //     | /     | /
                                //     |/      |/
                                //     1-------3
                                /* Find v0 (f0, f3, f4) */
                                GEO::index_t v0 = GEO::NO_VERTEX;
                                {
                                    for (GEO::index_t lv0 = 0; lv0 < 4; ++lv0) {
                                        const auto& f0v = mesh.facets.vertex(f0, lv0);
                                        for (GEO::index_t lv3 = 0; lv3 < 4; ++lv3) {
                                            if (const auto& f3v = mesh.facets.vertex(f3, lv3);
                                                f3v != f0v)
                                                continue;
                                            for (GEO::index_t lv4 = 0; lv4 < 4; ++lv4) {
                                                if (const auto& f4v = mesh.facets.vertex(f4, lv4);
                                                    f4v != f0v)
                                                    continue;
                                                v0 = f0v;
                                                break;
                                            }
                                            if (v0 != GEO::NO_VERTEX)
                                                break;
                                        }
                                        if (v0 != GEO::NO_VERTEX)
                                            break;
                                    }
                                    assert(v0 != GEO::NO_VERTEX);
                                }

                                /* Find v4 (f1, f3, f4) */
                                GEO::index_t v4 = GEO::NO_VERTEX;
                                {
                                    for (GEO::index_t lv1 = 0; lv1 < 4; ++lv1) {
                                        const auto& f1v = mesh.facets.vertex(f1, lv1);
                                        for (GEO::index_t lv3 = 0; lv3 < 4; ++lv3) {
                                            if (const auto& f3v = mesh.facets.vertex(f3, lv3);
                                                f3v != f1v)
                                                continue;
                                            for (GEO::index_t lv4 = 0; lv4 < 4; ++lv4) {
                                                if (const auto& f4v = mesh.facets.vertex(f4, lv4);
                                                    f4v != f1v)
                                                    continue;
                                                v4 = f1v;
                                                break;
                                            }
                                            if (v4 != GEO::NO_VERTEX)
                                                break;
                                        }
                                        if (v4 != GEO::NO_VERTEX)
                                            break;
                                    }
                                    assert(v4 != GEO::NO_VERTEX);
                                }

                                for (GEO::index_t lv = 0; lv < 4; ++lv) {
                                    if (mesh.facets.vertex(f0, lv) != v0)
                                        continue;
                                    if (hf0%2 == 0) {
                                        mesh.cells.set_vertex(new_c, 0, mesh.facets.vertex(f0, (lv+3)%4));
                                        mesh.cells.set_vertex(new_c, 1, mesh.facets.vertex(f0, lv));
                                        mesh.cells.set_vertex(new_c, 2, mesh.facets.vertex(f0, (lv+2)%4));
                                        mesh.cells.set_vertex(new_c, 3, mesh.facets.vertex(f0, (lv+1)%4));
                                    }
                                    else { // inverse
                                        mesh.cells.set_vertex(new_c, 0, mesh.facets.vertex(f0, (lv+1)%4));
                                        mesh.cells.set_vertex(new_c, 1, mesh.facets.vertex(f0, lv));
                                        mesh.cells.set_vertex(new_c, 2, mesh.facets.vertex(f0, (lv+2)%4));
                                        mesh.cells.set_vertex(new_c, 3, mesh.facets.vertex(f0, (lv+3)%4));
                                    }
                                }

                                for (GEO::index_t lv = 0; lv < 4; ++lv) {
                                    if (mesh.facets.vertex(f1, lv) != v4)
                                        continue;
                                    if (hf1%2 == 0) {
                                        mesh.cells.set_vertex(new_c, 4, mesh.facets.vertex(f1, (lv+1)%4));
                                        mesh.cells.set_vertex(new_c, 5, mesh.facets.vertex(f1, lv));
                                        mesh.cells.set_vertex(new_c, 6, mesh.facets.vertex(f1, (lv+2)%4));
                                        mesh.cells.set_vertex(new_c, 7, mesh.facets.vertex(f1, (lv+3)%4));
                                    }
                                    else { // inverse
                                        mesh.cells.set_vertex(new_c, 4, mesh.facets.vertex(f1, (lv+3)%4));
                                        mesh.cells.set_vertex(new_c, 5, mesh.facets.vertex(f1, lv));
                                        mesh.cells.set_vertex(new_c, 6, mesh.facets.vertex(f1, (lv+2)%4));
                                        mesh.cells.set_vertex(new_c, 7, mesh.facets.vertex(f1, (lv+1)%4));
                                    }
                                }
                                ++new_c;
                            }
                            begin_ci += cf_nb;
                        }
                        assert(new_c == mesh.cells.nb());
                    }

                    assert(begin_ci == end_ci);
                }

                mesh.cells.connect();
            }
            else if (kw == "Vertex_Property" || kw == "VProp") {
                if (!parse_property(in, mesh.vertices.attributes()))
                    return false;
            }
            else if (kw == "Edge_Property" || kw == "EProp") {
                if (!parse_property(in, mesh.edges.attributes()))
                    return false;
            }
            else if (kw == "HalfEdge_Property") {
                LOG::WARN("Halfedge data structure and related properties `{}` are not supported; skip it.", kw);
                for (const auto& e : mesh.edges) {
                    in.get_line();
                    in.get_line();
                }
            }
            else if (kw == "Face_Property" || kw == "FProp") {
                if (!parse_property(in, mesh.facets.attributes()))
                    return false;
            }
            else if (kw == "HalfFace_Property") {
                LOG::WARN("Halfedge data structure and related properties `{}` are not supported; skip it.", kw);
                for (const auto& f : mesh.facets) {
                    in.get_line();
                    in.get_line();
                }
            }
            else if (kw == "Polyhedron_Property") {
                if (!parse_property(in, mesh.cells.attributes()))
                    return false;
            }
            else {
                LOG::ERROR("{}", "Line "+std::to_string(in.line_number())+" :Unknown kw `"+kw+"`!");
                return false;
            }
        }

        return true;
    }

    bool OVM_IOHandler::parse_property(
        LineInput& in,
        GEO::AttributesManager& attributes_manager
        ) {
        if (in.nb_fields() != 3) {
            LOG::ERROR("{}", "Line "+std::to_string(in.line_number())+" :Invalid number of fields, expected 3!");
            return false;
        }

        const std::string prop_type = in.field(1);
        std::string prop_name = in.field(2);
        strip_quotes(prop_name);

        if (prop_type == "int") {
            GEO::Attribute<int> mesh_attribute(attributes_manager, prop_name);
            for (GEO::index_t i = 0, i_end = mesh_attribute.size(); i < i_end; ++i) {
                in.get_line();
                in.get_fields();
                if(in.nb_fields() != 1) {
                    LOG::ERROR("{}", "Line "+std::to_string(in.line_number())+" :Invalid int, expected 1!");
                    return false;
                }
                mesh_attribute[i] = in.field_as_int(0);
            }
        }
        else if (prop_type == "double") {
            GEO::Attribute<double> mesh_attribute(attributes_manager, prop_name);
            for (GEO::index_t i = 0, i_end = mesh_attribute.size(); i < i_end; ++i) {
                in.get_line();
                in.get_fields();
                if(in.nb_fields() != 1) {
                    LOG::ERROR("{}", "Line "+std::to_string(in.line_number())+" :Invalid double, expected 1!");
                    return false;
                }
                mesh_attribute[i] = in.field_as_double(0);
            }
        }
        else if (prop_type == "vec2d") {
            GEO::Attribute<GEO::vec2> mesh_attribute(attributes_manager, prop_name);
            for (GEO::index_t i = 0, i_end = mesh_attribute.size(); i < i_end; ++i) {
                in.get_line();
                in.get_fields();
                if(in.nb_fields() != 2) {
                    LOG::ERROR("{}", "Line "+std::to_string(in.line_number())+" :Invalid vec2d, expected 2!");
                    return false;
                }
                mesh_attribute[i] = GEO::vec2(in.field_as_double(0), in.field_as_double(1));
            }
        }
        else {
            LOG::ERROR("{}", "Line "+std::to_string(in.line_number())+" :Unknown prop type `"+prop_type+"`!");
            return false;
        }

        return true;
    }
}
