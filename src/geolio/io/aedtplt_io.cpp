//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/26.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "aedtplt_io.h"
#include <geogram/mesh/mesh_io.h>
#include <geogram/basic/line_stream.h>
#include "geolio/common/log.h"
#include "line_stream.h"

namespace geolio
{
    bool AEDTPLT_IOHandler::load(
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

        while (!in.eof()) {
            in.get_line();
            in.get_fields();

            if (in.nb_fields() == 0)
                continue;

            if (const std::string begin_kw = in.field(0);
                begin_kw == "$begin"
                ) {
                if (in.nb_fields() != 2) {
                    LOG::ERROR("Line {} :Expect drawing name!", in.line_number());
                    return false;
                }
                const std::string drawing_name = in.field(1);

                GEO::index_t nodes_nb = 0, elements_nb = 0;
                std::vector<double> vertices; // x, y, z
                std::vector<GEO::index_t> triangles; // 3*cells_nb
                std::vector<GEO::index_t> tetrahedra; // 4*cells_nb
                while (!in.eof()) {
                    in.get_line();

                    if (in.current_line()[0] == '$') { // end
                        in.get_fields();
                        if (in.nb_fields() != 2) {
                            LOG::ERROR("Line {} :Expect `$end drawing_name`!", in.line_number());
                            return false;
                        }
                        if (const std::string kw = in.field(0);
                            kw != "$end") {
                            LOG::ERROR("Line {} :Expect `$end`!", in.line_number());
                            return false;
                        }
                        if (in.field(1) != drawing_name) {
                            LOG::ERROR("Line {} The names of begin and end are different!!", in.line_number());
                            return false;
                        }
                        break;
                    }

                    if (const std::string kw = in.get_bracket_keyword("(");
                        kw == "Elements"
                        ) {
                        in.get_fields(",");

                        nodes_nb = in.field_as_uint(0);
                        elements_nb = in.field_as_uint(1);

                        GEO::index_t pos = 2;
                        for (GEO::index_t i = 0; i < elements_nb; ++i) {
                            pos += 4;

                            const GEO::index_t nb = in.field_as_uint(pos++);
                            if (nb == 6) { // 2-order triangle
                                /* Convert to linear triangle */
                                triangles.push_back(in.field_as_uint(pos)-1);
                                triangles.push_back(in.field_as_uint(pos+2)-1);
                                triangles.push_back(in.field_as_uint(pos+5)-1);
                            }
                            else if (nb == 10) { // 2-order tetrahedron
                                /* Convert to linear tetrahedron */
                                tetrahedra.push_back(in.field_as_uint(pos)-1);
                                tetrahedra.push_back(in.field_as_uint(pos+2)-1);
                                tetrahedra.push_back(in.field_as_uint(pos+5)-1);
                                tetrahedra.push_back(in.field_as_uint(pos+9)-1);
                            }
                            else {
                                LOG::ERROR("Line {} :Unknown element type!", in.line_number());
                                return false;
                            }

                            pos += nb;
                        }
                    }
                    else if (kw == "Nodes") {
                        in.get_fields(",");

                        vertices.reserve(in.nb_fields());
                        for (GEO::index_t i = 0, i_end = in.nb_fields(); i < i_end; ++i)
                            vertices.push_back(in.field_as_double(i));

                        if (vertices.size() != 3*nodes_nb) {
                            LOG::ERROR("Line {} Error nodes nb!", in.line_number());
                            return false;
                        }
                    }
                }

                /* Build mesh elements */
                const GEO::index_t new_v = mesh.vertices.nb();
                if (!vertices.empty()) {
                    const GEO::index_t vertices_nb = vertices.size()/3;
                    mesh.vertices.create_vertices(vertices_nb);
                    for (GEO::index_t v = 0; v < vertices_nb; ++v) {
                        mesh.vertices.point(new_v+v).x = vertices[3*v];
                        mesh.vertices.point(new_v+v).y = vertices[3*v+1];
                        mesh.vertices.point(new_v+v).z = vertices[3*v+2];
                    }
                }
                if (!triangles.empty()) {
                    const GEO::index_t triangles_nb = triangles.size()/3;
                    GEO::index_t new_f = mesh.facets.create_triangles(triangles_nb);
                    for (GEO::index_t f = 0; f < triangles_nb; ++f) {
                        for (GEO::index_t lv = 0; lv < 3; ++lv)
                            mesh.facets.set_vertex(new_f, lv, new_v+triangles[3*f+lv]);
                        ++new_f;
                    }
                }
                if (!tetrahedra.empty()) {
                    const GEO::index_t tetrahedra_nb = tetrahedra.size()/4;
                    GEO::index_t new_c = mesh.cells.create_tets(tetrahedra_nb);
                    for (GEO::index_t c = 0; c < tetrahedra_nb; ++c) {
                        for (GEO::index_t lv = 0; lv < 4; ++lv)
                            mesh.cells.set_vertex(new_c, lv, new_v+tetrahedra[4*c+lv]);
                        ++new_c;
                    }
                }
            }
        }

        return true;
    }
}