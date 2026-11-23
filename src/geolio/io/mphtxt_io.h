//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/11/23.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_MPHTXT_IO_H
#define GEOLIO_MPHTXT_IO_H

#include <geogram/mesh/mesh_io.h>

namespace geolio
{
    /**
     * @brief IO handler for ".mphtxt" files exported by COMSOL Multiphysics.
     * @details Only the mesh vertices and the tetrahedra of the file are
     *          loaded: every other element type (vertex, edge, triangle, quad,
     *          prism, hexahedron, pyramid), the geometric entity indices, the
     *          geometric model and the selections stored in the file are
     *          discarded. A second-order tetrahedron ("tet2") is loaded as the
     *          tetrahedron formed by its 4 corner vertices.
     *
     *          The format is documented in the COMSOL Multiphysics Mesh Import
     *          and Export Guide, section "Specification of COMSOL Formats for
     *          Mesh Data". The loader reads the sections through the comments
     *          that COMSOL writes (`# Mesh data`, `# Mesh vertex coordinates`,
     *          `# number of mesh vertices`, `# Type`, `# Elements`), so that
     *          the fields added by the newer versions of the format (such as
     *          the included geometric model) are skipped, and the fields of
     *          the older ones (such as the lowest mesh vertex index) are
     *          still honoured.
     * @see https://www.comsol.com/model/download/1273541/COMSOL_MeshImportExportGuide.pdf
     */
    class MPHTXT_IOHandler : public GEO::MeshIOHandler {
    public:
        bool load(
            const std::string& filename,
            GEO::Mesh& mesh,
            const GEO::MeshIOFlags& ioflags) override;

        [[deprecated("This function is not yet implemented.")]]
        bool save(
            const GEO::Mesh& mesh,
            const std::string& filename,
            const GEO::MeshIOFlags& ioflags
            ) override {
            return false;
        }
    };
}

#endif //GEOLIO_MPHTXT_IO_H
