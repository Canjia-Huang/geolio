//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/11/15.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_VTK_IO_H
#define GEOLIO_VTK_IO_H
#include <geogram/mesh/mesh_io.h>

namespace geolio
{
    /**
     * @brief IO handler for legacy ".vtk" files written by VTK / ParaView.
     * @details Only the ASCII flavour of the legacy VTK format is supported,
     *          for the two dataset types that carry an explicit connectivity:
     * - `DATASET UNSTRUCTURED_GRID`: the `CELLS` / `CELL_TYPES` blocks are
     *   converted to Geogram elements,
     * - `DATASET POLYDATA`: the `VERTICES` / `LINES` / `POLYGONS` /
     *   `TRIANGLE_STRIPS` blocks are converted to Geogram elements.
     *
     * Polygonal cells (triangles, quads, pixels, polygons, triangle strips)
     * become mesh facets, polyhedral cells (tetrahedra, hexahedra, voxels,
     * wedges, pyramids) become mesh cells and lines become mesh edges. The
     * `POINT_DATA` / `CELL_DATA` arrays are loaded as attributes of the
     * vertices / facets (or cells, or edges) with their VTK name.
     * @note A `CELL_DATA` block holds the arrays of all the cells of the
     *       dataset, while Geogram splits cells into edges, facets and cells.
     *       Its arrays are therefore only attached when all the cells of the
     *       file became elements of a single kind, and are ignored (with a
     *       warning) for a hybrid mesh that mixes e.g. facets and cells.
     * @see https://vtk.org/wp-content/uploads/2015/04/file-formats.pdf
     */
    class VTK_IOHandler : public GEO::MeshIOHandler {
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

#endif //GEOLIO_VTK_IO_H
