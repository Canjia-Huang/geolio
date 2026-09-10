//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/3.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_OVM_IO_H
#define GEOLIO_OVM_IO_H
#include <geogram/mesh/mesh_io.h>
#include "line_stream.h"

namespace geolio
{
    /**
     * @brief IO handler for ".ovm" files exported by OpenVolumeMesh
     * @see https://www.graphics.rwth-aachen.de/software/openvolumemesh/
     */
    class OVM_IOHandler : public GEO::MeshIOHandler {
    public:
        bool load(
            const std::string& filename,
            GEO::Mesh& mesh,
            const GEO::MeshIOFlags& ioflags) override;

        [[deprecated("This function is not yet implemented.")]]
        bool save(
            const GEO::Mesh& M,
            const std::string& filename,
            const GEO::MeshIOFlags& ioflags
            ) override {
            return false;
        }

    private:
        /**
         * @brief Parses a property block (`*_Property` keyword) and loads it
         *        into the given attributes manager.
         * @details On malformed input, logs an error with LOG::ERROR and
         *          returns false instead of throwing.
         * @return true on success, false if the block could not be parsed.
         */
        static bool parse_property(LineInput& in, GEO::AttributesManager& attributes_manager);
    };
}

#endif //GEOLIO_OVM_IO_H
