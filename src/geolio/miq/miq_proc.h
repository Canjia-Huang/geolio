#pragma once

#include <string>

namespace NeurFrame::proc_utils {
/**
 * @brief Run Mixed-Integer Quadrangulation (MIQ) on a surface mesh.
 *
 * Given a surface mesh and an optional cross field, computes a seamless
 * parametrisation and outputs the resulting quad mesh.
 *
 * @param[in] input_surface_mesh_path  Path to the input surface mesh.
 * @param[in] input_cross_path         Path to the cross field file (optional, may be empty).
 * @param[in] output_path              Directory for output files.
 * @param[in] scale                    Global scaling factor for the parametrisation.
 */
void run_miq(
    const std::string& input_surface_mesh_path,
    const std::string& input_cross_path,
    const std::string& output_path,
    double scale
);
}
