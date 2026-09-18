//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/18.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
// Helpers shared by the high-order control-grid tests (quad, hexahedral and the
// Jacobian-determinant analyzer): tolerances, parametric sample generators, control-node
// offsets, artifact naming and the invariants of the quality measures.
//
#ifndef GEOLIO_TEST_CONTROL_GRID_TEST_UTILS_H
#define GEOLIO_TEST_CONTROL_GRID_TEST_UTILS_H

#include <geogram/basic/geometry.h>
#include <geogram/basic/numeric.h>
#include <geogram/mesh/mesh.h>
#include <gtest/gtest.h>
#include "../utils.h"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace geolio::test
{
    /* == tolerances =========================================================================================== */

    /** Tolerance on squared distances for algebra that is exact up to rounding. */
    inline constexpr double EXACT_DIST2_TOL = 1e-20;

    /** Tolerance on quantities compared against an independently recomputed reference. */
    inline constexpr double REFERENCE_TOL = 1e-12;

    /** Tolerance on the quality-measure invariants, whose bounds are sharp. */
    inline constexpr double QUALITY_TOL = 1e-12;

    /* == parametric samples =================================================================================== */

    /** Default number of random samples drawn by the parametric property tests. */
    inline constexpr GEO::index_t RANDOM_SAMPLE_NB = 10000;

    /**
     * @brief Generate random 1D parameters.
     * @param[in] sample_nb Number of samples.
     * @return @p sample_nb values uniformly distributed in [0, 1].
     */
    [[nodiscard]] inline std::vector<double> random_unit_samples_1d(const GEO::index_t sample_nb) {
        std::vector<double> samples;
        samples.reserve(sample_nb);
        for (GEO::index_t i = 0; i < sample_nb; ++i)
            samples.push_back(GEO::Numeric::random_float32());
        return samples;
    }

    /**
     * @brief Generate random 2D parametric samples.
     * @param[in] sample_nb Number of samples.
     * @return @p sample_nb points uniformly distributed in [0,1]^2.
     */
    [[nodiscard]] inline std::vector<GEO::vec2> random_unit_samples_2d(const GEO::index_t sample_nb) {
        std::vector<GEO::vec2> samples;
        samples.reserve(sample_nb);
        for (GEO::index_t i = 0; i < sample_nb; ++i)
            samples.emplace_back(GEO::Numeric::random_float32(), GEO::Numeric::random_float32());
        return samples;
    }

    /**
     * @brief Generate random 3D parametric samples.
     * @param[in] sample_nb Number of samples.
     * @return @p sample_nb points uniformly distributed in [0,1]^3.
     */
    [[nodiscard]] inline std::vector<GEO::vec3> random_unit_samples_3d(const GEO::index_t sample_nb) {
        std::vector<GEO::vec3> samples;
        samples.reserve(sample_nb);
        for (GEO::index_t i = 0; i < sample_nb; ++i)
            samples.emplace_back(
                GEO::Numeric::random_float32(),
                GEO::Numeric::random_float32(),
                GEO::Numeric::random_float32());
        return samples;
    }

    /**
     * @brief Generate the regular samples of the unit square.
     * @param[in] resolution Number of subdivisions of each parametric direction.
     * @return The (resolution+1)^2 samples, row by row.
     */
    [[nodiscard]] inline std::vector<GEO::vec2> grid_unit_samples_2d(const GEO::index_t resolution) {
        std::vector<GEO::vec2> samples;
        samples.reserve((resolution+1)*(resolution+1));
        for (GEO::index_t i = 0; i <= resolution; ++i)
            for (GEO::index_t j = 0; j <= resolution; ++j)
                samples.emplace_back(static_cast<double>(i)/resolution, static_cast<double>(j)/resolution);
        return samples;
    }

    /**
     * @brief Generate the regular samples of the unit cube.
     * @param[in] resolution Number of subdivisions of each parametric direction.
     * @return The (resolution+1)^3 samples.
     */
    [[nodiscard]] inline std::vector<GEO::vec3> grid_unit_samples_3d(const GEO::index_t resolution) {
        std::vector<GEO::vec3> samples;
        samples.reserve((resolution+1)*(resolution+1)*(resolution+1));
        for (GEO::index_t i = 0; i <= resolution; ++i)
            for (GEO::index_t j = 0; j <= resolution; ++j)
                for (GEO::index_t k = 0; k <= resolution; ++k)
                    samples.emplace_back(
                        static_cast<double>(i)/resolution,
                        static_cast<double>(j)/resolution,
                        static_cast<double>(k)/resolution);
        return samples;
    }

    /* == control-node offsets ================================================================================= */

    /**
     * @brief Build an offset along one coordinate axis.
     * @param[in] d Coordinate index, 0,1,(2).
     * @param[in] value Signed displacement along \p d.
     * @return The offset vector, all other components being zero.
     */
    template <GEO::index_t DIM>
    [[nodiscard]] inline GEO::vecng<DIM, double> axis_offset(const GEO::index_t d, const double value) {
        GEO::vecng<DIM, double> offset;
        offset[d] = value;
        return offset;
    }

    /**
     * @brief Build a random offset.
     * @param[in] amplitude Upper bound of the displacement of each coordinate.
     * @return A vector whose components are uniformly distributed in [0, amplitude].
     */
    template <GEO::index_t DIM>
    [[nodiscard]] inline GEO::vecng<DIM, double> random_offset(const double amplitude) {
        GEO::vecng<DIM, double> offset;
        for (GEO::index_t d = 0; d < DIM; ++d)
            offset[d] = amplitude*GEO::Numeric::random_float32();
        return offset;
    }

    /* == artifacts ============================================================================================ */

    /**
     * @brief Path of an artifact file named after the running test.
     * @param[in] suffix Suffix appended to the test name, including the file extension.
     * @return The file name, "/" being replaced by "_" since typed tests report names such as
     *         "Suite/0".
     */
    [[nodiscard]] inline std::string artifact_path(const std::string_view suffix) {
        std::string path = get_current_test_name();
        std::ranges::replace(path, '/', '_');
        path += suffix;
        return path;
    }

    /* == diagnostics ========================================================================================== */

    /**
     * @brief Describe a parametric point for test failure messages.
     * @param[in] name Name of the parametric coordinates, e.g. "uv".
     * @param[in] point Parametric point.
     * @return A string of the form "uv = (0.5, 0.25)".
     */
    [[nodiscard]] inline std::string parametric_context(const char* name, const GEO::vec2& point) {
        std::ostringstream out;
        out << name << " = (" << point.x << ", " << point.y << ")";
        return out.str();
    }

    /** @copydoc parametric_context(const char*, const GEO::vec2&) */
    [[nodiscard]] inline std::string parametric_context(const char* name, const GEO::vec3& point) {
        std::ostringstream out;
        out << name << " = (" << point.x << ", " << point.y << ", " << point.z << ")";
        return out.str();
    }

    /* == quality measures ===================================================================================== */

    /**
     * Quality measures of a control-grid element evaluated at one parametric point.
     *
     * The hexahedral grid does not define an absolute-area measure, so such grids leave
     * @ref absolute_sq_area at zero.
     */
    struct Quality {
        double det_jacobian = 0.0;
        double absolute_sq_area = 0.0;
        double scaled_jacobian = 0.0;
        double inverse_mean_ratio = 0.0;
        double MIPS = 0.0;
    };

    /**
     * @brief Check the bounds that the quality measures must satisfy.
     *
     * - the absolute squared area is non-negative (and zero when the grid does not define it),
     * - the scaled Jacobian is in [-1, 1] (Cauchy-Schwarz),
     * - the inverse mean ratio is in [0, 1], 1 being the ideal element,
     * - MIPS is bounded from below by 1, its ideal value.
     *
     * @param[in] quality Measures evaluated at one parametric point.
     * @param[in] context Parametric point the measures were evaluated at, for diagnostics.
     */
    inline void expect_quality_invariants(const Quality& quality, const std::string& context = {}) {
        SCOPED_TRACE(context);

        EXPECT_FALSE(std::isnan(quality.det_jacobian));
        EXPECT_GE(quality.absolute_sq_area, 0.0);
        EXPECT_GE(quality.scaled_jacobian, -1.0 - QUALITY_TOL);
        EXPECT_LE(quality.scaled_jacobian, 1.0 + QUALITY_TOL);
        EXPECT_GE(quality.inverse_mean_ratio, -QUALITY_TOL);
        EXPECT_LE(quality.inverse_mean_ratio, 1.0 + QUALITY_TOL);
        EXPECT_GE(quality.MIPS, 1.0 - QUALITY_TOL);
    }

    /** The quality measures of every vertex of a discretized mesh, stored as attributes. */
    struct MeshQualityAttributes {
        GEO::Attribute<double> det_jacobian;
        GEO::Attribute<double> absolute_sq_area;
        GEO::Attribute<double> scaled_jacobian;
        GEO::Attribute<double> inverse_mean_ratio;
        GEO::Attribute<double> MIPS;

        /**
         * @brief Create the quality attributes of a discretized mesh.
         * @param[in] mesh Mesh whose vertices receive the attributes.
         * @param[in] with_absolute_area Also store the absolute squared area; grids that do not
         *            define it (the hexahedral one) leave the attribute unbound.
         */
        explicit MeshQualityAttributes(const GEO::Mesh& mesh, const bool with_absolute_area = true)
            : det_jacobian(mesh.vertices.attributes(), "det_jacobian"),
              scaled_jacobian(mesh.vertices.attributes(), "scaled_jacobian"),
              inverse_mean_ratio(mesh.vertices.attributes(), "inverse_mean_ratio"),
              MIPS(mesh.vertices.attributes(), "MIPS")
        {
            if (with_absolute_area)
                absolute_sq_area.bind(mesh.vertices.attributes(), "absolute_area");
        }

        /**
         * @brief Store the measures of one vertex.
         * @param[in] v Vertex index.
         * @param[in] quality Measures evaluated at that vertex.
         */
        void set(const GEO::index_t v, const Quality& quality) {
            det_jacobian[v] = quality.det_jacobian;
            if (absolute_sq_area.is_bound())
                absolute_sq_area[v] = quality.absolute_sq_area;
            scaled_jacobian[v] = quality.scaled_jacobian;
            inverse_mean_ratio[v] = quality.inverse_mean_ratio;
            MIPS[v] = quality.MIPS;
        }
    };
}

#endif //GEOLIO_TEST_CONTROL_GRID_TEST_UTILS_H
