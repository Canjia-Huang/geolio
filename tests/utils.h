//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/3/12.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_TEST_UTILS_H
#define GEOLIO_TEST_UTILS_H

#include <gtest/gtest.h>
#include <string>
#include <vector>
#include <geogram/delaunay/CDT_2d.h>
#include <../src/geolio/geogram_plus/CDT_2d.h>
#include <../src/geolio/geogram_plus/periodic_delaunay_3d.h>
#include <geogram/delaunay/periodic_delaunay_3d.h>

namespace geolio::test
{
    /**
     * @brief Generate a random 2D constrained Delaunay triangulation (CDT) and append it to a mesh.
     *
     * @param mesh [out] GEO::Mesh that will receive the generated triangulation. The function connects mesh facets before returning.
     * @param samples_nb [in] Number of random sample points to insert (default: 20).
     * @param w [in] Half-width of the square domain used to place the enclosing quad and sample points (default: 10).
     *
     * Implementation:
     * Creates a CDT, builds an enclosing quadrilateral from (-w,-w) to (w,w), inserts `samples_nb` random points
     * inside an expanded area, appends the CDT geometry to `mesh`, and connects the mesh facets.
     */
    inline void generate_random_CDT2d_mesh(
        GEO::Mesh& mesh,
        const GEO::index_t samples_nb = 20,
        const double w = 10
        ) {
        GEO::CDT2d CDT;
        CDT.create_enclosing_quad(GEO::vec2(-w, -w), GEO::vec2(w, -w), GEO::vec2(w, w), GEO::vec2(-w, w));
        for (GEO::index_t i = 0; i < samples_nb; ++i)
            CDT.insert(GEO::vec2(-0.8*w+1.6*w*GEO::Numeric::random_float32(), -0.8*w+1.6*w*GEO::Numeric::random_float32()));

        append_CDT2d_to_mesh(CDT, mesh);
        mesh.facets.connect();
    }

    /**
     * @brief Generate a random 3D periodic Delaunay triangulation and append it to a mesh.
     *
     * @param mesh [out] GEO::Mesh that will receive the generated tetrahedral mesh. The function connects mesh cells before returning.
     * @param samples_nb [in] Number of random vertices to generate (default: 20).
     * @param w [in] Scale parameter used to spread generated vertex coordinates (default: 10).
     *
     * Implementation:
     * Fills a flat vector with `samples_nb` random 3D coordinates, constructs a PeriodicDelaunay3d,
     * sets the vertices, computes the Delaunay triangulation, appends the result to `mesh`, and connects mesh cells.
     */
    inline void generate_random_delaunay3d_mesh (
        GEO::Mesh& mesh,
        const GEO::index_t samples_nb = 20,
        const double w = 10
        ) {
        std::vector<double> points;
        points.reserve(samples_nb*3);
        for (GEO::index_t i = 0, i_end = samples_nb*3; i < i_end; ++i)
            points.push_back(-w*1.8*w*GEO::Numeric::random_float32());

        GEO::SmartPointer<GEO::PeriodicDelaunay3d> delaunay = new GEO::PeriodicDelaunay3d(false);
        delaunay->set_vertices(samples_nb, points.data());
        delaunay->compute();

        append_PeriodicDelaunay3d_to_mesh(*delaunay, mesh);
        mesh.cells.connect();
    }

    /**
     * @brief Construct a printable name for the currently running Google Test.
     *
     * @return std::string A string in the form "test_<test_case_name>_<test_name>".
     *
     * Implementation:
     * Queries testing::UnitTest::GetInstance()->current_test_info() and concatenates the test case name
     * and the test name with separators to produce a reproducible identifier useful for filenames or logs.
     */
    inline std::string get_current_test_name(){
        const testing::TestInfo* const current_test_info = testing::UnitTest::GetInstance()->current_test_info();
        return std::string("test")
                + "_"
                + std::string(current_test_info->test_case_name())
                + "_"
                + std::string(current_test_info->name());
    }

    template<GEO::index_t DIM>
    struct DimWrapper {
        static constexpr GEO::index_t value = DIM;
    };

    using Dim2 = std::integral_constant<GEO::index_t, 2>;
    using Dim3 = std::integral_constant<GEO::index_t, 3>;
    using DimTypes = ::testing::Types<Dim2, Dim3>;
}


#endif //GEOLIO_TEST_UTILS_H