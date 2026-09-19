if(COMMAND geolio_begin_file)
    geolio_begin_file()
endif()

if(NOT TARGET Geogram::geogram)
    include(Geogram)
endif()
if (NOT TARGET Eigen3::Eigen)
    include(Eigen)
endif()
# After Eigen: libigl's own package config calls find_dependency(Eigen3 REQUIRED),
# and only MIQ needs libigl at all. Keyed off LIBIGL_TARGET rather than a target
# name, because which igl target an install exports varies (see cmake/libigl.cmake).
if(GEOLIO_ENABLE_MIQ AND NOT DEFINED LIBIGL_TARGET)
    include(libigl)
endif()

if(COMMAND geolio_end_file)
    geolio_end_file()
endif()