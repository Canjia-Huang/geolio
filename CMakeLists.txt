########################################################################################################################
# CMake options
########################################################################################################################
cmake_minimum_required(VERSION 3.30)
project(Geolio
        VERSION 0.1.0
        DESCRIPTION "Geometry library"
        HOMEPAGE_URL "https://github.com/Canjia-Huang/geolio"
        LANGUAGES CXX
)

# == Options ==
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    # standalone: tests on by default
    option(BUILD_TESTS "Build tests" ON)
else()
    # as submodule: tests off by default, but a parent project may opt in
    option(BUILD_TESTS "Build tests" OFF)
endif()

# Use the header-only LBFGS-Lite optimizer (https://github.com/ZJU-FAST-Lab/LBFGS-Lite)
# instead of / in addition to the bundled backends. When ON, LBFGS-Lite is downloaded
# at configure time via CMake FetchContent, its include directory is exposed through
# geolio::third_party, and the GEOLIO_ENABLE_LBFGS_LITE macro is defined for code
# that consumes it.
option(GEOLIO_ENABLE_LBFGS_LITE "Build with the LBFGS-Lite header-only L-BFGS optimizer" OFF)

# Use the CoMISo mixed-integer constrained solver (https://github.com/libigl/CoMISo)
# as the backend of the MIQ pipeline. When ON, CoMISo is downloaded at configure
# time via CMake FetchContent, built as a static library, linked into
# geolio::third_party, and the GEOLIO_ENABLE_COMISO macro is defined for code
# that consumes it.
option(GEOLIO_ENABLE_COMISO "Build with the CoMISo mixed-integer constrained solver" OFF)

# Build the Mixed-Integer Quadrangulation (MIQ) pipeline in src/geolio/miq. MIQ
# needs both the CoMISo mixed-integer solver and libigl, so turning this ON forces
# GEOLIO_ENABLE_COMISO ON below and makes libigl a hard requirement: an installed
# libigl package is preferred, otherwise the header-only source tree in LIBIGL_DIR is
# used instead (see cmake/libigl.cmake). Neither present is a configure error.
option(GEOLIO_ENABLE_MIQ "Build the Mixed-Integer Quadrangulation (MIQ) pipeline" OFF)

if(GEOLIO_ENABLE_MIQ)
    # MIQ cannot be built without CoMISo, so it is not a free choice here. FORCE in
    # the cache (rather than a plain set()) so the effective value is the one every
    # subdirectory sees and the one CMakeCache.txt reports, even when the user passed
    # -DGEOLIO_ENABLE_COMISO=OFF on the configure line.
    set(GEOLIO_ENABLE_COMISO ON CACHE BOOL "Build with the CoMISo mixed-integer constrained solver" FORCE)
endif()

# Location of the Geogram source tree. Defaults to the GEOGRAM_DIR environment
# variable; overridable at configure time with -DGEOGRAM_DIR=...
set(GEOGRAM_DIR "$ENV{GEOGRAM_DIR}" CACHE PATH "Path to the Geogram source directory (defaults to the GEOGRAM_DIR environment variable)")

# Location of the libigl source tree, used only as a fallback for MIQ: libigl is
# header-only, so when no installed libigl package is found this include directory is
# used directly and no install is needed. Defaults to the LIBIGL_DIR environment
# variable; overridable at configure time with -DLIBIGL_DIR=... Distinguished from the
# lowercase `libigl_DIR` that find_package itself sets for an installed package.
set(LIBIGL_DIR "$ENV{LIBIGL_DIR}" CACHE PATH "Path to the libigl source directory (defaults to the LIBIGL_DIR environment variable)")

if(WIN32)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /utf-8")
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

########################################################################################################################
# Output directories: executables -> build/bin, libraries -> build/lib
########################################################################################################################
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

########################################################################################################################
# CMake module path
########################################################################################################################
list(PREPEND CMAKE_MODULE_PATH
        ${CMAKE_CURRENT_LIST_DIR}/cmake)
include(LoggingHelper)
include(MyAddLibrary)

########################################################################################################################
# Include
########################################################################################################################
add_subdirectory(third_party)
add_subdirectory(src/geolio)

########################################################################################################################
# Add subdirectories
########################################################################################################################
if (BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

add_subdirectory(app)

########################################################################################################################
# Install
########################################################################################################################
#include(Config)
