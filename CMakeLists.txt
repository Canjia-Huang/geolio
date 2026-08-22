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

# Location of the Geogram source tree. Defaults to the GEOGRAM_DIR environment
# variable; overridable at configure time with -DGEOGRAM_DIR=...
set(GEOGRAM_DIR "$ENV{GEOGRAM_DIR}" CACHE PATH "Path to the Geogram source directory (defaults to the GEOGRAM_DIR environment variable)")

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
include(Dependencies)
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
