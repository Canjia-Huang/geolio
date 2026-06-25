########################################################################################################################
# CMake options
########################################################################################################################
cmake_minimum_required(VERSION 3.30)
project(geolio)

# == Options ==
option(BUILD_TESTS "Build tests" ON)

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    # standalone
else()
    # as submodule
    set(BUILD_TESTS OFF)
endif()

if(WIN32)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /utf-8")
endif()
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

########################################################################################################################
# CMake module path
########################################################################################################################
list(PREPEND CMAKE_MODULE_PATH
        ${CMAKE_CURRENT_LIST_DIR}/cmake)

########################################################################################################################
# Include deps
########################################################################################################################
include(Dependencies)

########################################################################################################################
# Add library
########################################################################################################################
add_subdirectory(src)

add_library(geolio INTERFACE)
target_include_directories(geolio
        INTERFACE
        include/
)
target_link_libraries(geolio
        INTERFACE
        ${PROJECT_NAME}_LIB
)

########################################################################################################################
# Add subdirectories
########################################################################################################################
if (BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
