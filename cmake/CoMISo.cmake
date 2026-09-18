if(COMMAND geolio_begin_file)
    geolio_begin_file()
endif()

include(FetchContent)

# CoMISo (https://github.com/libigl/CoMISo) is not header-only: its own
# CMakeLists.txt builds the static library target `CoMISo`, exposed as a
# minimalist file meant to be consumed as a subdirectory (libigl style).
#
# Two details of that CMakeLists.txt drive the declaration below:
#
#  1. It publishes the *parent* of its source directory as an include directory
#     ($<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>), because consumers
#     include <CoMISo/Solver/...>. The checkout therefore has to be named
#     `CoMISo`; FetchContent's default directory name (`comiso-src`) would leave
#     every <CoMISo/...> include unresolvable, so SOURCE_DIR is set explicitly.
#  2. It also publishes ${CMAKE_CURRENT_SOURCE_DIR}/../eigen and links
#     Eigen3::Eigen. That sibling path does not exist here and is not needed:
#     Eigen is already provided by the Eigen3::Eigen target (see Eigen.cmake).
FetchContent_Declare(
        comiso
        GIT_REPOSITORY https://github.com/libigl/CoMISo.git
        # CoMISo publishes no version tags, so pin the master tip for a
        # reproducible checkout instead of tracking a moving branch.
        GIT_TAG        562efe333edc8e649dc101469614f43378b1eb55
        SOURCE_DIR     "${FETCHCONTENT_BASE_DIR}/CoMISo"
)
FetchContent_MakeAvailable(comiso)
message(STATUS "CoMISo fetched at ${comiso_SOURCE_DIR}")

# The gmm library vendored inside CoMISo (ext/gmm-4.2) still uses the `register`
# storage class, which C++17 removed. Clang diagnoses that as an error by default
# (group -Wregister), not as a warning, so CoMISo cannot be compiled at all under the
# project's CMAKE_CXX_STANDARD 20 (first hit: gmm/gmm_domain_decomp.h). Suppress it
# for this third-party target only, instead of weakening warnings project-wide;
# -Wno-register is used rather than -Wno-error=register because the latter only
# downgrades the diagnostic, leaving a warning behind for every use.
# Note: gmm's headers are also included by first-party translation units, which need
# the same flag -- this only covers CoMISo's own sources. See src/geolio/CMakeLists.txt.
# Clang and GCC both spell the option this way; MSVC has no equivalent and accepts
# `register` anyway, hence the compiler check.
if(TARGET CoMISo AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(CoMISo PRIVATE -Wno-register)
endif()

if(COMMAND geolio_end_file)
    geolio_end_file()
endif()
