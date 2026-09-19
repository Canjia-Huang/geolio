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
# It is consumed through our own fork rather than upstream, so that the changes
# geolio needs in CoMISo can be committed and pushed: a change carried by a commit
# can be rebased onto new upstream revisions and offered back as a pull request,
# which a build-time patch over an upstream checkout cannot.
FetchContent_Declare(
        comiso
        GIT_REPOSITORY https://github.com/Canjia-Huang/CoMISo.git
        # CoMISo publishes no version tags, so pin a commit for a reproducible
        # checkout instead of tracking a moving branch. After pushing to the fork,
        # bump this to the new revision: `git -C <fork checkout> rev-parse HEAD`.
        GIT_TAG        ac7b11e770448f6a0337a534203e8b8a9aea93e4
        SOURCE_DIR     "${FETCHCONTENT_BASE_DIR}/CoMISo"
)
FetchContent_MakeAvailable(comiso)
message(STATUS "CoMISo fetched at ${comiso_SOURCE_DIR}")

# The revision pinned above also drops the vendored gmm library's single use of the
# `register` storage class (ext/gmm-4.2/include/gmm/gmm_domain_decomp.h), which C++17
# deleted from the language. Earlier revisions needed a -Wno-register suppression here
# and a matching one on the miq sources (see src/geolio/CMakeLists.txt), because Clang
# diagnoses it as an error by default ("ISO C++17 does not allow 'register' storage
# class specifier [-Wregister]") and GCC as a warning, so CoMISo could not be compiled
# at all under this project's CMAKE_CXX_STANDARD 20. Neither suppression ever helped
# MSVC, which rejects the keyword outright since 14.51 (Visual Studio 18, the compiler
# on the Windows CI runners) with "C2760: syntax error: 'register' was unexpected
# here". Removing the keyword in the fork is a no-op semantically -- it was only ever a
# hint compilers ignored -- and it is what makes this one revision build on GCC, Clang
# and MSVC alike, so no per-compiler suppression is needed now.

if(COMMAND geolio_end_file)
    geolio_end_file()
endif()
