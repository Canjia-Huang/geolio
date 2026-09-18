if(COMMAND geolio_begin_file)
    geolio_begin_file()
endif()

# libigl (https://github.com/libigl/libigl) is header-only, so there are two ways to
# make its <igl/...> headers available to src/geolio/miq, tried in this order:
#
#  1. An installed libigl CMake package. find_package() in CONFIG mode only looks for
#     a libigl-config.cmake file, which libigl generates and `cmake --install` places
#     in <prefix>/lib/cmake/igl. Being header-only says nothing about that file: there
#     is no compiled libigl to link, but the package config still has to be installed
#     before it exists on disk.
#  2. The libigl source checkout in LIBIGL_DIR (see the top-level CMakeLists.txt).
#     This needs no install at all -- with nothing to link, pointing at
#     <LIBIGL_DIR>/include is sufficient. Verified against libigl 2.5.0.
#
# Either way LIBIGL_TARGET ends up naming a target carrying libigl's include
# directory, so third_party/CMakeLists.txt links it without caring which path won.
#
# Only GEOLIO_ENABLE_MIQ pulls this in, and ending up with neither is a hard error:
# configuring without libigl would otherwise survive until the compiler hit
# "igl/local_basis.h: No such file or directory" in src/geolio/miq/miq.cpp.
# QUIET + explicit messages rather than REQUIRED, so the hints below are what the user
# reads instead of CMake's generic "could not find a package configuration file"
# boilerplate.
find_package(libigl QUIET)

if(libigl_FOUND)
    # Which target the package provides depends on how it was produced:
    #   * An install made straight from libigl's own CMakeLists.txt exports
    #     igl::igl_core. cmake/igl/igl_install.cmake sets EXPORT_NAME from
    #     `module_export`, a variable that is never defined anywhere upstream, so the
    #     property is left unset and the target keeps its in-tree name igl_core
    #     (https://github.com/libigl/libigl/issues/2186 names it that way).
    #   * igl::core is the alias libigl uses inside its own build tree, and what
    #     repackaged installs (vcpkg, conda, ...) expose once they correct the above.
    if(TARGET igl::core)
        set(LIBIGL_TARGET igl::core)
    elseif(TARGET igl::igl_core)
        set(LIBIGL_TARGET igl::igl_core)
    else()
        message(FATAL_ERROR
                "libigl was found at ${libigl_DIR}, but it defines neither igl::core nor\n"
                "igl::igl_core, so GEOLIO_ENABLE_MIQ cannot use it. The package looks\n"
                "incomplete or was built from a non-standard libigl.\n"
                "Reinstall libigl, or point -DLIBIGL_DIR=<libigl-src> at a libigl source\n"
                "tree to use the headers directly, or build without MIQ using\n"
                "-DGEOLIO_ENABLE_MIQ=OFF.")
    endif()

    # libigl installs its version file as LibiglConfigVersion.cmake, which CMake does
    # not associate with the `libigl` package, so libigl_VERSION is normally empty.
    if(libigl_VERSION)
        message(STATUS "libigl ${libigl_VERSION} found in ${libigl_DIR} (target ${LIBIGL_TARGET})")
    else()
        message(STATUS "libigl found in ${libigl_DIR} (target ${LIBIGL_TARGET})")
    endif()
elseif(LIBIGL_DIR)
    # No installed package, but an explicit source tree. Both the tree root and its
    # include/ directory are accepted, so -DLIBIGL_DIR=<libigl-src> and
    # -DLIBIGL_DIR=<libigl-src>/include both work.
    # NO_DEFAULT_PATH on purpose: this fallback is about the tree the user named, so
    # an unrelated /usr/local/include/igl must not silently satisfy it.
    find_path(LIBIGL_INCLUDE_DIR
            NAMES igl/igl_inline.h
            HINTS "${LIBIGL_DIR}/include" "${LIBIGL_DIR}"
            NO_DEFAULT_PATH
    )

    if(NOT LIBIGL_INCLUDE_DIR)
        message(FATAL_ERROR
                "LIBIGL_DIR is set to \"${LIBIGL_DIR}\", but no libigl headers were found\n"
                "there: expected <LIBIGL_DIR>/include/igl/igl_inline.h (or\n"
                "<LIBIGL_DIR>/igl/igl_inline.h).\n"
                "\n"
                "LIBIGL_DIR must be the libigl *source tree* (the directory containing\n"
                "include/igl), not the build directory of an installed libigl.\n"
                "\n"
                "Either fix LIBIGL_DIR, or install libigl and point libigl_DIR at its\n"
                "package config directory, or build without MIQ:\n"
                "    cmake -S . -B <build> -DGEOLIO_ENABLE_MIQ=OFF")
    endif()

    # Stand-in for libigl's igl::core: header-only, so an INTERFACE target carrying the
    # include directory is the whole of it. Eigen3::Eigen is required because the igl
    # headers include <Eigen/...>; Threads mirrors what libigl's own package config
    # requires (find_dependency(Threads)), so this path declares the same dependencies
    # as the installed-package path. cxx_std_17 matches igl_add_library()'s
    # target_compile_features.
    if(NOT TARGET geolio_libigl)
        add_library(geolio_libigl INTERFACE)
        target_include_directories(geolio_libigl SYSTEM INTERFACE "${LIBIGL_INCLUDE_DIR}")
        find_package(Threads REQUIRED)
        target_link_libraries(geolio_libigl INTERFACE Eigen3::Eigen Threads::Threads)
        target_compile_features(geolio_libigl INTERFACE cxx_std_17)
    endif()
    set(LIBIGL_TARGET geolio_libigl)

    message(STATUS "libigl package not found; using header-only source tree in ${LIBIGL_INCLUDE_DIR}")
else()
    message(FATAL_ERROR
            "GEOLIO_ENABLE_MIQ is ON, but libigl could not be found.\n"
            "\n"
            "libigl is header-only, so no install is strictly required -- but it has to be\n"
            "reachable in one of these two ways.\n"
            "\n"
            "1. Use a libigl source tree directly (no install needed):\n"
            "       cmake -S . -B <build> -DLIBIGL_DIR=<libigl-src>\n"
            "   or export LIBIGL_DIR=<libigl-src> and re-run configure.\n"
            "\n"
            "2. Or install libigl first, then point CMake at its package config:\n"
            "       cmake -S <libigl-src> -B <libigl-build> -DCMAKE_BUILD_TYPE=Release\n"
            "       cmake --install <libigl-build> --prefix <libigl-prefix>\n"
            "       cmake -S . -B <build> -Dlibigl_DIR=<libigl-prefix>/lib/cmake/igl\n"
            "   Find_package only sees the installed package, so the config file must\n"
            "   exist: <libigl-prefix>/lib/cmake/igl/libigl-config.cmake.\n"
            "   Note: -DCMAKE_PREFIX_PATH=<libigl-prefix> alone will NOT work -- the\n"
            "   package is named `libigl` while its config directory is named `igl`.\n"
            "\n"
            "Or build without MIQ:\n"
            "    cmake -S . -B <build> -DGEOLIO_ENABLE_MIQ=OFF")
endif()

if(COMMAND geolio_end_file)
    geolio_end_file()
endif()
