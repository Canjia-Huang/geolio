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
        GIT_TAG        73989173e8208f0f9b1b47d107b4f741dc786f95
        SOURCE_DIR     "${FETCHCONTENT_BASE_DIR}/CoMISo"
)
FetchContent_MakeAvailable(comiso)
message(STATUS "CoMISo fetched at ${comiso_SOURCE_DIR}")

# The revision pinned above carries the two portability fixes the vendored gmm library
# (ext/gmm-4.2) needs to compile under C++17 and later. Both are no-ops semantically,
# but both are hard errors on the toolchain used by the Windows CI runners (MSVC 14.51
# in Visual Studio 18), which is why they live in the fork rather than in a warning flag
# here:
#
#  1. gmm_domain_decomp.h declared a variable with the `register` storage class, which
#     C++17 deleted from the language. Clang rejects it by default ("ISO C++17 does not
#     allow 'register' storage class specifier [-Wregister]"), GCC warns, and MSVC 14.51
#     fails with "C2760: syntax error: 'register' was unexpected here". Earlier revisions
#     therefore needed -Wno-register here and a matching suppression on the miq sources
#     (see src/geolio/CMakeLists.txt); neither suppression ever helped MSVC.
#  2. gmm_real_part.h used std::divides<T> without including <functional>, the header that
#     declares it -- upstream gmm in getfem carries that include today, the vendored 4.2
#     copy predates it. It went unnoticed while <complex> dragged <functional> in, but MSVC
#     14.51's STL puts plus, minus and multiplies in <xutility> (which <complex> does
#     include) and keeps divides and modulus in <functional> (which it does not), so of the
#     four arithmetic functors that one class uses, only std::divides is undeclared there.
#     That yields "C3878: syntax error: unexpected token '>'" at the '>' of std::divides<T>
#     in any translation unit that instantiates gmm's complex ref_elt_vector partial
#     specialization before something else provides <functional> -- which is exactly
#     ConstrainedSolver.cc, whose GMM_Tools.hh pulls in <gmm/gmm.h> ahead of Eigen (Eigen is
#     the one header in the chain that does include <functional>). The neighbouring
#     std::plus/std::minus/std::multiplies uses staying clean is what identifies a missing
#     include rather than a language change.
#
# CoMISo's Windows build also links the OpenBLAS import library vendored next to it
# (ext/OpenBLAS-v0.2.14-Win64-int64/lib/libopenblas.dll.a.lib), because Solver/GMM_Tools.cc
# defines GMM_USES_LAPACK before including <gmm/gmm_lapack_interface.h>: that turns gmm's
# vector products into Fortran BLAS calls on every platform (the built archive references
# ddot_ and daxpy_). An import library turns those references into a load-time dependency on
# libopenblas.dll -- and that DLL is a MinGW build, so it in turn needs libgcc_s_seh-1.dll,
# libgfortran-3.dll and libquadmath-0.dll. Windows resolves imports from the executable's own
# directory first, and gtest_discover_tests() runs the test binary at build time with no PATH
# augmentation, so this whole DLL set has to sit next to every executable that loads
# Geolio.dll. Geogram.cmake copies geogram's DLLs into build/bin/<config> the same way;
# COMISO_BLAS_DLL_DIR is exported as a cache variable so tests/ can copy them next to the
# test binary as well.
if(WIN32 AND TARGET CoMISo)
    set(COMISO_BLAS_DLL_DIR
            "${comiso_SOURCE_DIR}/ext/OpenBLAS-v0.2.14-Win64-int64/bin"
            CACHE INTERNAL "Directory with the BLAS DLLs CoMISo's import library needs at runtime")
    file(GLOB COMISO_BLAS_DLLS "${COMISO_BLAS_DLL_DIR}/*.dll")
    if(COMISO_BLAS_DLLS)
        foreach(config Release Debug RelWithDebInfo MinSizeRel)
            message(STATUS "Copying CoMISo BLAS dlls -> ${CMAKE_BINARY_DIR}/bin/${config}")
            file(COPY ${COMISO_BLAS_DLLS} DESTINATION "${CMAKE_BINARY_DIR}/bin/${config}")
        endforeach()
    else()
        message(WARNING
                "No BLAS dll found in ${COMISO_BLAS_DLL_DIR}: executables importing "
                "libopenblas.dll will fail to start")
    endif()
endif()

if(COMMAND geolio_end_file)
    geolio_end_file()
endif()
