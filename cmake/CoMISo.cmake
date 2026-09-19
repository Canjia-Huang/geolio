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
        GIT_TAG        175e9c3222d6d2717fb112478787d36d68c19766
        SOURCE_DIR     "${FETCHCONTENT_BASE_DIR}/CoMISo"
)
FetchContent_MakeAvailable(comiso)
message(STATUS "CoMISo fetched at ${comiso_SOURCE_DIR}")

# The revision pinned above carries the three portability fixes the vendored gmm library
# (ext/gmm-4.2) and CoMISo's build need to work on this project's toolchains. All three
# are no-ops semantically, and all three are hard failures on the Windows CI runners
# (MSVC 14.51 in Visual Studio 18), which is why they live in the fork rather than in a
# warning flag here:
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
#  3. The fork's Windows build no longer links the OpenBLAS copy vendored under
#     ext/OpenBLAS-v0.2.14-Win64-int64. That library is an INTERFACE64 (-i8) build whose
#     Fortran INTEGER is 64-bit (readme.txt: INTERFACE64=1; openblas_config.h:
#     OPENBLAS_USE64BITINT, typedef BLASLONG blasint), while gmm's BLAS/LAPACK interface
#     declares and calls every routine with 32-bit ints -- 27 call sites passing the
#     addresses of local ints. Each call therefore made the BLAS routine read four bytes
#     past its argument, so lengths and increments became whatever the neighbouring stack
#     slot held: undefined behaviour that surfaces or not depending on the optimization
#     level, which is how it passed on Debug and SIGSEGV'd on Release, on the very first
#     gmm::vect_sp -> ddot_ of the MIQ pipeline. The fork now compiles that interface out
#     on Windows (COMISO_NO_BLAS, PUBLIC so the miq sources that include GMM_Tools.cc see
#     the same setting) and gmm uses its own implementations there, while macOS and Linux
#     keep their BLAS (Accelerate, find_package(BLAS)), both of them 32-bit-integer
#     builds. Nothing imports libopenblas.dll any more either, so no BLAS runtime DLLs
#     have to be shipped next to the executables -- re-enabling BLAS on Windows means
#     pointing at a 32-bit-integer build and copying its DLLs, see Geogram.cmake for how
#     geogram's own DLL set is staged into build/bin/<config>.

if(COMMAND geolio_end_file)
    geolio_end_file()
endif()
