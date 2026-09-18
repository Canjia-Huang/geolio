if(COMMAND geolio_begin_file)
    geolio_begin_file()
endif()
set(CURRENT_LIB_NAME ${PROJECT_NAME}_third_party)
set(CURRENT_LIB_ALIAS_NAME ${PROJECT_NAME}::third_party)

# Resolve dependencies (Geogram, Eigen) before any target_include_directories
# below that expands ${GEOGRAM_INCLUDE_DIR}. find_path() only sets that cache
# variable during this include, so on a fresh configure the imgui include path
# (${GEOGRAM_INCLUDE_DIR}/geogram_gfx/third_party/imgui) would otherwise be
# recorded with an empty prefix and 'imgui.h' would not be found.
include(Dependencies)

add_library(${CURRENT_LIB_NAME} INTERFACE)
add_library(${CURRENT_LIB_ALIAS_NAME} ALIAS ${CURRENT_LIB_NAME})

# Optional header-only LBFGS-Lite optimizer (https://github.com/ZJU-FAST-Lab/LBFGS-Lite).
# Downloaded at configure time via FetchContent when GEOLIO_ENABLE_LBFGS_LITE is ON.
if(GEOLIO_ENABLE_LBFGS_LITE)
    include(LBFGS-Lite)
    # LBFGS-Lite ships a single header named lbfgs.hpp, which is generic enough to
    # collide with headers from other dependencies. Stage it in the build tree under
    # the unique name lbfgs_lite.hpp, so consumers include <lbfgs_lite.hpp> and never
    # depend on FetchContent's internal directory naming (lbfgs_lite-src).
    set(LBFGS_LITE_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/lbfgs_lite/include")
    file(MAKE_DIRECTORY "${LBFGS_LITE_INCLUDE_DIR}")
    file(COPY_FILE
            "${lbfgs_lite_SOURCE_DIR}/include/lbfgs.hpp"
            "${LBFGS_LITE_INCLUDE_DIR}/lbfgs_lite.hpp"
            ONLY_IF_DIFFERENT
    )
    # SYSTEM: treat the staged header as a third-party include dir (suppresses warnings
    # from it). lbfgs.hpp is header-only but uses Eigen, which is already exposed to
    # this target below.
    target_include_directories(${CURRENT_LIB_NAME} SYSTEM INTERFACE
            "${LBFGS_LITE_INCLUDE_DIR}"
    )

    # Feature macro: defined when the LBFGS-Lite optimizer is enabled (see the
    # GEOLIO_ENABLE_LBFGS_LITE option in the top-level CMakeLists.txt). PUBLIC so the
    # macro is also visible to consumers of geolio::common in their own translation
    # units; code may guard LBFGS-Lite-specific includes/implementations with
    #   #if GEOLIO_ENABLE_LBFGS_LITE
    if(GEOLIO_ENABLE_LBFGS_LITE)
        target_compile_definitions(${CURRENT_LIB_NAME} INTERFACE GEOLIO_ENABLE_LBFGS_LITE=1)
    endif()
endif()

# Optional CoMISo mixed-integer constrained solver (https://github.com/libigl/CoMISo).
# Downloaded at configure time via FetchContent when GEOLIO_ENABLE_COMISO is ON.
if(GEOLIO_ENABLE_COMISO)
    include(CoMISo)
    # CoMISo is not header-only: the fetched CMakeLists.txt defines the static
    # library target `CoMISo`, whose PUBLIC include directories already expose both
    # the <CoMISo/...> prefix and its bundled GMM headers, so nothing extra has to
    # be added to the include list below. CoMISo builds with POSITION_INDEPENDENT_CODE,
    # so it links into the SHARED geolio target.
    target_link_libraries(${CURRENT_LIB_NAME} INTERFACE CoMISo)

    # Feature macro: defined when CoMISo is enabled (see the GEOLIO_ENABLE_COMISO
    # option in the top-level CMakeLists.txt). INTERFACE so the macro is also visible
    # to consumers of geolio::third_party in their own translation units; code may
    # guard CoMISo-specific includes/implementations with
    #   #if GEOLIO_ENABLE_COMISO
    target_compile_definitions(${CURRENT_LIB_NAME} INTERFACE GEOLIO_ENABLE_COMISO=1)
endif()

target_include_directories(${CURRENT_LIB_NAME} INTERFACE
        "${CMAKE_CURRENT_SOURCE_DIR}"
        "${CMAKE_CURRENT_SOURCE_DIR}/CLI11/include"
#        "${CMAKE_CURRENT_SOURCE_DIR}/json/include"
#        "${CMAKE_CURRENT_SOURCE_DIR}/libTimekeeper/src"
        "${CMAKE_CURRENT_SOURCE_DIR}/imoguizmo"
        "${CMAKE_CURRENT_SOURCE_DIR}/spdlog/include"
        "${GEOGRAM_INCLUDE_DIR}/geogram_gfx/third_party/imgui"
)

target_link_libraries(${CURRENT_LIB_NAME} INTERFACE
        Geogram::geogram
        Geogram::geogram_gfx
        Eigen3::Eigen
)

if(COMMAND geolio_end_file)
    geolio_end_file()
endif()