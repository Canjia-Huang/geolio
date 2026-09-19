if(COMMAND geolio_begin_file)
    geolio_begin_file()
endif()
set(CURRENT_LIB_NAME ${PROJECT_NAME})
set(CURRENT_LIB_ALIAS_NAME ${PROJECT_NAME}::geolio)

add_library(${CURRENT_LIB_NAME}
        SHARED
)
add_library(${CURRENT_LIB_ALIAS_NAME} ALIAS ${CURRENT_LIB_NAME})

target_include_directories(${CURRENT_LIB_NAME}
        PUBLIC
        # Build
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>/../
        # Install
        $<INSTALL_INTERFACE:include>
)

file(GLOB SOURCE_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/common/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/geobox/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/geobox/object/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/geobox/application/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/geogram_plus/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/high_order/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/io/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/local_operation_optimization/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/motorcycle/*.cpp
)

# miq/ is compiled only when GEOLIO_ENABLE_MIQ is ON: it needs CoMISo and an
# installed libigl (see the GEOLIO_ENABLE_MIQ option in the top-level
# CMakeLists.txt), neither of which the default build brings in.
if(GEOLIO_ENABLE_MIQ)
    file(GLOB MIQ_SOURCE_FILES
            ${CMAKE_CURRENT_SOURCE_DIR}/miq/*.cpp
    )
    list(APPEND SOURCE_FILES ${MIQ_SOURCE_FILES})

    # miq/*.cpp include <CoMISo/Solver/GMM_Tools.hh> -> <gmm/gmm.h>, but they no longer
    # need -Wno-register: the vendored gmm's single use of the `register` storage class
    # was dropped in the CoMISo fork revision pinned in cmake/CoMISo.cmake, so gmm parses
    # cleanly on Clang, GCC and MSVC under the project's CMAKE_CXX_STANDARD 20 without a
    # suppression on any translation unit. See that file for the compiler details.
endif()

target_sources(${CURRENT_LIB_NAME}
        PRIVATE
        ${SOURCE_FILES}
)

target_link_libraries(${CURRENT_LIB_NAME}
        PUBLIC
        ${PROJECT_NAME}::third_party
)

if(COMMAND geolio_end_file)
    geolio_end_file()
endif()
