if(COMMAND geolio_begin_file)
    geolio_begin_file()
endif()
set(CURRENT_LIB_NAME ${PROJECT_NAME}_tests)

########################################################################################################################
# googletest
########################################################################################################################
# Vendored as a git submodule (third_party/googletest, initialized by
# `git submodule update --init --recursive`) rather than downloaded at configure
# time. The submodule is pinned to the same revision the previous FetchContent
# declaration used (googletest 1.12.1, commit 03597a01) and, like the other
# third-party checkouts, is cloned shallow.
#
# For Windows: prevent googletest from overriding the parent project's
# compiler/linker settings.
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# The binary directory is given explicitly so googletest's build tree stays next
# to the other dependency build trees instead of appearing under tests/.
add_subdirectory("${PROJECT_SOURCE_DIR}/third_party/googletest"
        "${CMAKE_BINARY_DIR}/third_party/googletest-build")

########################################################################################################################
# Build tests
########################################################################################################################
file(GLOB SOURCE_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/common/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/geobox/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/high_order/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/io/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/local_operation_optimization/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/miq/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/motorcycle/*.cpp
)

add_executable(${CURRENT_LIB_NAME}
        ${SOURCE_FILES})

# Keep the test executable in build/tests (its own subdirectory), overriding the
# global build/bin set in the top-level CMakeLists.txt.
set_target_properties(${CURRENT_LIB_NAME} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")

target_link_libraries(${CURRENT_LIB_NAME}
        ${PROJECT_NAME}::geolio
        GTest::gtest_main
)

add_compile_definitions(TEST_DATA_PATH="${CMAKE_CURRENT_SOURCE_DIR}/data/")

if(WIN32)
    # gtest_discover_tests() runs the test executable directly at build time to
    # enumerate tests, without any PATH augmentation, so every DLL it needs at
    # runtime must sit next to the executable (Windows searches the exe
    # directory first).
    add_custom_command(TARGET ${CURRENT_LIB_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<TARGET_FILE:${PROJECT_NAME}>"
            "$<TARGET_FILE_DIR:${CURRENT_LIB_NAME}>"
        COMMENT "Copying geolio.dll to test directory"
    )
    # geolio.dll (and thus the test executable) depends not only on geogram.dll
    # but - through the geobox component - also on geogram_gfx.dll and its own
    # runtime dependencies (e.g. glfw.dll). Copy the whole geogram DLL set
    # instead of just geogram.dll.
    if(GEOGRAM_DLL)
        get_filename_component(GEOGRAM_DLL_DIR "${GEOGRAM_DLL}" DIRECTORY)
        file(GLOB GEOGRAM_RUNTIME_DLLS "${GEOGRAM_DLL_DIR}/*.dll")
        if(GEOGRAM_RUNTIME_DLLS)
            add_custom_command(TARGET ${CURRENT_LIB_NAME} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    ${GEOGRAM_RUNTIME_DLLS}
                    "$<TARGET_FILE_DIR:${CURRENT_LIB_NAME}>"
                COMMENT "Copying geogram runtime DLLs to test directory"
            )
        endif()
    endif()
endif()

include(GoogleTest)

# Test enumeration runs the test executable with --gtest_list_tests, which only
# needs the DLLs to load (gtest returns before the test environments run), but on
# Windows that first launch of a freshly linked Geolio_tests.exe is not cheap: it
# loads Geolio.dll plus the geogram Debug DLL set copied above while the file cache
# is still cold and Defender is scanning those binaries for the first time. CMake's
# default discovery timeout is 5 seconds, and when that launch loses the race
# discovery kills the process and fails the *build* with
#
#   Error running test executable.
#     Result: Process terminated due to timeout
#
# even though the test list itself is perfectly fine (observed on the nightly
# Windows Debug job, while the same commit passed the regular Windows Debug job).
# Two guards against that false negative:
#
#   DISCOVERY_TIMEOUT 120 - a cold first launch is allowed to take its time.
#   DISCOVERY_MODE PRE_TEST - enumeration happens at ctest time instead of during
#     the post-build step, so a slow launch can no longer fail the build; the list
#     is cached next to the executable and only re-generated when it changes.
gtest_discover_tests(${CURRENT_LIB_NAME}
        DISCOVERY_TIMEOUT 120
        DISCOVERY_MODE PRE_TEST
)

if(COMMAND geolio_end_file)
    geolio_end_file()
endif()