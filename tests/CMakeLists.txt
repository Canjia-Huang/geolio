if(COMMAND geolio_begin_file)
    geolio_begin_file()
endif()
set(CURRENT_LIB_NAME ${PROJECT_NAME}_tests)

########################################################################################################################
# Download and install googletest
########################################################################################################################
include(FetchContent)
FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

########################################################################################################################
# Build tests
########################################################################################################################
file(GLOB SOURCE_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/common/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/high_order/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/io/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/local_operation_optimization/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/*.cpp
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
gtest_discover_tests(${CURRENT_LIB_NAME})

if(COMMAND geolio_end_file)
    geolio_end_file()
endif()