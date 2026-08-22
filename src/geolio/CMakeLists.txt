geolio_begin_file()
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

# Add subdirectory (component library)
add_subdirectory(common)
add_subdirectory(geobox)
add_subdirectory(local_operation_optimization)
add_subdirectory(mesh)

target_link_libraries(${CURRENT_LIB_NAME}
        PUBLIC
        Geogram::geogram
        ${PROJECT_NAME}::geobox
        ${PROJECT_NAME}::local_operation_optimization
        ${PROJECT_NAME}::mesh
        ${PROJECT_NAME}::third_party
)

geolio_end_file()
