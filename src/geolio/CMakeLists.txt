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

# Add subdirectory (component library)
add_subdirectory(common)
add_subdirectory(geobox)
#add_subdirectory(geogram_plus) // header only
add_subdirectory(high_order)
add_subdirectory(io)
add_subdirectory(local_operation_optimization)
add_subdirectory(mesh)
add_subdirectory(motorcycle)

target_link_libraries(${CURRENT_LIB_NAME}
        PUBLIC
        ${PROJECT_NAME}::common
        ${PROJECT_NAME}::geobox
        ${PROJECT_NAME}::high_order
        ${PROJECT_NAME}::io
        ${PROJECT_NAME}::local_operation_optimization
        ${PROJECT_NAME}::mesh
        ${PROJECT_NAME}::motorcycle
        ${PROJECT_NAME}::third_party
)

if(COMMAND geolio_end_file)
    geolio_end_file()
endif()
