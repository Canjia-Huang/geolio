geolio_begin_file()
set(CURRENT_LIB_NAME ${PROJECT_NAME}_third_party)
set(CURRENT_LIB_ALIAS_NAME ${PROJECT_NAME}::third_party)

add_library(${CURRENT_LIB_NAME} INTERFACE)
add_library(${CURRENT_LIB_ALIAS_NAME} ALIAS ${CURRENT_LIB_NAME})

target_include_directories(${CURRENT_LIB_NAME} INTERFACE
        "${CMAKE_CURRENT_SOURCE_DIR}"
        "${CMAKE_CURRENT_SOURCE_DIR}/CLI11/include"
#        "${CMAKE_CURRENT_SOURCE_DIR}/json/include"
#        "${CMAKE_CURRENT_SOURCE_DIR}/libTimekeeper/src"
        "${CMAKE_CURRENT_SOURCE_DIR}/imoguizmo"
        "${CMAKE_CURRENT_SOURCE_DIR}/spdlog/include"
        "${GEOGRAM_INCLUDE_DIR}/geogram_gfx/third_party/imgui"
)

geolio_end_file()