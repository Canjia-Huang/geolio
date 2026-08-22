geolio_begin_file()
set(CURRENT_LIB_NAME ${PROJECT_NAME}_geobox)
set(CURRENT_LIB_ALIAS_NAME ${PROJECT_NAME}::geobox)

add_library(${CURRENT_LIB_NAME} OBJECT)
add_library(${CURRENT_LIB_ALIAS_NAME} ALIAS ${CURRENT_LIB_NAME})

# Objects are linked into the shared libGeolio, so they must be PIC.
set_target_properties(${CURRENT_LIB_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

file(GLOB SOURCE_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/*.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/object/*.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/object/*.cpp)
target_sources(${CURRENT_LIB_NAME}
        PRIVATE
        ${SOURCE_FILES}
)

target_include_directories(${CURRENT_LIB_NAME}
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../
)

target_link_libraries(${CURRENT_LIB_NAME}
        PUBLIC
        Geogram::geogram
        Geogram::geogram_gfx
        ${PROJECT_NAME}::third_party
)

geolio_end_file()
