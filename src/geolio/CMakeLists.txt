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
