add_library(${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
    PRIVATE
    "${CMAKE_SOURCE_DIR}/src"
    "${CMAKE_SOURCE_DIR}/deps"
)

file(GLOB SOURCE
        ${CMAKE_CURRENT_SOURCE_DIR}/*.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

target_sources(${PROJECT_NAME}
        PRIVATE
        ${SOURCE}
)

target_link_libraries(${PROJECT_NAME}
        PRIVATE
        Geogram::geogram
)