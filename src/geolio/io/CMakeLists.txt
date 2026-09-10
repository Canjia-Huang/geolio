if(COMMAND geolio_begin_file)
    geolio_begin_file()
endif()
set(CURRENT_LIB_NAME ${PROJECT_NAME}_io)
set(CURRENT_LIB_ALIAS_NAME ${PROJECT_NAME}::io)

add_library(${CURRENT_LIB_NAME} OBJECT)
add_library(${CURRENT_LIB_ALIAS_NAME} ALIAS ${CURRENT_LIB_NAME})

# Objects are linked into the shared libGeolio, so they must be PIC.
set_target_properties(${CURRENT_LIB_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

file(GLOB SOURCE_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/*.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
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
        ${PROJECT_NAME}::high_order
        ${PROJECT_NAME}::third_party
)

if(COMMAND geolio_end_file)
    geolio_end_file()
endif()
