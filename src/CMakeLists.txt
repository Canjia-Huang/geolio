get_filename_component(CURRENT_FILE_NAME "${CMAKE_CURRENT_LIST_FILE}" NAME)
list(APPEND CMAKE_MESSAGE_INDENT "[${CMAKE_CURRENT_LIST_FILE}]: ")
message(STATUS "")
set(CURRENT_LIB_NAME ${PROJECT_NAME}_LIB)

add_library(${CURRENT_LIB_NAME})

target_include_directories(${CURRENT_LIB_NAME}
        PRIVATE
        "${PROJECT_SOURCE_DIR}/include/"
)

# configure sources
file(GLOB SOURCE_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/*.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/*.cxx)

# add sources to the library
target_sources(${CURRENT_LIB_NAME}
        PRIVATE
        ${SOURCE_FILES}
)
target_link_libraries(${CURRENT_LIB_NAME}
        PUBLIC
        Geogram::geogram
)

list(POP_BACK CMAKE_MESSAGE_INDENT)