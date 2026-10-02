set(CURRENT_APP_NAME lpcvt)

add_executable(${CURRENT_APP_NAME}
        main.cpp
)

target_link_libraries(${CURRENT_APP_NAME}
        PRIVATE
        Geolio::geolio
)
