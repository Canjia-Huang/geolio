add_executable(geobox
        main.cpp
)

target_link_libraries(geobox
        PRIVATE
        Geolio::geolio
)