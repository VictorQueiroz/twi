project(ds18b20 C)

add_library(
    ds18b20_static STATIC
        ds18b20.c ds18b20.h
)
target_link_libraries(
    ds18b20_static PRIVATE
    owi_static twi_static pico_stdlib
)
