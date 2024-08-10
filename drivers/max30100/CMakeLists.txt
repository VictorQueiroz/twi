project(max30100 C)

set(CMAKE_C_STANDARD 99)

add_library(
    max30100 STATIC
        max30100.c
        include/max30100/max30100.h
)
target_link_libraries(
    max30100 PRIVATE
    twi_static
)
target_include_directories(
    max30100 PUBLIC
        include
)