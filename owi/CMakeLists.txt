cmake_minimum_required(VERSION 3.10)

project(owi C)

set(CMAKE_C_STANDARD_REQUIRED true)
set(CMAKE_C_STANDARD 23)

add_library(
    owi_static STATIC
    owi.c include/owi/owi.h
)
target_link_libraries(
    owi_static
    PRIVATE pico_stdlib twi_static
)
target_include_directories(
    owi_static
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)