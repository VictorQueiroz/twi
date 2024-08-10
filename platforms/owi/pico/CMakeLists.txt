cmake_minimum_required(VERSION 3.12)
project(owi_pico C ASM)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED true)

add_library(owi_pico owi_pico.c owi_pico.h)
target_link_libraries(owi_pico PUBLIC owi_static twi_static)