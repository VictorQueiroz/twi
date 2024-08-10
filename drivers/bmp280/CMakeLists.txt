project(BMP280 C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED true)

add_library(bmp280_static STATIC bmp280.h bmp280.c)
target_link_libraries(bmp280_static PRIVATE twi_static)
