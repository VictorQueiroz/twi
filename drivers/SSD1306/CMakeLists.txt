project(TWI_SSD1306 C)

set(CMAKE_C_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED true)

add_library(
  twi_ssd1306 STATIC
        SSD1306.c SSD1306.h
)
target_link_libraries(
  twi_ssd1306
  PRIVATE
  twi_static
)
