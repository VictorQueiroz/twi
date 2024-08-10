add_library(twi_pico_static STATIC twi_pico.c twi_pico.h)
target_link_libraries(twi_pico_static PUBLIC twi_static)
