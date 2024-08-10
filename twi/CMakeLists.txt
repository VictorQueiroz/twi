project(twi C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED true)

include(CheckTypeSize)
include(${CMAKE_CURRENT_SOURCE_DIR}/FindHeaders.cmake)

# List of headers to check
set(INTEGER_HEADER_TYPES "stdint.h" "inttypes.h" "sys/types.h" "stddef.h")

find_headers("${INTEGER_HEADER_TYPES}" "STANDARD_TYPE_INTEGERS")

check_type_size("size_t" HAS_SIZE_T LANGUAGE C)
check_type_size("uint64_t" HAS_UINT64_T LANGUAGE C)
check_type_size("uint32_t" HAS_UINT32_T LANGUAGE C)
check_type_size("uint16_t" HAS_UINT16_T LANGUAGE C)
check_type_size("uint8_t" HAS_UINT8_T LANGUAGE C)
check_type_size("int64_t" HAS_INT64_T LANGUAGE C)
check_type_size("int32_t" HAS_INT32_T LANGUAGE C)
check_type_size("int16_t" HAS_INT16_T LANGUAGE C)
check_type_size("int8_t" HAS_INT8_T LANGUAGE C)
check_type_size("unsigned char" HAS_UNSIGNED_CHAR LANGUAGE C)
check_type_size("unsigned int" HAS_UNSIGNED_INT LANGUAGE C)
check_type_size("unsigned long" HAS_UNSIGNED_LONG LANGUAGE C)
check_type_size("unsigned long long" HAS_UNSIGNED_LONG_LONG LANGUAGE C)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.in ${CMAKE_CURRENT_BINARY_DIR}/include/twi/config.h)

add_library(
  twi_static STATIC
  twi.c include/twi/twi.h
        twi_transmission.c
)
target_include_directories(
  twi_static PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR}/include
)