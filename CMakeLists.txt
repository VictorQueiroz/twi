cmake_minimum_required(VERSION 3.28)

project(twi ASM C)

# Check if CMAKE_INCLUDE_PATH is set
if(CMAKE_INCLUDE_PATH)
  # Split the paths by the appropriate delimiter (colon on Unix, semicolon on Windows)
  if(UNIX)
    string(REPLACE ":" ";" INCLUDE_DIRS_LIST "${CMAKE_INCLUDE_PATH}")
  else()
    string(REPLACE ";" ";" INCLUDE_DIRS_LIST "${CMAKE_INCLUDE_PATH}")
  endif()

  # Include the directories in the project
  include_directories(${INCLUDE_DIRS_LIST})
endif()

add_subdirectory(twi)
add_subdirectory(owi)
add_subdirectory(platforms)
#add_subdirectory(SSD1306)
#add_subdirectory(max30100)
#add_subdirectory(bmp280)
#add_subdirectory(ds18b20)
