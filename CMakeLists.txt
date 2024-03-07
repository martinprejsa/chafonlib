cmake_minimum_required(VERSION 3.18)
project(chafon-reader C)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_STANDARD 11)

add_library(chafon-reader src/reader.c)
target_include_directories(chafon-reader PUBLIC include/chafon-reader)

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME AND BUILD_TESTING)
  include(CTest)
  add_subdirectory(tests)
endif()