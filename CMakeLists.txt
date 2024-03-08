cmake_minimum_required(VERSION 3.18)
project(chafon-reader C)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_STANDARD 11)

add_library(chafon-reader SHARED src/reader.c)
target_include_directories(chafon-reader PUBLIC include)

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME AND BUILD_TESTING)
  include(CTest)
  add_subdirectory(tests)
endif()

set_target_properties(chafon-reader PROPERTIES PUBLIC_HEADER "include/chafon-reader/reader.h;include/chafon-reader/commands.h")

install(TARGETS chafon-reader 
        LIBRARY DESTINATION /usr/local/lib
        PUBLIC_HEADER DESTINATION /usr/local/include/chafon-reader)