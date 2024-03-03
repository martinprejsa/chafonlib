cmake_minimum_required(VERSION 3.18)
project(chafon-reader C)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(chafon-reader src/reader.h src/reader.c src/main.c)