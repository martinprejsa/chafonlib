cmake_minimum_required(VERSION 3.20)
project(reader C)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(reader src/reader.h src/reader.c src/main.c)