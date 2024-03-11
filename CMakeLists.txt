cmake_minimum_required(VERSION 3.18)

project(chafonlib DESCRIPTION "Interacts with chafon rfid readers"
                  LANGUAGES C)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_INSTALL_RPATH $ORIGIN)

add_library(chafonlib STATIC src/reader.c)

set_target_properties(chafonlib PROPERTIES
                                VERSION 1.0.0
                                SOVERSION 1)

target_include_directories(chafonlib PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME AND BUILD_TESTING)
  include(CTest)
  add_subdirectory(tests)
endif()

install(
    TARGETS chafonlib
    EXPORT chafonlibTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

include(CMakePackageConfigHelpers)

write_basic_package_version_file(
        chafonlibConfigVersion.cmake
        VERSION 1.0.0
        COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
    "${CMAKE_CURRENT_LIST_DIR}/cmake/chafonlibConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/chafonlibConfig.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/cmake/chafonlib
)

install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/chafonlibConfig.cmake 
          ${CMAKE_CURRENT_BINARY_DIR}/chafonlibConfigVersion.cmake 
    DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/cmake/chafonlib)

install(
    EXPORT chafonlibTargets
    FILE chafonlibTargets.cmake
    DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/cmake/chafonlib
)