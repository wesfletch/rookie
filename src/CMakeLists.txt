cmake_minimum_required(VERSION 3.28)
project(rookie
    VERSION 0.0.1
    DESCRIPTION "Top-level rookie build"
    LANGUAGES CXX C
)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory(pico_interface)

option(BUILD_FIRMWARE "Build rookie_pico firmware" OFF)
if(BUILD_FIRMWARE)
    add_subdirectory(rookie_pico)
endif()

