cmake_minimum_required(VERSION 3.13)

# Where to put external dependencies that we fetch
set(EXTERNAL_DIR "${CMAKE_CURRENT_LIST_DIR}/external")
file(MAKE_DIRECTORY "${EXTERNAL_DIR}")

# Dependency: pico_sdk, the toolchain for building code for the RPi Pico
include(FetchContent)
file(MAKE_DIRECTORY "${EXTERNAL_DIR}/pico-sdk")
FetchContent_Declare(
    pico_sdk_FETCH
    GIT_REPOSITORY https://github.com/raspberrypi/pico-sdk
    GIT_TAG 1.5.1 # TODO: update this to 2.0
    SOURCE_DIR "${EXTERNAL_DIR}/pico-sdk"
    GIT_SUBMODULES_RECURSE FALSE
)
if (NOT pico_sdk_FETCH)
    FetchContent_Populate(pico_sdk_FETCH)
endif()
set(PICO_SDK_PATH "${EXTERNAL_DIR}/pico-sdk" CACHE PATH "Path to the Raspberry Pi Pico SDK")
set(PICO_SDK_INIT_CMAKE_FILE "${PICO_SDK_PATH}/pico_sdk_init.cmake")

# Must be done before call to project()
include("${PICO_SDK_INIT_CMAKE_FILE}")

project(rookie_pico C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(
    -Wall
    -Werror
    -Wextra
    -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
    -Wno-unused-function # we have some for the docs that aren't called
    -Wno-maybe-uninitialized
)

# initialize the Raspberry Pi Pico SDK
# Needs PICO_SDK_PATH and PICO_SDK_INIT_CMAKE_FILE set
pico_sdk_init()

set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE BOTH)
set(CMAKE_FIND_PACKAGE_PREFER_CONFIG ON)  # prefer *Config.cmake over Find*.cmake
find_package(pico_interface REQUIRED)

######################################################
#### Libraries

add_library(${PROJECT_NAME}_lib
    src/ClosedLoopController.cpp
    src/Config.cpp
    src/Encoders.cpp
    src/Motors.cpp
)
target_include_directories(${PROJECT_NAME}_lib
    PUBLIC
        include
        # ${pico_interface_INCLUDE_DIRS}
)
set(LIBS
    pico_stdlib
    pico_multicore
    pico_sync 
    hardware_uart
    hardware_gpio
    hardware_i2c
    hardware_pwm
    hardware_watchdog
)
target_link_libraries(${PROJECT_NAME}_lib
    ${LIBS}
    pico_interface::pico_interface
)
set_target_properties(${PROJECT_NAME}_lib
    PROPERTIES 
        VERSION ${PROJECT_VERSION}
        SOVERSION ${PROJECT_VERSION_MAJOR}
)
set(LIBS ${LIBS} ${PROJECT_NAME}_lib)

######################################################
#### Executables

##############
# rookie
add_executable(rookie
    ${HEADERS}
    src/main.cpp
    # src/encoder_test.cpp
)
target_link_libraries(rookie
    ${LIBS}
)

add_executable(encoder_test
    ${HEADERS}
    src/encoder_test.cpp
)
target_link_libraries(encoder_test
    ${LIBS}
)

add_executable(command_test
    ${HEADERS}
    src/command_test.cpp
)
target_link_libraries(command_test
    ${LIBS}
)   

# enable usb output, disable uart output
pico_enable_stdio_usb(rookie 1)
pico_enable_stdio_uart(rookie 0)

# create map/bin/hex file etc.
# pico_add_extra_outputs(rookie)
