cmake_minimum_required(VERSION 3.13)

include(FetchContent)

FetchContent_Declare(
    pico_sdk
    GIT_REPOSITORY https://github.com/raspberrypi/pico-sdk
    GIT_TAG 2.2.0
    SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/external/pico-sdk"
    GIT_SUBMODULES "lib/tinyusb"
)
FetchContent_GetProperties(pico_sdk)
if(NOT pico_sdk_POPULATED)
    FetchContent_Populate(pico_sdk)
endif()

set(PICO_SDK_PATH "${pico_sdk_SOURCE_DIR}" CACHE PATH "Path to the Raspberry Pi Pico SDK")
include("${PICO_SDK_PATH}/pico_sdk_init.cmake")

project(rookie_pico C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # clangd
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
if(NOT TARGET pico_interface)
    find_package(pico_interface REQUIRED)
    set(_PICO_INTERFACE_TARGET pico_interface::pico_interface)
else()
    set(_PICO_INTERFACE_TARGET pico_interface)
endif()

######################################################
#### Libraries

add_library(${PROJECT_NAME}_lib
    src/ClosedLoopController.cpp
    src/Config.cpp
    src/Encoders.cpp
    src/Motors.cpp
    src/OutboundQueue.cpp
    src/System.cpp
)
target_include_directories(${PROJECT_NAME}_lib
    PUBLIC
        include
        # ${pico_interface_INCLUDE_DIRS}
)
set(LIBS
    hardware_gpio
    hardware_i2c
    hardware_pwm
    hardware_spi
    hardware_uart
    hardware_watchdog
    pico_multicore
    pico_stdlib
    pico_sync 
)
target_link_libraries(${PROJECT_NAME}_lib
    ${LIBS}
    ${_PICO_INTERFACE_TARGET}
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

add_executable(imu_test
    ${HEADERS}
    src/Imu.cpp
)
target_link_libraries(imu_test
    ${LIBS}
)
pico_enable_stdio_usb(imu_test 1)
pico_add_extra_outputs(imu_test)



option(OUTBOUND_USE_UART "Use UART for outbound transport instead of USB stdio" OFF)     
if(OUTBOUND_USE_UART)
    target_compile_definitions(rookie_pico_lib PUBLIC OUTBOUND_USE_UART)
    pico_enable_stdio_usb(rookie 0)
    pico_enable_stdio_uart(rookie 1)                 
else()
    # enable usb output, disable uart output
    pico_enable_stdio_usb(rookie 1)
    pico_enable_stdio_uart(rookie 0)
endif()

set_target_properties(rookie PROPERTIES SUFFIX ".elf")

# create map/bin/hex file etc.
pico_add_extra_outputs(rookie)
