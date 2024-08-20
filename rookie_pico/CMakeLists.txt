cmake_minimum_required(VERSION 3.13)

set(PICO_SDK_PATH "/home/wes/pico/pico-sdk")

# initialize pico-sdk from submodule
include(cmake/pico_sdk_import.cmake)
if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

project(rookie_pico C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_compile_options(
    -Wall
    -Werror
    -Wextra
    -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
    -Wno-unused-function # we have some for the docs that aren't called
    -Wno-maybe-uninitialized
)

##############
# Generate nanopb messages
# /usr/lib/x86_64-linux-gnu/cmake/nanopb
set(NANOPB_SRC_ROOT_FOLDER "/home/wes/pico/nanopb")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${NANOPB_SRC_ROOT_FOLDER}/extra)
find_package(Nanopb REQUIRED)

NANOPB_GENERATE_CPP(TARGET proto proto/Motors.proto)

######################################################
#### Libraries

include_directories(include)
# include_directories(proto)
set(LIBS
    pico_stdlib
    pico_multicore
    pico_sync 
    hardware_uart
    hardware_gpio
    hardware_i2c
    hardware_pwm
)

set(PROTO_LIBS
    nanopb
    proto)

add_library(${PROJECT_NAME}_lib
    src/Comms.cpp
    src/Config.cpp
    src/Encoders.cpp
    src/Motors.cpp    
)
target_link_libraries(${PROJECT_NAME}_lib
    ${LIBS}
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
)
target_link_libraries(rookie
    ${LIBS}
    ${PROTO_LIBS}
)

# enable usb output, disable uart output
pico_enable_stdio_usb(rookie 1)
pico_enable_stdio_uart(rookie 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(rookie)


# ##############
# # encoder_test
# add_executable(encoder_test
#     include/rookie_pico/Encoders.hpp
#     src/Encoders.cpp
# )
# target_link_libraries(encoder_test
#     ${LIBS}
# )

# # enable usb output, disable uart output
# pico_enable_stdio_usb(encoder_test 1)
# pico_enable_stdio_uart(encoder_test 0)

# # create map/bin/hex file etc.
# pico_add_extra_outputs(encoder_test)
