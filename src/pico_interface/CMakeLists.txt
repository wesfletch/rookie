cmake_minimum_required(VERSION 3.28)

project(pico_interface
    VERSION 0.0.1
    DESCRIPTION "Interop between MCU and Main computer"
    LANGUAGES CXX C
)

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

include(GNUInstallDirs)

###################################################################################################
### Dependencies

# Use NanoPB for generating proto headers.
include(FetchContent)
file(MAKE_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/nanopb")
FetchContent_Declare(
    nanopb
    GIT_REPOSITORY https://github.com/nanopb/nanopb
    GIT_TAG master
    SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/nanopb"
)
FetchContent_GetProperties(nanopb)
if(NOT nanopb_POPULATED)
    FetchContent_Populate(nanopb)
endif()

# Load nanopb's CMake helpers (provides nanopb_generate_cpp)
list(APPEND CMAKE_MODULE_PATH "${nanopb_SOURCE_DIR}/extra")
find_package(Nanopb REQUIRED MODULE)


###################################################################################################
### Code generation

# Generate the .cpp/.hpp files from the .protos
add_custom_target(generate_protos
    COMMAND bash "-c" "${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_protos.sh"
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Generating protobuf sources"
)

set(PICO_INTERFACE_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/include/pico_interface
)
file(GLOB NANOPB_C
    CONFIGURE_DEPENDS
    ${PICO_INTERFACE_DIR}/pb_*.c
)
file(GLOB PICO_INTERFACE_CPP
    CONFIGURE_DEPENDS
    ${PICO_INTERFACE_DIR}/protos/*.pb.cpp
)
###################################################################################################
### Interface library

add_library(${PROJECT_NAME} INTERFACE)
add_dependencies(${PROJECT_NAME} generate_protos)

target_sources(${PROJECT_NAME}
    INTERFACE
        # Build tree: absolute paths to generated .c plus nanopb runtime .c
        $<BUILD_INTERFACE:${NANOPB_C}>
        $<BUILD_INTERFACE:${PICO_INTERFACE_CPP}>
)

# Propagate include paths (generated headers + nanopb headers)
target_include_directories(${PROJECT_NAME}
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_compile_features(${PROJECT_NAME}
    INTERFACE
        cxx_std_17
        c_std_99
)
set_property(TARGET ${PROJECT_NAME} PROPERTY
    INTERFACE_POSITION_INDEPENDENT_CODE ON
)

###################################################################################################
### Install rules

install(
    DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
)

# Export the interface target for consumers
install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
)

###################################################################################################
### Export target and package config files

include(CMakePackageConfigHelpers)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}"
)

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    VERSION "${PROJECT_VERSION}"
    COMPATIBILITY AnyNewerVersion
    ARCH_INDEPENDENT
)

install(
    FILES
        "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}"
)

install(
    EXPORT ${PROJECT_NAME}Targets
    NAMESPACE "${PROJECT_NAME}::"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}"
)
