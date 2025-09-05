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

set(PROTO_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/protos")
set(proto_files
    "${PROTO_SOURCE_DIR}/Heartbeat.proto"
    "${PROTO_SOURCE_DIR}/Motors.proto"
    "${PROTO_SOURCE_DIR}/Reset.proto"
    "${PROTO_SOURCE_DIR}/State.proto"
    "${PROTO_SOURCE_DIR}/Velocity.proto"
)
message("Generating .proto files: ${proto_files}")

# Generate the .c and .h headers with nanopb
set(proto_relroot "${CMAKE_CURRENT_LIST_DIR}/..")
nanopb_generate_cpp(
    TARGET generated_protos
    RELPATH "${proto_relroot}"
    "${proto_files}"
)

# nanopb-generated files are placed in the binary dir;
# calculate the paths of all files created so we can use them at config time,
# as opposed to file globs that won't work until build time
set(GENERATED_PROTO_DIR "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/protos")
message("GENERATED_PROTO_DIR==${GENERATED_PROTO_DIR}")
foreach(proto_file ${proto_files})
    file(RELATIVE_PATH relative_file_path "${proto_relroot}" "${proto_file}")
    string(REGEX REPLACE "[.]proto$" ".pb.h" generated_h_file "${relative_file_path}")
    string(REGEX REPLACE "[.]proto$" ".pb.c" generated_c_file "${relative_file_path}")
    list(APPEND generated_h_files "${generated_h_file}")
    list(APPEND generated_c_files "${generated_c_file}")

    # Later, during install, we'll move our generated files to make for nicer imports.
    # This path is where the .c files are AFTER the move.
    list(APPEND generated_c_files_install "${CMAKE_CURRENT_BINARY_DIR}/include/${generated_c_file}")
endforeach()
message("GENERATED H FILES: ${generated_h_files}")
message("GENERATED C FILES: ${generated_c_files}")
message("GENERATED C FILES (INSTALL): ${generated_c_files_install}")

# For nice includes: <pico_interface/protos/Thing.pb.h> instead of <protos/Thing.pb.h>,
# we'll copy our generated files into include/pico_interface/protos instead of just pico_interface/protos
set(PICO_IFACE_BUILD_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/include/${PROJECT_NAME}")
add_custom_target(pico_iface_headers ALL
    COMMAND ${CMAKE_COMMAND} -E rm -rf "${PICO_IFACE_BUILD_INCLUDE_DIR}/protos"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${PICO_IFACE_BUILD_INCLUDE_DIR}"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${GENERATED_PROTO_DIR}"
            "${PICO_IFACE_BUILD_INCLUDE_DIR}"
    DEPENDS generated_protos
)

###################################################################################################
### Header-only-esque interface target

add_library(${PROJECT_NAME} INTERFACE)
add_dependencies(${PROJECT_NAME} generated_protos pico_iface_headers)

target_sources(${PROJECT_NAME}
    INTERFACE
        # Build tree: absolute paths to generated .c plus nanopb runtime .c
        $<BUILD_INTERFACE:${generated_c_files}>
        $<BUILD_INTERFACE:${nanopb_SOURCE_DIR}/pb_common.c>
        $<BUILD_INTERFACE:${nanopb_SOURCE_DIR}/pb_encode.c>
        $<BUILD_INTERFACE:${nanopb_SOURCE_DIR}/pb_decode.c>

        # Install tree: point to installed copies
        # $<INSTALL_INTERFACE:${generated_c_files}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/nanopb/pb_common.c>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/nanopb/pb_encode.c>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/nanopb/pb_decode.c>
)

# Propagate include paths (generated headers + nanopb headers)
target_include_directories(${PROJECT_NAME}
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include/${PROJECT_NAME}>
        $<BUILD_INTERFACE:${nanopb_SOURCE_DIR}>

        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)


###################################################################################################
### Install rules

# Install pico_interface's own public headers (e.g., include/pico_interface/*.hpp)
install(
    DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
    FILES_MATCHING PATTERN "*.h*"
)

# Include our code-gen'd headers and source files in our exported interface
install(
    DIRECTORY "${GENERATED_PROTO_DIR}"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/"
    FILES_MATCHING PATTERN "*.pb.h"
)
install(
    DIRECTORY "${PICO_IFACE_BUILD_INCLUDE_DIR}/"
    DESTINATION "${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/sources"
    FILES_MATCHING PATTERN "*.pb.c"
)

# Include the nanopb .c and .h files in our exported interface
install(
    FILES
        "${nanopb_SOURCE_DIR}/pb_common.c"
        "${nanopb_SOURCE_DIR}/pb_encode.c"
        "${nanopb_SOURCE_DIR}/pb_decode.c"
    DESTINATION "${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/nanopb"
)
install(
    DIRECTORY "${nanopb_SOURCE_DIR}/"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
    FILES_MATCHING PATTERN "pb*.h"
)


###################################################################################################
### Export target and package config files

include(CMakePackageConfigHelpers)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}Config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}"
)

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    VERSION "${PROJECT_VERSION}"
    COMPATIBILITY AnyNewerVersion
    ARCH_INDEPENDENT
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}"
)

# Export the interface target for consumers
install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
)

install(
    EXPORT ${PROJECT_NAME}Targets
    NAMESPACE "${PROJECT_NAME}::"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}"
)

# ----------------------------------------------------------------------------------------------------------------------
# Usage (for reference):
#   find_package(pico_interface REQUIRED)
#   target_link_libraries(their_target PRIVATE pico_interface::pico_interface)
#   #include <pico_interface/protos/YourMessage.pb.h>
#   #include <pb.h>