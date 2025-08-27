cmake_minimum_required(VERSION 3.13)

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
FetchContent_MakeAvailable(nanopb)

# Load nanopb's CMake helpers (provides nanopb_generate_cpp)
list(APPEND CMAKE_MODULE_PATH "${nanopb_SOURCE_DIR}/extra")
find_package(Nanopb REQUIRED MODULE)


###################################################################################################
### Code generation

# Gather all protos in our /protos directory
file(GLOB proto_files CONFIGURE_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/protos/*.proto")
message("Got files ${proto_files}")

# Generate the .c and .h headers with nanopb
nanopb_generate_cpp(
    TARGET protos_gen 
    RELPATH "${CMAKE_CURRENT_LIST_DIR}"
    ${proto_files}
)
add_custom_target(nanopb_codegen ALL DEPENDS protos_gen)

# Compute paths to the generated .pb.c/.pb.h (mirrors your source layout) ---
set(generated_c)
set(generated_h)
foreach(p ${proto_files})
    file(RELATIVE_PATH relative_path "${CMAKE_CURRENT_SOURCE_DIR}" "${p}")
    string(REGEX REPLACE "\\.proto$" ".pb.c" gen_c "${relative_path}")
    string(REGEX REPLACE "\\.proto$" ".pb.h" gen_h "${relative_path}")
    list(APPEND generated_c "${CMAKE_CURRENT_BINARY_DIR}/${gen_c}")
    list(APPEND generated_h "${CMAKE_CURRENT_BINARY_DIR}/${gen_h}")
endforeach()
message("GEN_C ${generated_c};")
message("GEN_H ${generated_h}")

# For nice includes: <pico_interface/protos/Thing.pb.h>
# Copy build headers to build/include/pico_interface/protos/
set(PICO_IFACE_BUILD_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/include/${PROJECT_NAME}")
add_custom_target(pico_iface_headers ALL
    COMMAND ${CMAKE_COMMAND} -E rm -rf "${PICO_IFACE_BUILD_INCLUDE_DIR}/protos"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${PICO_IFACE_BUILD_INCLUDE_DIR}/protos"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_CURRENT_BINARY_DIR}/protos"
            "${PICO_IFACE_BUILD_INCLUDE_DIR}/protos"
    DEPENDS protos_gen
)

###################################################################################################
### Header-only-esque interface target

add_library(${PROJECT_NAME} INTERFACE)
add_dependencies(${PROJECT_NAME} protos_gen pico_iface_headers)

set(generated_c_install)
foreach(p ${proto_files})
    file(RELATIVE_PATH rel "${CMAKE_CURRENT_SOURCE_DIR}" "${p}")    # protos/Foo.proto
    string(REGEX REPLACE "\\.proto$" ".pb.c" gen_c_rel "${rel}")    # protos/Foo.pb.c
    list(APPEND generated_c_install 
        "$<INSTALL_PREFIX>/${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/sources/${gen_c_rel}")
endforeach()

target_sources(${PROJECT_NAME}
    INTERFACE
        # Build tree: absolute paths to generated .c plus nanopb runtime .c
        $<BUILD_INTERFACE:${generated_c}>
        $<BUILD_INTERFACE:${nanopb_SOURCE_DIR}/pb_common.c>
        $<BUILD_INTERFACE:${nanopb_SOURCE_DIR}/pb_encode.c>
        $<BUILD_INTERFACE:${nanopb_SOURCE_DIR}/pb_decode.c>

        # Install tree: point to installed copies
        $<INSTALL_INTERFACE:${generated_c_install}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/nanopb/pb_common.c>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/nanopb/pb_encode.c>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/nanopb/pb_decode.c>
)

# Propagate include paths (generated headers + nanopb headers)
target_include_directories(${PROJECT_NAME}
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
        $<BUILD_INTERFACE:${nanopb_SOURCE_DIR}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)


###################################################################################################
### Install rules

# Include our code-gen'd headers and source files in our exported interface
install(
    DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/protos/"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/protos"
    FILES_MATCHING PATTERN "*.pb.h"
)
install(
    DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/protos/"
    DESTINATION "${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/sources/protos"
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
    DESTINATION "${CMAKE_INSTALL_DATADIR}/cmake/${PROJECT_NAME}"
)

# Export the interface target for consumers
install(
    TARGETS ${PROJECT_NAME}
    EXPORT picoProtosTargets
)

install(
    EXPORT picoProtosTargets
    NAMESPACE "${PROJECT_NAME}::"
    DESTINATION "${CMAKE_INSTALL_DATADIR}/cmake/${PROJECT_NAME}"
)

# ----------------------------------------------------------------------------------------------------------------------
# Usage (for reference):
#   find_package(pico_interface REQUIRED)
#   target_link_libraries(their_target PRIVATE pico_interface::pico_interface)
#   #include <pico_interface/protos/YourMessage.pb.h>
#   #include <pb.h>