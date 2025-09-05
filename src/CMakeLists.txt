cmake_minimum_required(VERSION 3.28)
project(rookie
    VERSION 0.0.1
    DESCRIPTION "Top-level rookie build"
    LANGUAGES CXX C
)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(ExternalProject)

set(INTERFACE_PREFIX ${CMAKE_BINARY_DIR}/_install/pico_interface)
ExternalProject_Add(pico_interface
    SOURCE_DIR ${CMAKE_SOURCE_DIR}/pico_interface
    BINARY_DIR ${CMAKE_BINARY_DIR}/pico_interface/build
    CMAKE_ARGS 
        -DCMAKE_INSTALL_PREFIX=${INTERFACE_PREFIX}
    INSTALL_DIR ${INTERFACE_PREFIX}
    STEP_TARGETS build install
)

# We'll build the rookie-pico subdir as an ExternalProject so that we can use the pico sdk toolchain.
set(ROOKIE_PICO_BDIR "${CMAKE_SOURCE_DIR}/build/rookie_pico")
file(MAKE_DIRECTORY "${ROOKIE_PICO_BDIR}")
ExternalProject_Add(rookie_pico
    SOURCE_DIR "${CMAKE_SOURCE_DIR}/rookie_pico"
    CONFIGURE_COMMAND 
        ${CMAKE_COMMAND} -E chdir "${CMAKE_SOURCE_DIR}/rookie_pico"
            ${CMAKE_COMMAND} -E env
                ROOKIE_PICO_BINARY_DIR=${ROOKIE_PICO_BDIR}
                pico_interface_DIR=${INTERFACE_PREFIX}/lib/cmake/pico_interface
                CMAKE_FIND_ROOT_PATH_MODE_PACKAGE=BOTH 
                cmake --preset pico
    BUILD_COMMAND 
        ${CMAKE_COMMAND} --build "${CMAKE_BINARY_DIR}/rookie_pico"
    INSTALL_COMMAND ""
    
    USES_TERMINAL_CONFIGURE 1
    USES_TERMINAL_BUILD 1
)
add_dependencies(rookie_pico pico_interface-install)
