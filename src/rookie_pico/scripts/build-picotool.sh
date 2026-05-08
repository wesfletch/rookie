#!/usr/bin/env bash

set -o errexit
set -o pipefail
set -o nounset
# set -o xtrace

SCRIPT_DIR="$(dirname "$(readlink -f -- "${BASH_SOURCE[0]}")")"
PARENT_DIR="${SCRIPT_DIR}/.."

# Where to fetch picotool from during clone
PICOTOOL_URI="https://github.com/raspberrypi/picotool"

EXTERNAL_DIR="${PARENT_DIR}/external"
# Where the picotool repo is checked out.
PICOTOOL_DIR="${EXTERNAL_DIR}/picotool"
PICO_SDK_DIR="${PICO_SDK_DIR:-"${EXTERNAL_DIR}/pico-sdk"}"
# Where to put the final executable
INSTALL_PREFIX="${HOME}/.local"

COMMAND="build"


function _ensure_deps()
{
    sudo apt install -y \
        build-essential \
        pkg-config \
        libusb-1.0-0-dev \
        cmake
}

function _clone()
{
    if [[ -d "${PICOTOOL_DIR:?}" ]]; then
        echo "* picotool repo already checked out at ${PICOTOOL_DIR:?}; skipping"
        return 0
    fi

    echo "* Cloning picotool to ${PICOTOOL_DIR:?}"
    git clone "${PICOTOOL_URI}" "${PICOTOOL_DIR}"

    # NOTE: It looks like we don't need to do this: the actual CMakeLists.txt in picotool
    # makes reference to no longer needing submodules, but the instructions still include it
    # git -C "${PICOTOOL_DIR}" submodule update --init lib/mbedtls
}

function _build()
{
    if [[ ! -d "${PICO_SDK_DIR:?}" ]]; then
        echo "ERR: pico-sdk not found at ${PICO_SDK_DIR}"
        echo "     Run 'cmake --preset dev' first to set up the build"
        return 1
    fi

    echo "* Configuring build"
    cmake \
        -S "${PICOTOOL_DIR}" \
        -B "${PICOTOOL_DIR}/build" \
        -DPICO_SDK_PATH="${PICO_SDK_DIR}" \
        -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}"

    echo "* Building"
    cmake --build "${PICOTOOL_DIR}/build" --parallel "$(nproc)"

    echo "* Installing to ${INSTALL_PREFIX}"
    cmake --install "${PICOTOOL_DIR}/build"
}

function _clean()
{
    if [[ ! -d "${PICOTOOL_DIR}/build" ]]; then
        echo "* Nothing to clean"
        return 0
    fi

    echo "* Removing ${PICOTOOL_DIR}/build"
    rm -rf "${PICOTOOL_DIR}/build"
}


while [[ "${#}" -gt 0 ]]; do
    ARG="${1}"
    case "${ARG}" in
        "build")
            COMMAND="build"
            ;;
        "clean")
            COMMAND="clean"
            ;;
        "-h" | "--help")
            echo "Usage: $(basename "$0") [build|clean]"
            exit 0
            ;;
        *)
            echo "What? ${ARG}"
            exit 1
            ;;
    esac
    shift
done

case "${COMMAND}" in
    "build")
        if ! _ensure_deps; then
            echo "ERR: failed to satisfy dependencies!"
            exit 1
        fi
        _clone
        _build
        ;;
    "clean")
        _clean
        ;;
esac
