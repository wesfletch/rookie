#!/usr/bin/env bash

set -o errexit
set -o pipefail
set -o nounset
# set -o xtrace

SCRIPT_DIR="$(dirname "$(readlink -f -- "${BASH_SOURCE[0]}")")"

PROTO_DIR="${SCRIPT_DIR}/../protos"

CPP_OUTPUT_DIR="${SCRIPT_DIR}/../include/pico_interface"
CPP_NANOPB_DIR="${SCRIPT_DIR}/../nanopb"

PYTHON_PKG_DIR="${SCRIPT_DIR}/../py/pico-interface"
PYTHON_OUTPUT_DIR="${PYTHON_PKG_DIR}/src/pico_interface"

_PYTHON="$(which python3)"

function ensure_deps()
{
    echo "* Installing deps"
    if [[ ! -d "${SCRIPT_DIR}/.venv" ]]; then
        "${_PYTHON}" -m venv .venv
    fi

    _PYTHON="${SCRIPT_DIR}/.venv/bin/python3"
    echo "Using python at ${_PYTHON}"

    "${_PYTHON}" -m pip install grpcio-tools protobuf
}

function generate_cpp_nanopb()
{
    local output_dir="${1:-"${CPP_OUTPUT_DIR}"}"

    mkdir -p "${output_dir}"
    rm -f "${output_dir}/protos"/*.pb.*

    pushd "${PROTO_DIR}" >>/dev/null || exit 1

    echo "* Generating headers for .proto files"
    for proto_file in ./*.proto; do
        abs_proto="${PROTO_DIR}/${proto_file#./}"
        echo ">>> ${proto_file}:"
        "${_PYTHON}" \
            "${CPP_NANOPB_DIR}/generator/nanopb_generator.py" \
            --output-dir "${output_dir}/protos" \
            --cpp-descriptors \
            --no-timestamp \
            --header-extension ".hpp" \
            --source-extension ".cpp" \
            --library-include-format '#include <pico_interface/%s>' \
            --generated-include-format '#include <pico_interface/protos/%s>' \
            -I "${PROTO_DIR}" \
            -I "${CPP_NANOPB_DIR}/generator/proto" \
            -I /usr/include \
            "${abs_proto}"
    done

    popd >>/dev/null || exit 1

    echo "* Copying pb headers from ${CPP_NANOPB_DIR} to ${output_dir}"
    cp "${CPP_NANOPB_DIR}"/pb*{.h,.c} "${output_dir}"
}

function generate_python()
{
    local output_dir="${1:-"${PYTHON_OUTPUT_DIR}"}"

    mkdir -p "${output_dir}"
    rm -f "${output_dir}"/*_pb2.py*

    PROTO_FILES="$(find "${PROTO_DIR}" -name "*.proto")"
    echo "Found .proto files: ${PROTO_FILES[*]}"

    # shellcheck disable=SC2086
    "${_PYTHON}" -m grpc_tools.protoc \
        --proto_path="${PROTO_DIR}" \
        --proto_path="${CPP_NANOPB_DIR}/generator/proto" \
        --python_out="${output_dir}" \
        --pyi_out="${output_dir}" \
        ${PROTO_FILES}

    if command -v uv &>/dev/null; then
        pushd ${PYTHON_PKG_DIR} >>/dev/null || exit 1
        echo "* Building the python package at ${PYTHON_PKG_DIR}"
        uv build
        popd >>/dev/null || exit 1
    else
        echo "* uv not found, skipping Python package build"
    fi
}


if ! ensure_deps; then
    echo "Failed to satisfy dependencies!"
    exit 1
fi

generate_cpp_nanopb
generate_python
