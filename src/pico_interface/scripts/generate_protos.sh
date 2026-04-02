#!/usr/bin/env bash

set -o errexit
set -o pipefail
set -o nounset
# set -o xtrace

SCRIPT_DIR="$(dirname "$(readlink -f -- "${BASH_SOURCE[0]}")")"

PROTO_DIR="${SCRIPT_DIR}/../protos"
NANOPB_DIR="${SCRIPT_DIR}/../nanopb"
OUTPUT_DIR="${SCRIPT_DIR}/../include/pico_interface"

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

    mkdir -p "${OUTPUT_DIR}"
    mkdir -p "${OUTPUT_DIR}/protos"
}
ensure_deps

rm "${OUTPUT_DIR}/protos"/*.pb.*

pushd "${PROTO_DIR}" >>/dev/null || exit 1

echo "* Generating headers for .proto files"
for proto_file in ./*.proto; do
    echo ">>> ${proto_file}:"
    "${_PYTHON}" \
        "${NANOPB_DIR}/generator/nanopb_generator.py" \
        --output-dir "${OUTPUT_DIR}/protos" \
        --cpp-descriptors \
        --no-timestamp \
        --header-extension ".hpp" \
        --source-extension ".cpp" \
        --library-include-format '#include <pico_interface/%s>' \
        --generated-include-format '#include <pico_interface/protos/%s>' \
        "${proto_file}" 
done

popd >>/dev/null || exit 1

echo "* Copying pb headers from ${NANOPB_DIR} to ${OUTPUT_DIR}"
cp "${NANOPB_DIR}"/pb*{.h,.c} "${OUTPUT_DIR}"