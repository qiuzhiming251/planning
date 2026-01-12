#!/bin/bash

#run in docker

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
#source "${TOP_DIR}/scripts/apollo.bashrc"
set -e

# cp from `apollo.bashrc`

function _file_ext() {
    local filename="$(basename $1)"
    local actual_ext="${filename##*.}"
    if [[ "${actual_ext}" == "${filename}" ]]; then
        actual_ext=""
    fi
    echo "${actual_ext}"
}

function _proto_ext() {
    if [[ "$(_file_ext $1)" == "proto" ]]; then
        return 0
    else
        return 1
    fi
}

function _c_family_ext() {
    local actual_ext
    actual_ext="$(_file_ext $1)"
    for ext in "h" "hh" "hxx" "hpp" "cxx" "cc" "cpp" "cu"; do
        if [[ "${ext}" == "${actual_ext}" ]]; then
            return 0
        fi
    done
    return 1
}

function _shell_ext() {
    local actual_ext
    actual_ext="$(_file_ext $1)"
    for ext in "sh" "bash" "zsh" "bashrc"; do
        if [[ "${ext}" == "${actual_ext}" ]]; then
            return 0
        fi
    done
    return 1
}

function _bazel_ext() {
    local actual_ext
    actual_ext="$(_file_ext $1)"
    for ext in "BUILD" "bzl"; do
        if [[ "${ext}" == "${actual_ext}" ]]; then
            return 0
        fi
    done
    return 1
}

FILE_CHANGES=$(git diff --name-only HEAD~)
for f in $FILE_CHANGES; do
    if [[ -f "$f" ]]; then
        if _c_family_ext "$f" || _proto_ext "$f"; then
            clang-format -i "$f"
            echo "format $f"
        elif _shell_ext "$f"; then
            shfmt -i 4 -w "$f"
            echo "format $f"
        elif _bazel_ext "$f"; then
            buildifier -mode fix -lint fix "$f"
        fi
    fi
done
