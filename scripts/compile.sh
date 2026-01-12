#!/usr/bin/env bash
set -e

#==============================================================
#                  Constants & Defaults
#==============================================================
ARTIFACTS_DIR="/apollo/modules/cnoa_pnc/cnoa_plan"
BUILD_CYBER="--config=release_build"
COMPILE_MODE="release"
ASAN_ENABLED=false

declare -A BUILD_COMMAND_MAP=(
    ["x86"]="build"
    ["arm"]="build_orinx2b"
    ["orinx"]="build_orinx2b"
    ["orinn"]="build_orin3"
    ["j6m"]="build_j6m"
    ["vcpb"]="build_vcpb"
)

declare -A PLATFORM_MAP=(
    ["x86"]="x86"
    ["arm"]="arm"
    ["orinx"]="arm"
    ["orinn"]="arm"
    ["j6m"]="arm"
    ["vcpb"]="arm"
)

#==============================================================
#                  Helper Functions
#==============================================================
usage() {
    echo "Usage: $0 -p <platform> (--dbg / --release)"
    # TODO: delete
    echo "  -m                : keep cicd work."
    echo "  -p <platform>     : Target platform (e.g., x86, arm, orinn, orinx, j6m, vcpb)"
    echo "  -j <jobs>         : Number of parallel jobs (default: BAZEL_BUILD_THREAD_NUM)"
    echo "  --dbg             : (optional) Compile with debug symbols"
    echo "  --release         : (optional) Compile with optimizations (default)"
    echo "  --asan            : (optional) Enable AddressSanitizer"
    echo "  --serialize       : (optional) Enable serialize"
    echo "  --build_cyber     : Use repo_release_build config"
    exit 1
}

bazel_platform_path() {
    local platform="$1"
    if [ "${platform}" == "x86" ]; then
        echo "k8"
    elif [ "${platform}" == "arm" ]; then
        echo "aarch64"
    else
        echo "Unsupported platform: ${platform}"
        exit 1
    fi
}

prepare_platform_lib_artifacts() {
    local platform="$1"
    local bazel_platform=$(bazel_platform_path "${platform}")
    local artifacts_dir="${ARTIFACTS_DIR}/libs/${platform}"
    local lib_path="/apollo/bazel-out/${bazel_platform}-opt/bin/modules/cnoa_pnc/planning/libplan_component.so"

    if [ -f "${lib_path}" ]; then
        mkdir -p "${artifacts_dir}"
        sudo cp "${lib_path}" "${artifacts_dir}"
        sudo chown ${USER}:${USER} ${artifacts_dir}/*
        echo "Prepare lib artifacts: ${lib_path}"
    else
        echo "No lib artifacts found: ${lib_path}"
        exit 1
    fi
}

build_libraries() {
    local platform=$1
    local mode=$2
    echo "Build thread num: ${BAZEL_BUILD_THREAD_NUM}"
    local mode_options=()
    case $mode in
    release) echo "compile mode: release" ;;
    dbg)
        echo "compile mode: dbg"
        if [ "$platform" == "x86" ]; then
            mode_options=(--cxxopt='-g' --cxxopt='-Og' --define=debug_x86=1)
        else
            mode_options=(--cxxopt='-g' --cxxopt='-Og')
        fi
        ;;
    *)
        echo "Unsupported compile mode: $mode"
        exit 1
        ;;
    esac

    if [ "$ASAN_ENABLED" = true ]; then
        echo "Enabling AddressSanitizer"
        for i in "${!mode_options[@]}"; do
            if [[ "${mode_options[i]}" == "--cxxopt=-Og" ]]; then
                mode_options[i]="--cxxopt=-O1"
            fi
        done
        mode_options+=(
            --config=asan
            --copt=-fno-omit-frame-pointer
            --copt=-fsanitize-recover=address
        )
        mode_options=("${mode_options[@]//--cxxopt=-Og/}")
    fi

    if [ "$serialize_enabled" = true ]; then
        echo "Enabling serialize"
        mode_options+=("--cxxopt=-DSERIALIZE_MACRO")
    fi

    echo "mode option ${mode_options[@]}"

    local build_cmd=${BUILD_COMMAND_MAP[$target]}
    cd /apollo
    ./main.sh "$build_cmd" --jobs="${BAZEL_BUILD_THREAD_NUM}" "${mode_options[@]}" modules/cnoa_pnc/planning:libplan_component.so "${BUILD_CYBER}"

    echo "Copy planning libs..."
    prepare_platform_lib_artifacts "$platform"
}

update_dvc() {
    if [ -d "${ARTIFACTS_DIR}/libs/${platform}" ]; then
        pushd "${ARTIFACTS_DIR}"
        dvc add libs/${platform}
        popd
        # dvc push "${ARTIFACTS_DIR}/libs/${platform}"
        # git add "${ARTIFACTS_DIR}/libs/${platform}.dvc"
        echo "Update artifacts dvc: ${ARTIFACTS_DIR}/libs/${platform}.dvc"
    else
        echo "No artifacts found: ${ARTIFACTS_DIR}/libs/${platform}"
    fi
}

#==============================================================
#                  Parse Arguments
#==============================================================
ARGS=$(getopt -o m:p:j: --long dbg,release,asan,serialize,build_cyber -- "$@")
if [ $? -ne 0 ]; then
    usage
fi

eval set -- "$ARGS"

while true; do
    case "$1" in
    # TODO: delete
    -m) shift 2 ;;
    -p)
        target="$2"
        shift 2
        ;;
    -j)
        jobs_arg="$2"
        shift 2
        ;;
    --dbg)
        COMPILE_MODE="dbg"
        shift
        ;;
    --release)
        COMPILE_MODE="release"
        shift
        ;;
    --asan)
        ASAN_ENABLED=true
        shift
        ;;
    --serialize)
        serialize_enabled=true
        shift
        ;;
    --build_cyber)
        BUILD_CYBER="--config=repo_release_build"
        shift
        ;;
    --)
        shift
        break
        ;;
    *) usage ;;
    esac
done

#==============================================================
#                  Validation & Setup
#==============================================================
if [[ -z "${BUILD_COMMAND_MAP[$target]}" ]]; then
    echo "Error: Unsupported target '$target'"
    echo "Supported targets: ${!BUILD_COMMAND_MAP[@]}"
    exit 1
fi

platform="${PLATFORM_MAP[$target]}"

if [ -z "$platform" ]; then
    usage
fi

if [[ -n "$jobs_arg" ]]; then
    if [[ "$jobs_arg" =~ ^[0-9]+$ ]]; then
        BAZEL_BUILD_THREAD_NUM=$jobs_arg
    else
        echo "Invalid jobs number: $jobs_arg, must be an integer"
        exit 1
    fi
fi

#==============================================================
#                  Main Execution
#==============================================================
build_libraries "$platform" "$COMPILE_MODE"

echo "built successfully for $target chip, $platform platform."

echo "update dvc..."
update_dvc
