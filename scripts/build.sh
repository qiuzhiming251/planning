#!/usr/bin/env bash
set -e

ARTIFACTS_DIR="/apollo/modules/cnoa_pnc/cnoa_plan"
BUILD_CYBER="--config=release_build"

declare -A build_command_map=(
    ["x86"]="build"
    ["arm"]="build_orinx2b"
    ["orinx"]="build_orinx2b"
    ["orinn"]="build_orin3"
    ["j6m"]="build_j6m"
    ["vcpb"]="build_vcpb"
)

declare -A platform_map=(
    ["x86"]="x86"
    ["arm"]="arm"
    ["orinx"]="arm"
    ["orinn"]="arm"
    ["j6m"]="arm"
    ["vcpb"]="arm"
)

# List of all library targets to build
declare -A libraries=(
    ["plan_common"]="modules/cnoa_pnc/planning/plan_common:plan_common"
    ["predictor"]="modules/cnoa_pnc/planning/predictor:predictor"
    ["router"]="modules/cnoa_pnc/planning/router:router"
    ["object_manager"]="modules/cnoa_pnc/planning/object_manager:object_manager"
    ["decision_manager"]="modules/cnoa_pnc/planning/decider:decision_manager"
    ["scene_manager"]="modules/cnoa_pnc/planning/decider:scene_manager"
    ["scheduler"]="modules/cnoa_pnc/planning/decider:scheduler"
    ["initializer"]="modules/cnoa_pnc/planning/decider:initializer"
    ["selector"]="modules/cnoa_pnc/planning/decider:selector"
    ["alternative_gaming"]="modules/cnoa_pnc/planning/alternative_gaming:alternative_gaming"
    ["planner_manager"]="modules/cnoa_pnc/planning/planner:planner_manager"
    ["speed_optimizer"]="modules/cnoa_pnc/planning/planner:speed_optimizer"
    ["trajectory_optimizer"]="modules/cnoa_pnc/planning/planner:trajectory_optimizer"
    ["msg_adapter"]="modules/cnoa_pnc/planning/node_manager:msg_adapter"
    ["task_runner"]="modules/cnoa_pnc/planning/node_manager:task_runner"
    ["node_manager"]="modules/cnoa_pnc/planning/node_manager:node_manager"
    ["plan_component"]="modules/cnoa_pnc/planning:libplan_component.so"
    ["acc"]="modules/cnoa_pnc/planning/acc:acc"
    ["logsim"]="modules/cnoa_pnc/planning/offboard/logsim_fillback:logsim_fillback"
)

# Function to display usage information
usage() {
    echo "Usage: $0 -m <module_name> -p <platform> (--dbg / --release)"
    echo "  -m <module_name> : Name of the module to build (e.g.,
                            plan_common,
                            predictor,
                            router,
                            object_manager,
                            decision_manager,
                            scene_manager,
                            scheduler,
                            initializer,
                            selector,
                            alternative_gaming,
                            planner_manager,
                            speed_optimizer,
                            trajectory_optimizer,
                            msg_adapter,
                            task_runner,
                            node_manager,
                            plan_component,
                            acc,
                            logsim,
                            all)"
    echo "  -p <platform>    : Target platform (e.g., x86, arm, orinn, orinx, j6m, vcpb)"
    echo "  -j <jobs>        : Number of parallel jobs (default: BAZEL_BUILD_THREAD_NUM)"
    echo "  --dbg / --release : (optional) Compile mode, default release"
    echo "  --asan            : (optional) Enable AddressSanitizer"
    echo "  --serialize       : (optional) Enable serialize"
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
    local module_name="$2"
    local bazel_platform=$(bazel_platform_path "${platform}")
    # lib name
    local module_lib_name="lib${module_name}.so"
    local module_path=$(echo "${libraries[$module_name]}" | sed 's/:.*//')

    local artifacts_dir="${ARTIFACTS_DIR}/libs/${platform}"
    local lib_path="/apollo/bazel-out/${bazel_platform}-opt/bin/${module_path}/${module_lib_name}"

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

# Determine the Bazel target based on the module name and platform
build_libraries() {
    local platform=$1
    local module_name=$2
    local compile_mode=$3
    local asan_enable=$4
    echo "Build thread num: ${BAZEL_BUILD_THREAD_NUM}"
    echo "Building module: $module_name"
    echo "Bazel target: ${libraries[$module_name]}"
    local mode_options=()
    case $compile_mode in
    release)
        echo "compile mode: release"
        ;;
    dbg)
        echo "compile mode: dbg"
        if [ "$platform" == "x86" ]; then
            mode_options=(--cxxopt='-g' --cxxopt='-Og' --define=debug_x86=1)
        else
            mode_options=(--cxxopt='-g' --cxxopt='-Og')
        fi
        ;;
    *)
        echo "Unsupported compile mode: $compile_mode"
        exit 1
        ;;
    esac

    if [ "$asan_enabled" = true ]; then
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

    echo "mode option ${mode_options[@]} ${libraries[$module_name]}"

    local build_cmd=${build_command_map[$target]}
    cd /apollo
    ./main.sh "$build_cmd" --jobs="${BAZEL_BUILD_THREAD_NUM}" "${mode_options[@]}" "${libraries[$module_name]}" "${BUILD_CYBER}"

    echo "Copy planning libs..."
    if [ "$module_name" != "release" ]; then
        prepare_platform_lib_artifacts "$platform" "$module_name"
    fi
}

# Parse command line arguments
compile_mode="release"
ARGS=$(getopt -o m:p:j: --long dbg,release,asan,serialize,build_cyber -- "$@")
if [ $? -ne 0 ]; then
    usage
fi

eval set -- "$ARGS"

# 解析参数
while true; do
    case "$1" in
    -m)
        module_name="$2"
        shift 2
        ;;
    -p)
        target="$2"
        shift 2
        ;;
    -j)
        jobs_arg="$2"
        shift 2
        ;;
    --dbg)
        compile_mode="dbg"
        shift
        ;;
    --release)
        compile_mode="release"
        shift
        ;;
    --asan)
        asan_enabled=true
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
    *)
        usage
        ;;
    esac
done

if [[ -z "${build_command_map[$target]}" ]]; then
    echo "Error: Unsupported target '$target'"
    echo "Supported targets: ${!build_command_map[@]}"
    exit 1
fi

platform="${platform_map[$target]}"

echo "Target chip: $target"
echo "Mapped platform: $platform"

if [[ -n "$jobs_arg" ]]; then
    if [[ "$jobs_arg" =~ ^[0-9]+$ ]]; then
        BAZEL_BUILD_THREAD_NUM=$jobs_arg
    else
        echo "Invalid jobs number: $jobs_arg, must be an integer"
        exit 1
    fi
fi

# Check if both module name and platform are provided
if [ -z "$module_name" ] || [ -z "$platform" ]; then
    usage
fi

if [[ -v libraries["$module_name"] || "$module_name" == "all" ]]; then
    echo "supported module: $module_name"
else
    echo "unsupported module: $module_name"
    usage
    exit 1
fi

if [ "$module_name" == "all" ]; then
    echo "Building all modules..."
    for module in "${!libraries[@]}"; do
        echo "Building $module..."
        build_libraries "$platform" "$module" "$compile_mode"
    done
else
    echo "Building $module_name..."
    build_libraries "$platform" "$module_name" "$compile_mode"
fi

echo "$module_name built successfully for $platform."

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

echo "update dvc..."
update_dvc
