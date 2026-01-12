#!/usr/bin/bash
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd -P)
PLANNING_DIR="$(cd "${SCRIPT_DIR}/.." && pwd -P)"
TOP_DIR="$(cd "${SCRIPT_DIR}/../../../.." && pwd -P)"
source ${TOP_DIR}/scripts/common.bashrc

export CYBER_PATH="${TOP_DIR}/cyber_release"

bazel_paths=(${TOP_DIR}/.cache/bazel/*/execroot/apollo/bazel-out/k8-opt/bin)
bazel_bin_path="${bazel_paths[0]}"
# 新版app仓库编译产物路径变更为.../base/...，优先使用该路径
for path in "${bazel_paths[@]}"; do
    if [[ "$path" == *"bazel/base/"* ]]; then
        bazel_bin_path="$path"
        break
    fi
done

worldview_path="/usr/local/share/worldview"
third_party_path=${TOP_DIR}/third_party

# cyber dependencies
pathprepend ${bazel_bin_path}/cyber_release
pathprepend ${bazel_bin_path}/cyber_release/mainboard
pathprepend ${bazel_bin_path}/cyber_release/cyber/cyber_recorder
pathprepend ${bazel_bin_path}/cyber_release/cyber/cyber_monitor
pathprepend ${bazel_bin_path}/cyber_release/cyber/cyber_channel
pathprepend ${CYBER_PATH}/lib/x86_64 LD_LIBRARY_PATH
pathprepend ${bazel_bin_path}/cyber LD_LIBRARY_PATH
pathappend ${worldview_path}/platform/libs LD_LIBRARY_PATH

# planning dependencies
pathprepend ${PLANNING_DIR}/libs/x86 LD_LIBRARY_PATH
pathprepend ${bazel_bin_path}/modules/simulator/deterministic_scheduler LD_LIBRARY_PATH
pathprepend ${third_party_path}/absl/lib/x86_64 LD_LIBRARY_PATH
pathprepend ${third_party_path}/ipopt/lib/x86_64 LD_LIBRARY_PATH
pathprepend ${third_party_path}/openblas/lib/x86_64 LD_LIBRARY_PATH
pathprepend ${third_party_path}/boost/boost/lib LD_LIBRARY_PATH
pathprepend ${third_party_path}/jsoncpp/lib/x86_64 LD_LIBRARY_PATH
pathprepend ${third_party_path}/osqp/lib/x86_64 LD_LIBRARY_PATH

#CYBER env setup
# ip_addr=$(hostname -I | awk '{print $2}')
# if [ -z "${ip_addr}" ]; then
#     export CYBER_IP=$(hostname -I | awk '{print $1}')
# else
#     export CYBER_IP=${ip_addr}
# fi
# echo "CYBER_IP=${CYBER_IP}"

if [ ! -d "/tmp/fillback_logs" ]; then
    mkdir /tmp/fillback_logs
fi

export GLOG_log_dir="/tmp/fillback_logs"
export GLOG_alsologtostderr=1
export GLOG_colorlogtostderr=1
export GLOG_minloglevel=0

source ${TOP_DIR}/scripts/cyber_tools_auto_complete.bash
