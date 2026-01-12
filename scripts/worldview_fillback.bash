#!/usr/bin/bash
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd -P)
PLANNING_DIR="$(cd "${SCRIPT_DIR}/.." && pwd -P)"
TOP_DIR="$(cd "${SCRIPT_DIR}/../../../.." && pwd -P)"

source ${SCRIPT_DIR}/fillback_setup.bash
cd ${TOP_DIR}
mainboard -d ${PLANNING_DIR}/dag/planning_fillback.dag
