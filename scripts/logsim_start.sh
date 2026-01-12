#!/usr/bin/bash

# SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
# echo "$SCRIPT_DIR"

program=planning
LOG_DIR="/apollo/data/log"
if [ $# -eq 0 ]; then
    log_file="$LOG_DIR/$program.log"
else
    log_file="$LOG_DIR/$1"
fi
mainboardBin="/apollo/bazel-bin/cyber_release/mainboard"
planningDag="/apollo/modules/cnoa_pnc/planning/dag/planning_logsim_batch_fillback.dag"
libplan_component="/apollo/bazel-bin/modules/cnoa_pnc/planning/libplan_component.so"

# create log dir
mkdir -p "$LOG_DIR"

# check mainboard
if [ ! -f "$mainboardBin" ]; then
    /apollo/main.sh build_cyber
fi

# check plan_component
if [ ! -f "$libplan_component" ]; then
    bash /apollo/modules/cnoa_pnc/planning/scripts/build.sh -p x86 -m plan_component --dbg -j 10
fi

# start planning node
source /apollo/scripts/cyber_setup.bash
start_planning_command="$mainboardBin -d $planningDag"
nohup bash -c "cd /apollo && $start_planning_command" >"$log_file" 2>&1 &
