#!/bin/bash
set -e

CUR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
source "${CUR_DIR}"/shflags

DEFINE_boolean 'enable_scenario_test' false "enable scenarios test"

# parse the command-line
FLAGS "$@" || exit $?
eval set -- "${FLAGS_ARGV}"

# CPPLINT
# bazel test --config=cpplint //modules/cnoa_pnc/planning/...
bazel test --config=cpplint //modules/cnoa_pnc/planning/decider:selector_srcs_cpplint
bazel test --config=cpplint //modules/cnoa_pnc/planning/decider:selector_hdrs_cpplint

# Selector test
bazel query "attr(tags, '\\bselector\\b',//modules/cnoa_pnc/planning/decider/...)" | xargs bazel test

# Scenarions
if [[ ${FLAGS_enable_scenario_test} -eq ${FLAGS_TRUE} ]]; then
    source "${CUR_DIR}"/scenario_test.sh
    scenario_test
fi
