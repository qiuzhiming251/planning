load("@rules_cc//cc:defs.bzl","cc_binary","cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

config_setting(name="x86_mode", values={"cpu": "k8"})
config_setting(name="arm_mode", values={"cpu": "aarch64"})

COPTS = select({
    "//modules/cnoa_pnc/planning:debug_x86": [
        "-Wno-deprecated-declarations",
        "-Wno-sign-compare",
        "-Wno-unused-function",
        "-Wno-unused-value",
        "-Wno-unused-variable",
        "-Wno-unused-but-set-variable",
        "-Imodules/cnoa_pnc/planning",
        "-std=c++17",
        "-DDEBUG_X86_ENABLED=1",
        "-DMODULE_NAME=\\\"cnoa_plan\\\"",
    ],
   "//conditions:default": [
        "-Wno-deprecated-declarations",
        "-Wno-sign-compare",
        "-Wno-unused-function",
        "-Wno-unused-value",
        "-Wno-unused-variable",
        "-Wno-unused-but-set-variable",
        "-Imodules/cnoa_pnc/planning",
        "-std=c++17",
        "-DMODULE_NAME=\\\"cnoa_plan\\\"",
    ],
}) + select({
    ":arm_mode": ["-DPLATFORM_ARM"],
    ":x86_mode": ["-DPLATFORM_X86"],
})

# msg_adapter
filegroup(
    name = "msg_adapter_srcs",
    srcs = glob(["msg_adapter/**/*.cpp"])
)

filegroup(
    name = "msg_adapter_hdrs",
    srcs = glob(["msg_adapter/**/*.h"]),
)

cc_library(
    name = "msg_adapter",
    srcs = ["msg_adapter_srcs"],
    hdrs = ["msg_adapter_hdrs"],

    deps = [
        "//cyber_release",
        #lib
        "//modules/cnoa_pnc/planning/plan_common:plan_common", 
        "//modules/simulator/deterministic_scheduler:deterministic_scheduler",
    ],
    copts = COPTS,
    alwayslink = True,
) 

#task_runner
filegroup(
    name = "task_runner_srcs",
    srcs = glob(["task_runner/**/*.cpp","task_runner/**/*.cc"],
        exclude = ["task_runner/**/*_test.cc",
                   "task_runner/**/*_bm.cc",
                   "task_runner/**/*_trace.cc"
    ])
)

filegroup(
    name = "task_runner_hdrs",
    srcs = glob([
        "task_runner/**/*.h",
        "task_runner/city_planner/dump_result_util.h",
    ]),
)

cc_library(
    name = "task_runner",
    srcs = ["task_runner_srcs"],
    hdrs = ["task_runner_hdrs"],

    deps = [
        "//cyber_release",
        #msg
        "//modules/cnoa_pnc/planning/proto:planning_cc_proto",
        "//modules/msg/orin_msgs:map_event_cc_proto",
        #lib
        ":msg_adapter", 
        "//modules/cnoa_pnc/planning/router:router",
        "//modules/cnoa_pnc/planning/object_manager:object_manager",

        "//modules/cnoa_pnc/planning/planner:planner_manager",
        "//modules/cnoa_pnc/planning/planner:trajectory_optimizer",
        "//modules/cnoa_pnc/planning/planner:speed_optimizer",
        
        "//modules/cnoa_pnc/planning/decider:decision_manager",
        "//modules/cnoa_pnc/planning/decider:initializer",
        "//modules/cnoa_pnc/planning/decider:scene_manager",
        "//modules/cnoa_pnc/planning/decider:scheduler",
        "//modules/cnoa_pnc/planning/decider:selector",
        "//modules/cnoa_pnc/planning/acc:acc",
    ],
    copts = COPTS,
    alwayslink = True,
) 

#node_manager
cc_library(
    name = "node_manager",
    srcs = ["byd_planning_component.cpp"],
    hdrs = ["byd_planning_component.h", "version_header"],
    includes = ["."],
    deps = [
        #lib
        ":task_runner", 
        "//modules/simulator/deterministic_scheduler:deterministic_scheduler",
    ],
    copts = COPTS,
    alwayslink = True,
) 

genrule(
    name = "git_hash",
    outs = ["git_hash.txt"],
    cmd = """
        cd /apollo/modules/cnoa_pnc/planning && \
        echo "$$(git rev-parse --abbrev-ref HEAD)-$$(git log -1 --pretty=format:%h)" > /apollo/$@ && \
        cd /apollo
    """,
)

genrule(
    name = "version_header",
    srcs = [":git_hash"],
    outs = ["version.h"],
    cmd = """
        echo '#pragma once \n#define GIT_COMMIT_HASH \"'$$(cat $(location :git_hash))'\"' > $@ 
        cp $@ /apollo/modules/cnoa_pnc/planning/node_manager/
    """,
)

cpplint()