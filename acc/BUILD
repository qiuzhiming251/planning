load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

config_setting(
    name = "x86_mode",
    values = {"cpu": "k8"},
)

config_setting(
    name = "arm_mode",
    values = {"cpu": "aarch64"},
)

COPTS = select({
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
})  + select({
    ":arm_mode": ["-DPLATFORM_ARM"],
    ":x86_mode": ["-DPLATFORM_X86"],
})

cc_library(
    name = "acc",
    srcs = ["acc_srcs"],
    hdrs = ["acc_hdrs"],
    copts = COPTS,
    deps = [
        #lib
        "//modules/cnoa_pnc/planning/router:router",
        "//modules/cnoa_pnc/planning/object_manager:object_manager",
        "//modules/cnoa_pnc/planning/planner:planner_manager",
        "//modules/cnoa_pnc/planning/planner:speed_optimizer",
        "//modules/cnoa_pnc/planning/plan_common:plan_common",
        "//modules/cnoa_pnc/planning/decider:scheduler",
        #msg
        "//modules/cnoa_pnc/planning/proto:planning_cc_proto",

        "@boost",
        "@com_google_protobuf//:protobuf",
        "@absl",
        "@jsoncpp//:json-cpp",
        "@glog_gflags//:glog",
        "@glog_gflags//:gflags",
    ],
    alwayslink = True,
)

filegroup(
    name = "acc_srcs",
    srcs = glob(
        [
            "*.cpp",
            "*.cc",
        ],
    ),
)

filegroup(
    name = "acc_hdrs",
    srcs = glob(
        [
            "*.hpp",
            "*.h",
        ],
    ),
)

cpplint()
