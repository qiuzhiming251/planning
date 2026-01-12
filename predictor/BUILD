load("@rules_cc//cc:defs.bzl","cc_binary","cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

config_setting(name="x86_mode", values={"cpu": "k8"})
config_setting(name="arm_mode", values={"cpu": "aarch64"})

filegroup(
    name = "predictor_srcs",
    srcs = glob(["**/*.cpp","**/*.cc"])
)

filegroup(
    name = "predictor_hdrs",
    srcs = glob(["**/*.h","**/*.hpp"]),
)

cc_library(
    name = "predictor",
    srcs = ["predictor_srcs"],
    hdrs = ["predictor_hdrs"],
    deps = [
        #msg
        "//modules/cnoa_pnc/planning/proto:planning_cc_proto",
        #lib
        "//modules/cnoa_pnc/planning/plan_common:plan_common",
    ],
    copts = [
        "-std=c++17",
        "-Wno-deprecated-declarations",
        "-Wno-sign-compare",
        "-Wno-unused-function",
        "-Wno-unused-value",
        "-Imodules/cnoa_pnc/planning",
        "-DMODULE_NAME=\\\"cnoa_plan\\\"",
    ]  + select({
        ":arm_mode": ["-DPLATFORM_ARM"],
        ":x86_mode": ["-DPLATFORM_X86"],
    }),
    alwayslink = True,
) 

cpplint()
