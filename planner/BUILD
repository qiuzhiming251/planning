load("@rules_cc//cc:defs.bzl","cc_binary","cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

config_setting(name="x86_mode", values={"cpu": "k8"})
config_setting(name="arm_mode", values={"cpu": "aarch64"})

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

filegroup(
    name = "planner_manager_srcs",
    srcs = glob(["planner_manager/**/*.cc"],
        exclude = ["planner_manager/**/*_test.cc",
                   "planner_manager/**/*_bm.cc",
        ])
)

filegroup(
    name = "planner_manager_hdrs",
    srcs = glob(["planner_manager/**/*.h"],
         exclude = ["planner_manager/**/*_bm.h",       
    ]),
)

cc_library(
    name = "planner_manager",
    srcs = ["planner_manager_srcs"],
    hdrs = ["planner_manager_hdrs"],
    deps = [
        #msg
        "//modules/cnoa_pnc/planning/proto:planning_cc_proto",
        #lib
        "//modules/cnoa_pnc/planning/object_manager:object_manager",
    ],
    copts = COPTS,
    alwayslink = True,
) 

#speed_optimizer
filegroup(
    name = "speed_optimizer_srcs",
    srcs = glob(["speed_optimizer/**/*.cc"],
        exclude = ["speed_optimizer/**/*_test.cc",
                   "speed_optimizer/**/*_bm.cc",
        ])
)

filegroup(
    name = "speed_optimizer_hdrs",
    srcs = glob(["speed_optimizer/**/*.h"],
        exclude = ["speed_optimizer/**/*_bm.h"
    ]),
)

cc_library(
    name = "speed_optimizer",
    srcs = ["speed_optimizer_srcs"],
    hdrs = ["speed_optimizer_hdrs"],
    deps = [
        #lib
        # "//modules/cnoa_pnc/planning/object_manager:libobject_manager",
        ":planner_manager",
        "//modules/cnoa_pnc/planning/alternative_gaming:alternative_gaming",
    ],
    copts = COPTS,
    alwayslink = True,
) 

#trajectory_optimizer

filegroup(
    name = "trajectory_optimizer_srcs",
    srcs = glob(["trajectory_optimizer/**/*.cc"],
        exclude = ["trajectory_optimizer/**/*_test.cc",
                   "trajectory_optimizer/**/*_bm.cc",
        ])
)

filegroup(
    name = "trajectory_optimizer_hdrs",
    srcs = glob(["trajectory_optimizer/**/*.h"],
        exclude = ["trajectory_optimizer/**/*_bm.h"
    ]),
)

cc_library(
    name = "trajectory_optimizer",
    srcs = ["trajectory_optimizer_srcs"],
    hdrs = ["trajectory_optimizer_hdrs"],
    deps = [
        #third_party
        "@ipopt//:ipopt",
        #lib
        # "//modules/cnoa_pnc/planning/object_manager:libobject_manager",
        ":planner_manager",
    ],
    copts = COPTS,
    alwayslink = True,
) 

cpplint()