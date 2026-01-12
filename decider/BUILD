load("@rules_cc//cc:defs.bzl","cc_binary","cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("@rules_python//python:defs.bzl", "py_test")

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
    name = "decision_manager_srcs",
    srcs = glob(["decision_manager/**/*.cc"],
        exclude = [ "decision_manager/**/*_test.cc"])
)

filegroup(
    name = "decision_manager_hdrs",
    srcs = glob(["decision_manager/**/*.h"])
)

cc_library(
    name = "decision_manager",
    srcs = ["decision_manager_srcs"],
    hdrs = ["decision_manager_hdrs"],
    deps = [
        #lib
        "//modules/cnoa_pnc/planning/planner:planner_manager",
        "//modules/cnoa_pnc/planning/object_manager:object_manager",
        "//modules/cnoa_pnc/planning/router:router"
    ],
    copts = COPTS,
    alwayslink = True,
) 

# scene_manager
filegroup(
    name = "scene_manager_srcs",
    srcs = glob(["scene_manager/**/*.cc"],
        exclude = [ "scene_manager/**/*_test.cc"])
)

filegroup(
    name = "scene_manager_hdrs",
    srcs = glob(["scene_manager/**/*.h"])
)

cc_library(
    name = "scene_manager",
    srcs = ["scene_manager_srcs"],
    hdrs = ["scene_manager_hdrs"],
    deps = [
        #lib
        "//modules/cnoa_pnc/planning/object_manager:object_manager",
        "//modules/cnoa_pnc/planning/router:router"
    ],
    copts = COPTS,
    alwayslink = True,
) 

# scheduler
filegroup(
    name = "scheduler_srcs",
    srcs = glob(["scheduler/**/*.cc"],
        exclude = [ "scheduler/**/*_test.cc"])
)

filegroup(
    name = "scheduler_hdrs",
    srcs = glob(["scheduler/**/*.h"])
)

cc_library(
    name = "scheduler",
    srcs = ["scheduler_srcs"],
    hdrs = ["scheduler_hdrs"],
    deps = [
        #msg
        "//modules/cnoa_pnc/planning/proto:planning_cc_proto",
        #lib
        "//modules/cnoa_pnc/planning/object_manager:object_manager",
        "//modules/cnoa_pnc/planning/router:router"
    ],
    copts = COPTS,
    alwayslink = True,
) 

# initializer
filegroup(
    name = "initializer_srcs",
    srcs = glob(["initializer/**/*.cc"],
        exclude = [ 
            "initializer/**/*_test.cc",
            "initializer/**/*_bm.cc"
        ])
)

filegroup(
    name = "initializer_hdrs",
    srcs = glob(["initializer/**/*.h"])
)

cc_library(
    name = "initializer",
    srcs = ["initializer_srcs"],
    hdrs = ["initializer_hdrs"],
    deps = [
        ":scheduler",
        ":decision_manager",
        #lib
        "//modules/cnoa_pnc/planning/object_manager:object_manager",
        "//modules/cnoa_pnc/planning/router:router",
        "//modules/cnoa_pnc/planning/alternative_gaming:alternative_gaming",
        #third party
        "@boost//:boost"
    ],
    copts = COPTS,
    alwayslink = True,
) 

# selector
filegroup(
    name = "selector_srcs",
    srcs = glob(["selector/**/*.cc"],
        exclude = [ "selector/**/*_test.cc"])
)

filegroup(
    name = "selector_hdrs",
    srcs = glob(["selector/**/*.h"])
)


cc_library(
    name = "selector",
    srcs = ["selector_srcs"],
    hdrs = ["selector_hdrs"],
    deps = [
        #lib
        "//modules/cnoa_pnc/planning/object_manager:object_manager",
        "//modules/cnoa_pnc/planning/router:router"
    ],
    copts = COPTS,
    alwayslink = True,
) 

cc_test(
    name = "selector_test",
    srcs = ["selector/selector_test.cc","selector_srcs"],
    deps = [
        ":selector",
        "@absl//:absl",
        "@osqp//:osqp",
        "@ipopt//:ipopt",
        "@eigen//:eigen",
        "@cereal//:cereal",
        "@jsoncpp//:json-cpp",
        "@glog_gflags//:glog",
        "@glog_gflags//:gflags",
        "@com_google_protobuf//:protobuf",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main"
    ],
    tags=["selector",],
    copts = COPTS,
)

cc_test(
    name = "cost_feature_util_test",
    srcs = ["selector/cost_feature_util_test.cc","selector_srcs"],
    deps = [
        ":selector",
        "@absl//:absl",
        "@osqp//:osqp",
        "@ipopt//:ipopt",
        "@eigen//:eigen",
        "@cereal//:cereal",
        "@jsoncpp//:json-cpp",
        "@glog_gflags//:glog",
        "@glog_gflags//:gflags",
        "@com_google_protobuf//:protobuf",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main"
    ],
    tags=["selector",],
    copts = COPTS,
)

cc_test(
    name = "ensure_lc_feasibility_test",
    srcs = ["selector/ensure_lc_feasibility_test.cc","selector_srcs"],
    deps = [
        ":selector",
        "@absl//:absl",
        "@osqp//:osqp",
        "@ipopt//:ipopt",
        "@eigen//:eigen",
        "@cereal//:cereal",
        "@jsoncpp//:json-cpp",
        "@glog_gflags//:glog",
        "@glog_gflags//:gflags",
        "@com_google_protobuf//:protobuf",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main"
    ],
    tags=["selector",],
    copts = COPTS,
)

cc_test(
    name = "speed_cost_estimator_test",
    srcs = ["selector/speed_cost_estimator_test.cc"],
    deps = [
        ":selector",
        "@absl//:absl",
        "@osqp//:osqp",
        "@ipopt//:ipopt",
        "@eigen//:eigen",
        "@cereal//:cereal",
        "@jsoncpp//:json-cpp",
        "@glog_gflags//:glog",
        "@glog_gflags//:gflags",
        "@com_google_protobuf//:protobuf",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main"
    ],
    tags=["selector",],
    copts = COPTS,
)

cpplint()
