load("@rules_cc//cc:defs.bzl","cc_binary","cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

config_setting(name="x86_mode", values={"cpu": "k8"})
config_setting(name="arm_mode", values={"cpu": "aarch64"})

filegroup(
    name = "plan_common_srcs",
    srcs = glob(["**/*.cpp","**/*.cc"],
        exclude = ["**/*_test.cpp",
                   "**/*_test.cc",
                   "**/*_bm.cc",
                   "**/*plot*.cc",
                   "multi_timer_util.cc",
                   "inference/ad_byd_inference/*.cpp",
                ])
)

filegroup(
    name = "plan_common_hdrs",
    srcs = glob(["**/*.h","**/*.hpp"],
        exclude = ["**/*_plot_*.h",
                   "multi_timer_util.h",
                   "inference/ad_byd_inference/*.h",
        ]),
)

cc_library(
    name = "plan_common",
    srcs = ["plan_common_srcs"],
    hdrs = ["plan_common_hdrs"],
    deps = [
        #msg
        "//modules/msg/st_msgs:planning_result_cc_proto",
        "//modules/msg/st_msgs:planning_debug_frame_cc_proto",
        "//modules/msg/st_msgs:map_info_cc_proto",
        "//modules/msg/st_msgs:sm_behavior_cc_proto",
        "//modules/msg/localization_msgs:localization_info_cc_proto",
        "//modules/msg/drivers_msgs:canbus_uplink_cc_proto",
        "//modules/msg/prediction_msgs:pred_obj_info_v2_cc_proto",
        "//modules/msg/perception_msgs:dynamic_common_cc_proto",
        "//modules/msg/orin_msgs:routing_map_cc_proto",
        "//modules/msg/planning_msgs:plan_traj_info_cc_proto",
        "//modules/msg/orin_msgs:vmc_msgs_cc_proto",
        "//modules/msg/orin_msgs:map_event_cc_proto",
        "//modules/msg/state_machine_msgs:top_state_cc_proto",
        "//modules/cnoa_pnc/planning/proto:planning_cc_proto",
        #third_party
        "@absl//:absl",
        "@osqp//:osqp",
        "@eigen//:eigen",
        "@boost//:boost",
        "@cereal//:cereal",
        "@jsoncpp//:json-cpp",
        "@glog_gflags//:glog",
        "@glog_gflags//:gflags",
        "@com_google_googletest//:gtest",
        "//cyber_release",
    ],
    copts = [
        "-std=c++17",
        "-Wno-deprecated-declarations",
        "-Wno-sign-compare",
        "-Wno-unused-function",
        "-Wno-unused-value",
        "-Imodules/cnoa_pnc/planning",
        "-DMODULE_NAME=\\\"cnoa_plan\\\"",
    ] + select({
        ":arm_mode": ["-DPLATFORM_ARM"],
        ":x86_mode": ["-DPLATFORM_X86"],
    }),
    alwayslink = True,
)

cpplint()