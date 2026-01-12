load("@rules_cc//cc:defs.bzl","cc_binary","cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

config_setting(name="x86_mode", values={"cpu": "k8"})

COPTS = select({
   "//conditions:default": [
        "-Wno-deprecated-declarations",
        "-Wno-sign-compare",
        "-Wno-unused-function",
        "-Wno-unused-value",
        "-Wno-unused-variable",
        "-Wno-unused-but-set-variable",
        "-std=c++17",
    ], 
})

filegroup(
    name = "logsim_hdrs",
    srcs = glob(["**/*.h"])
)

filegroup(
    name = "logsim_srcs",
    srcs = glob(["**/*.cpp","**/*.cc"],
    exclude = ["**/*_test.cpp", "main.cpp"]),
)

# cc_library(
#     name = "supported_cyber_types_lib",
#     srcs = ["msg/cyber/supported_types_cyber.cpp"],
#     hdrs = ["logsim_hdrs"],
#     includes = ["."],
#     copts = COPTS,
#     deps = [
#         "//cyber_release",
#         "@glog_gflags//:glog",
#         "@glog_gflags//:gflags",
#         #msg
#         "//modules/msg/st_msgs:planning_result_cc_proto",
#         "//modules/msg/st_msgs:planning_debug_frame_cc_proto",
#         "//modules/msg/st_msgs:map_info_cc_proto",
#         "//modules/msg/st_msgs:sm_behavior_cc_proto",
#         "//modules/msg/localization_msgs:localization_info_cc_proto",
#         "//modules/msg/drivers_msgs:canbus_uplink_cc_proto",
#         "//modules/msg/drivers_msgs:veh_info_cc_proto",
#         "//modules/msg/prediction_msgs:pred_obj_info_v2_cc_proto",
#         "//modules/msg/perception_msgs:dynamic_common_cc_proto",
#         "//modules/msg/orin_msgs:routing_map_cc_proto",
#         "//modules/msg/planning_msgs:plan_traj_info_cc_proto",
#         "//modules/msg/orin_msgs:vmc_msgs_cc_proto",
#         "//modules/msg/orin_msgs:map_event_cc_proto",
#         "//modules/msg/state_machine_msgs:top_state_cc_proto",
#         "//modules/msg/environment_model_msgs:local_map_info_cc_proto",
#         "//modules/cnoa_pnc/planning/proto:planning_cc_proto",
#     ],
# )

# cc_binary(
#     name = "libsupported_cyber_types.so",
#     srcs = [],
#     linkshared = True,
#     linkstatic = False,
#     deps = [
#         ":supported_cyber_types_lib",
#     ],
#     copts = COPTS,
#     linkopts = ["-ldl"],
# )

cc_library(
    name = "logsim_fillback_lib",
    srcs = ["logsim_srcs"],
    hdrs = ["logsim_hdrs"],
    includes = ["."],
    copts = COPTS,
    deps = [
        "//cyber_release",
        #":supported_cyber_types_lib",
        #thirdparty
        "@jsoncpp//:json-cpp",
        "@eigen//:eigen",
        #msg
        "//modules/msg/st_msgs:planning_result_cc_proto",
        "//modules/msg/st_msgs:planning_debug_frame_cc_proto",
        "//modules/msg/st_msgs:map_info_cc_proto",
        "//modules/msg/st_msgs:sm_behavior_cc_proto",
        "//modules/msg/localization_msgs:localization_info_cc_proto",
        "//modules/msg/drivers_msgs:canbus_uplink_cc_proto",
        "//modules/msg/drivers_msgs:veh_info_cc_proto",
        "//modules/msg/prediction_msgs:pred_obj_info_v2_cc_proto",
        "//modules/msg/perception_msgs:dynamic_common_cc_proto",
        "//modules/msg/orin_msgs:routing_map_cc_proto",
        "//modules/msg/planning_msgs:plan_traj_info_cc_proto",
        "//modules/msg/orin_msgs:vmc_msgs_cc_proto",
        "//modules/msg/orin_msgs:map_event_cc_proto",
        "//modules/msg/state_machine_msgs:top_state_cc_proto",
        "//modules/msg/environment_model_msgs:local_map_info_cc_proto",
        "//modules/cnoa_pnc/planning/proto:planning_cc_proto",
    ],
)

cc_binary(
    name = "logsim_fillback",
    srcs = ["main.cpp"],

    deps = [
        ":logsim_fillback_lib",
        #":supported_cyber_types_lib",
        "//cyber_release",
    ],
    copts = COPTS,
    linkopts = ["-ldl"],
)