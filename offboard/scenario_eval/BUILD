load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_python//python:defs.bzl", "py_library","py_binary")
load("//tools:python_rules.bzl","py_proto_library")
package(default_visibility = ["//visibility:public"])

filegroup(
    name = "distribution",
    srcs = glob(["**"]),
    visibility = ["//python:__pkg__"],
)

py_binary(
    name = "intersection_evaluator",
    srcs = ["intersection_evaluator.py"],
    deps = [
        "//modules/cnoa_pnc/planning/offboard/scenario_eval/common:common",
        "//modules/msg/st_msgs:planning_debug_frame_py_pb2",
        "@bazel_tools//tools/python/runfiles",
    ],
    legacy_create_init = False,
    imports = [
        ".",
        "/apollo/bazel-bin",
    ],
)


py_binary(
    name = "back_and_forth_evaluator",
    srcs = ["back_and_forth_evaluator.py"],
    deps = [
        "//modules/cnoa_pnc/planning/offboard/scenario_eval/common:common",
        "//modules/msg/st_msgs:planning_result_py_pb2",
        "@bazel_tools//tools/python/runfiles",
    ],
    legacy_create_init = False,
    imports = [
        ".",
        "/apollo/bazel-bin",
    ],
)



py_binary(
    name = "lane_change_evaluator",
    srcs = ["lane_change_evaluator.py"],
    deps = [
        "//modules/cnoa_pnc/planning/offboard/scenario_eval/common:common",
        "//modules/msg/st_msgs:planning_debug_frame_py_pb2",
        "@bazel_tools//tools/python/runfiles",
    ],
    legacy_create_init = False,
    imports = [
        ".",
        "/apollo/bazel-bin",
    ],
)


py_binary(
    name = "metric_reader",
    srcs = ["metric_reader.py"],
    deps = [
        "//modules/cnoa_pnc/planning/offboard/scenario_eval/proto:selector_scenatro_eval",
        "@bazel_tools//tools/python/runfiles",
    ],
    legacy_create_init = False,
    imports = [
        ".",
        "/apollo/bazel-bin",
    ],
)


py_binary(
    name = "evaluator_integration",
    srcs = ["evaluator_integration.py"],
    deps = [
        "//modules/cnoa_pnc/planning/offboard/scenario_eval/common:common",
        "//modules/msg/st_msgs:planning_debug_frame_py_pb2",
        "//modules/msg/st_msgs:planning_result_py_pb2",
        "//modules/cnoa_pnc/planning/offboard/scenario_eval/proto:selector_scenatro_eval_py_pb2",
        "@bazel_tools//tools/python/runfiles",
    ],
    legacy_create_init = False,
    imports = [
        ".",
        "/apollo/bazel-bin",
    ],
)


