load("@rules_python//python:defs.bzl", "py_library","py_binary")
package(default_visibility = ["//visibility:public"])

filegroup(
    name = "distribution",
    srcs = glob(["**"]),
    visibility = ["//python:__pkg__"],
)

py_library(
    name = "common",
    srcs = ["common.py"],
    deps = [
        "@bazel_tools//tools/python/runfiles",
    ]
)

