load("@bazel_tools//tools/build_defs/pkg:pkg.bzl", "pkg_tar")
load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library","cc_import")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

config_setting(name="x86_mode", values={"cpu": "k8"})
config_setting(name="arm_mode", values={"cpu": "aarch64"})
config_setting(name = "debug_x86", values = {"define": "debug_x86=1"})

config_setting(
    name = "release_build",
    values = {
        "define": "release_build=true",
    }
)

cc_binary(
    name = "libplan_component.so",

    linkshared = True,
    linkstatic = True,
    deps = [
        # ":libnode_manager",
        "//modules/cnoa_pnc/planning/node_manager:node_manager"
    ],
    copts = ["-std=c++17",
             "-DMODULE_NAME=\\\"cnoa_plan\\\"",
            ],
)

filegroup(
    name = "libplanning_all",
    srcs = select(
        {
            "x86_mode": glob(["libs/x86/lib*.so"],
                exclude = ["libs/x86/libplan_component.so"]),
            "arm_mode": glob(["libs/arm/lib*.so"],
                exclude = ["libs/arm/libplan_component.so"]),
        }
    )
)

pkg_tar(
    name = "dag_pkg",
    srcs = glob([
        "dag/planning_onboard.dag",
        "dag/planning.dag"
        ]),
    mode = "0664",
    package_dir = "dag",
)

pkg_tar(
    name = "conf_pkg",
    srcs = glob([
        "conf/*.pb.txt",
        "conf/planner_flags.conf",
        ]),
    package_dir = "conf",
)

pkg_tar(
    name = "history_so",
    srcs = select(
        {
            "x86_mode": ["libs/x86/libplan_component.so"],
            "arm_mode": ["libs/arm/libplan_component.so"],
        }
    ),
)

pkg_tar(
    name = "release",
    srcs = select(
        {
            ":release_build": [":libplan_component.so"],
            "//conditions:default": []
        }
    ),
    deps = [
        ":dag_pkg",
        ":conf_pkg",
    ] + select(
        {
            ":release_build": [],
            "//conditions:default": [":history_so"]
        }
    ),
    package_dir = "planning",
    extension="tar.gz",
)

pkg_tar(
    name = "libs",
    srcs = [
        "@absl//:absl",
        "@jsoncpp//:json-cpp",
        "@ipopt//:ipopt",
        #lib
        # ":libplanning_all",
    ],
    extension="tar.gz",
    include_runfiles = True
)

cpplint()

