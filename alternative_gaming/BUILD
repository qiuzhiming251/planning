load("@rules_cc//cc:defs.bzl","cc_binary","cc_library","cc_test")
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
        "-Imodules/cnoa_pnc/planning/planner/include",
        "-Imodules/cnoa_pnc/planning",
        "-std=c++17",
        #"-g",
        #"-O0",
        "-DMODULE_NAME=\\\"cnoa_plan\\\"",
    ],
})  + select({
    ":arm_mode": ["-DPLATFORM_ARM"],
    ":x86_mode": ["-DPLATFORM_X86"],
})

filegroup(
    name = "speed_gaming_srcs",
    srcs = glob(["speed_gaming/**/*.cc"],exclude = ["**/*_test.cc"])
)

filegroup(
    name = "speed_gaming_hdrs",
    srcs = glob(["speed_gaming/**/*.h"])
)

filegroup(
    name = "driveline_srcs",
    srcs = glob(["drf_driveline/**/*.cc"],exclude = ["**/*_test.cc"])
)

filegroup(
    name = "driveline_hdrs",
    srcs = glob(["drf_driveline/**/*.h"])
)

cc_library(
    name = "alternative_gaming",
    srcs = ["speed_gaming_srcs", "driveline_srcs"],
    hdrs = ["speed_gaming_hdrs", "driveline_hdrs"],
    deps = [#lib
    "//modules/cnoa_pnc/planning/object_manager:object_manager",],
    copts = COPTS,
    alwayslink = True,
    #defines = ["ALTERNATIVE_GAMING_UNIT_TEST"],
) 

filegroup(
    name = "test_srcs",
    srcs = glob(["**/*_test.cc"],exclude = ["**/speed_obstacle_preprocessor_test.cc","**/speed_gaming_unit_test.cc","**/multi_agent_unit_test.cc"])
)

cc_test(
    name = "my_ut",
    srcs = ["test_srcs"],  
    deps = [
        "@com_google_googletest//:gtest", 
        ":alternative_gaming",
    ],
    copts = COPTS,
    linkstatic = True,
    defines = ["ALTERNATIVE_GAMING_UNIT_TEST"],
)

cpplint()