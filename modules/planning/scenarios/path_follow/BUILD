load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install", "install_files", "install_plugin", "install_src_files")
load("//tools:apollo.bzl", "cyber_plugin_description")

package(default_visibility = ["//visibility:public"])
PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

install(
    name = "install",
    data_dest = "planning-scenario-path-follow",
    data = [
        ":cyberfile.xml",
        ":planning-scenario-path-follow.BUILD",
    ],
    deps = [
        "install_plugin",
    ],
)

install_plugin(
    name = "install_plugin",
    data = [
    ] + glob(["conf/**"]),
    description = ":plugins.xml",
    plugin = ":libpath_follow_scenario.so",
)

install_src_files(
    name = "install_src",
    deps = [
        ":install_all_src",
        ":install_hdrs"
    ],
)

install_src_files(
    name = "install_all_src",
    src_dir = ["."],
    dest = "planning-scenario-path-follow/src",
    filter = "*",
)

install_src_files(
    name = "install_hdrs",
    src_dir = ["."],
    dest = "planning-scenario-path-follow/include",
    filter = "*.h",
)

cyber_plugin_description(
    name = "plugin_path_follow_scenario_description",
    description = ":plugins.xml",
    plugin = ":libpath_follow_scenario.so",
)


cc_binary(
    name = "libpath_follow_scenario.so",
    linkshared = True,
    linkstatic = True,
    copts = PLANNING_COPTS,
    deps = [":path_follow_scenario_lib"],
)

cc_library(
    name = "path_follow_scenario_lib",
    srcs = ["path_follow_scenario.cc"],
    hdrs = ["path_follow_scenario.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//cyber",
        "//modules/planning/scenarios/lane_follow:lane_follow_scenario_lib",
        "//modules/common_msgs/external_command_msgs:path_follow_command_cc_proto",
        "@com_github_gflags_gflags//:gflags",
        "@eigen",
    ],
    alwayslink = True,
)

cc_test(
    name = "path_follow_scenario_test",
    size = "small",
    srcs = ["path_follow_scenario_test.cc"],
    data = [
        "//modules/planning/planning_base:planning_conf",
    ],
    linkopts = ["-lgomp"],
    deps = [
        ":path_follow_scenario_lib",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

cpplint()
