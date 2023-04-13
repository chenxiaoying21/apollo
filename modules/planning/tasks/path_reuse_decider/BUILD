load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install", "install_files", "install_plugin", "install_src_files")
load("//tools:apollo.bzl", "cyber_plugin_description")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    data_dest = "planning-task-path-reuse-decider",
    data = [
        ":cyberfile.xml",
        ":planning-task-path-reuse-decider.BUILD",
    ],
    deps = [
        "install_plugin_path_reuse_decider",
    ],
)

install_plugin(
    name = "install_plugin_path_reuse_decider",
    data = [
    ] + glob(["conf/**"]),
    description = ":plugins.xml",
    plugin = ":libpath_reuse_decider.so",
)

cyber_plugin_description(
    name = "plugin_path_reuse_decider_description",
    description = ":plugins.xml",
    plugin = ":libpath_reuse_decider.so",
)

cc_binary(
    name = "libpath_reuse_decider.so",
    linkshared = True,
    linkstatic = True,
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [":path_reuse_decider_lib"],
)

cc_library(
    name = "path_reuse_decider_lib",
    srcs = ["path_reuse_decider.cc"],
    hdrs = ["path_reuse_decider.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/planning/common:history",
        "//modules/planning/common:obstacle_blocking_analyzer",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:reference_line_info",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/tasks/deciders:decider_base",
        "//modules/planning/tasks/path_reuse_decider/proto:path_reuse_decider_cc_proto",
    ],
    alwayslink = True,
)

cc_test(
    name = "path_reuse_decider_test",
    size = "small",
    srcs = ["path_reuse_decider_test.cc"],
    deps = [
        "path_reuse_decider_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
