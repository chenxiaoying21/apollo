load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install", "install_files", "install_plugin", "install_src_files")
load("//tools:apollo.bzl", "cyber_plugin_description")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    data_dest = "planning-task-path-bounds-decider",
    data = [
        ":cyberfile.xml",
        ":planning-task-path-bounds-decider.BUILD",
    ],
    deps = [
        "install_plugin_path_bounds_decider",
    ],
)

install_plugin(
    name = "install_plugin_path_bounds_decider",
    data = [
    ] + glob(["conf/**"]),
    description = ":plugins.xml",
    plugin = ":libpath_bounds_decider.so",
)

cyber_plugin_description(
    name = "plugin_path_bounds_decider_description",
    description = ":plugins.xml",
    plugin = ":libpath_bounds_decider.so",
)

cc_binary(
    name = "libpath_bounds_decider.so",
    linkshared = True,
    linkstatic = True,
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [":path_bounds_decider_lib"],
)

cc_library(
    name = "path_bounds_decider_lib",
    srcs = ["path_bounds_decider.cc"],
    hdrs = ["path_bounds_decider.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/tasks/deciders:decider_base",
        "//modules/planning/tasks/deciders/utils:path_decider_obstacle_utils",
        "//modules/planning/tasks/path_bounds_decider/proto:path_bounds_decider_cc_proto",
    ],
    alwayslink = True,
)

cc_test(
    name = "path_bounds_decider_test",
    size = "small",
    srcs = ["path_bounds_decider_test.cc"],
    deps = [
        "path_bounds_decider_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
