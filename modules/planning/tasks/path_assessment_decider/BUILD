load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install", "install_files", "install_plugin", "install_src_files")
load("//tools:apollo.bzl", "cyber_plugin_description")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    data_dest = "planning-task-path-assessment-decider",
    data = [
        ":cyberfile.xml",
        ":planning-task-path-assessment-decider.BUILD",
    ],
    deps = [
        "install_plugin_path_assessment_decider",
    ],
)

install_plugin(
    name = "install_plugin_path_assessment_decider",
    data = [
    ] + glob(["conf/**"]),
    description = ":plugins.xml",
    plugin = ":libpath_assessment_decider.so",
)

cyber_plugin_description(
    name = "plugin_path_assessment_decider_description",
    description = ":plugins.xml",
    plugin = ":libpath_assessment_decider.so",
)


cc_binary(
    name = "libpath_assessment_decider.so",
    linkshared = True,
    linkstatic = True,
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [":path_assessment_decider_lib"],
)

cc_library(
    name = "path_assessment_decider_lib",
    srcs = ["path_assessment_decider.cc"],
    hdrs = ["path_assessment_decider.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/tasks/deciders:decider_base",
        "//modules/planning/tasks/deciders/utils:path_decider_obstacle_utils",
    ],
    alwayslink = True,
)

cpplint()
