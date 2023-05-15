load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install", "install_files", "install_plugin", "install_src_files")
load("//tools:apollo.bzl", "cyber_plugin_description")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    data_dest = "planning-task-pull-over-path",
    data = [
        ":cyberfile.xml",
        ":planning-task-pull-over-path.BUILD",
    ],
    deps = [
        "install_plugin_pull_over_path",
    ],
)

install_plugin(
    name = "install_plugin_pull_over_path",
    data = [
    ] + glob(["conf/**"]),
    description = ":plugins.xml",
    plugin = ":libpull_over_path.so",
)

cyber_plugin_description(
    name = "plugin_pull_over_path_description",
    description = ":plugins.xml",
    plugin = ":libpull_over_path.so",
)

cc_binary(
    name = "libpull_over_path.so",
    linkshared = True,
    linkstatic = True,
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [":pull_over_path_lib"],
)

cc_library(
    name = "pull_over_path_lib",
    srcs = ["pull_over_path.cc"],
    hdrs = ["pull_over_path.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/map/pnc_map",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/tasks/common:path_generation",
        "//modules/planning/tasks/pull_over_path/proto:pull_over_path_cc_proto",
        "//modules/planning/tasks/common/path_util:path_util"
    ],
    alwayslink = True,
)

cpplint()
