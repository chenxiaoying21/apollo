load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install", "install_files", "install_plugin", "install_src_files")
load("//tools:apollo.bzl", "cyber_plugin_description")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    data_dest = "planning-task-lane-change-path",
    data = [
        ":cyberfile.xml",
        ":planning-task-lane-change-path.BUILD",
    ],
    deps = [
        "install_plugin_lane_change_path",
    ],
)

install_plugin(
    name = "install_plugin_lane_change_path",
    data = [
    ] + glob(["conf/**"]),
    description = ":plugins.xml",
    plugin = ":liblane_change_path.so",
)

cyber_plugin_description(
    name = "plugin_lane_change_path_description",
    description = ":plugins.xml",
    plugin = ":liblane_change_path.so",
)

cc_binary(
    name = "liblane_change_path.so",
    linkshared = True,
    linkstatic = True,
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [":lane_change_path_lib"],
)

cc_library(
    name = "lane_change_path_lib",
    srcs = ["lane_change_path.cc"],
    hdrs = ["lane_change_path.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/map/pnc_map",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:obstacle_blocking_analyzer",
        "//modules/planning/common:reference_line_info",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/tasks/common:path_generation",
        "//modules/planning/tasks/lane_change_path/proto:lane_change_path_cc_proto",
        "//modules/planning/tasks/common/path_util:path_util"
    ],
    alwayslink = True,
)

cpplint()
