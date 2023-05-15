load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install", "install_files", "install_plugin", "install_src_files")
load("//tools:apollo.bzl", "cyber_plugin_description")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    data_dest = "planning-task-lane-follow-path",
    data = [
        ":cyberfile.xml",
        ":planning-task-lane-follow-path.BUILD",
    ],
    deps = [
        "install_plugin_lane_follow_path",
    ],
)

install_plugin(
    name = "install_plugin_lane_follow_path",
    data = [
    ] + glob(["conf/**"]),
    description = ":plugins.xml",
    plugin = ":liblane_follow_path.so",
)

cyber_plugin_description(
    name = "plugin_lane_follow_path_description",
    description = ":plugins.xml",
    plugin = ":liblane_follow_path.so",
)

cc_binary(
    name = "liblane_follow_path.so",
    linkshared = True,
    linkstatic = True,
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [":lane_follow_path_lib"],
)

cc_library(
    name = "lane_follow_path_lib",
    srcs = ["lane_follow_path.cc"],
    hdrs = ["lane_follow_path.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/map/pnc_map",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/tasks/common:path_generation",
        "//modules/planning/tasks/lane_follow_path/proto:lane_follow_path_cc_proto",
        "//modules/planning/tasks/common/path_util:path_util"
    ],
    alwayslink = True,
)

cpplint()
