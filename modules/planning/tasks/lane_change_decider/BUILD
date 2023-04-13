load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install", "install_files", "install_plugin", "install_src_files")
load("//tools:apollo.bzl", "cyber_plugin_description")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    data_dest = "planning-task-lane-change-decider",
    data = [
        ":cyberfile.xml",
        ":planning-task-lane-change-decider.BUILD",
    ],
    deps = [
        "install_plugin_lane_change_decider",
    ],
)

install_plugin(
    name = "install_plugin_lane_change_decider",
    data = [
    ] + glob(["conf/**"]),
    description = ":plugins.xml",
    plugin = ":liblane_change_decider.so",
)

cyber_plugin_description(
    name = "plugin_lane_change_decider_description",
    description = ":plugins.xml",
    plugin = ":liblane_change_decider.so",
)

cc_binary(
    name = "liblane_change_decider.so",
    linkshared = True,
    linkstatic = True,
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [":lane_change_decider_lib"],
)

cc_library(
    name = "lane_change_decider_lib",
    srcs = ["lane_change_decider.cc"],
    hdrs = ["lane_change_decider.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/map/pnc_map",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/tasks/deciders:decider_base",
        "//modules/planning/tasks/common/lane_change_util:lane_change_util",
        "//modules/planning/tasks/lane_change_decider/proto:lane_change_decider_cc_proto"
    ],
    alwayslink = True,
)

cpplint()
