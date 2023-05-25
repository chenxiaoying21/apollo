load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install", "install_files", "install_plugin", "install_src_files")
load("//tools:apollo.bzl", "cyber_plugin_description")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    data_dest = "external-command-lane-follow",
    data = [
        ":cyberfile.xml",
        ":external-command-lane-follow.BUILD",
    ],
    deps = [
        "install_plugin_lane_follow_command_processor",
    ],
)

install_plugin(
    name = "install_plugin_lane_follow_command_processor",
    data = [
    ] + glob(["conf/**"]),
    description = ":plugins.xml",
    plugin = ":liblane_follow_command_processor.so",
)

cyber_plugin_description(
    name = "plugin_lane_follow_command_processor_description",
    description = ":plugins.xml",
    plugin = ":liblane_follow_command_processor.so",
)

cc_binary(
    name = "liblane_follow_command_processor.so",
    linkshared = True,
    linkstatic = True,
    copts = ["-DMODULE_NAME=\\\"external_command\\\""],
    deps = [":lane_follow_command_processor_lib"],
)

cc_library(
    name = "lane_follow_command_processor_lib",
    srcs = ["lane_follow_command_processor.cc"],
    hdrs = ["lane_follow_command_processor.h"],
    copts = ["-DMODULE_NAME=\\\"external_command\\\""],
    deps = [
        "//cyber",
        "//modules/external_command_process/command_processor/command_processor_base:motion_command_processor_base_lib",
        "//modules/common_msgs/external_command_msgs:lane_follow_command_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_command_cc_proto",
    ],
    alwayslink = True,
)

cpplint()
