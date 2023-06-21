load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

ROUTING_COPTS = ['-DMODULE_NAME=\\"routing\\"']

cc_library(
    name = "lane_way_tool",
    srcs = ["lane_way_tool.cc"],
    hdrs = ["lane_way_tool.h"],
    copts = ROUTING_COPTS,
    deps = [
        "//cyber",
        "//modules/map/hdmap:hdmap",
        "//modules/map/hdmap:hdmap_util",
        "//modules/common_msgs/external_command_msgs:geometry_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common_msgs/routing_msgs:routing_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/external_command/command_processor/command_processor_base/util:message_reader",
    ],
    alwayslink = True,
)

cc_library(
    name = "message_reader",
    srcs = ["message_reader.cc"],
    hdrs = ["message_reader.h"],
    copts = ROUTING_COPTS,
    deps = [
        "//cyber",
    ],
    alwayslink = True,
)

cpplint()
