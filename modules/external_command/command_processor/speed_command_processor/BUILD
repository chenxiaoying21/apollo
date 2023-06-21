load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install", "install_files", "install_plugin", "install_src_files")
load("//tools:apollo.bzl", "cyber_plugin_description")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    data_dest = "external-command-speed",
    data = [
        ":cyberfile.xml",
        ":external-command-speed.BUILD",
    ],
    deps = [
        "install_plugin",
    ],
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
    dest = "external-command-speed/src",
    filter = "*",
)

install_src_files(
    name = "install_hdrs",
    src_dir = ["."],
    dest = "external-command-speed/include",
    filter = "*.h",
)

install_plugin(
    name = "install_plugin",
    data = [
    ] + glob(["conf/**"]),
    description = ":plugins.xml",
    plugin = ":libspeed_command_processor.so",
)

cyber_plugin_description(
    name = "plugin_speed_command_processor_description",
    description = ":plugins.xml",
    plugin = ":libspeed_command_processor.so",
)

cc_binary(
    name = "libspeed_command_processor.so",
    linkshared = True,
    linkstatic = True,
    copts = ["-DMODULE_NAME=\\\"external_command_process\\\""],
    deps = [":speed_command_processor_lib"],
)

cc_library(
    name = "speed_command_processor_lib",
    srcs = ["speed_command_processor.cc"],
    hdrs = ["speed_command_processor.h"],
    copts = ["-DMODULE_NAME=\\\"external_command_process\\\""],
    deps = [
        "//cyber",
        "//modules/external_command/command_processor/command_processor_base:command_processor_base_lib",
        "//modules/external_command/command_processor/command_processor_base/util:message_reader",
        "//modules/common_msgs/external_command_msgs:speed_command_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_command_cc_proto",
    ],
    alwayslink = True,
)

cpplint()
