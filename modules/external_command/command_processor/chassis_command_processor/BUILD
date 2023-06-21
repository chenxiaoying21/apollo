load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install", "install_files", "install_plugin", "install_src_files")
load("//tools:apollo.bzl", "cyber_plugin_description")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    data_dest = "external-command-chassis",
    data = [
        ":cyberfile.xml",
        ":external-command-chassis.BUILD",
    ],
    deps = [
        "install_plugin",
    ],
)

install_plugin(
    name = "install_plugin",
    data = [
    ] + glob(["conf/**"]),
    description = ":plugins.xml",
    plugin = ":libchassis_command_processor.so",
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
    dest = "external-command-chassis/src",
    filter = "*",
)

install_src_files(
    name = "install_hdrs",
    src_dir = ["."],
    dest = "external-command-chassis/include",
    filter = "*.h",
)

cyber_plugin_description(
    name = "plugin_chassis_command_processor_description",
    description = ":plugins.xml",
    plugin = ":libchassis_command_processor.so",
)

cc_binary(
    name = "libchassis_command_processor.so",
    linkshared = True,
    linkstatic = True,
    copts = ["-DMODULE_NAME=\\\"external_command_process\\\""],
    deps = [":chassis_command_processor_lib"],
)

cc_library(
    name = "chassis_command_processor_lib",
    srcs = ["chassis_command_processor.cc"],
    hdrs = ["chassis_command_processor.h"],
    copts = ["-DMODULE_NAME=\\\"external_command_process\\\""],
    deps = [
        "//cyber",
        "//modules/external_command/command_processor/command_processor_base:command_processor_base_lib",
        "//modules/external_command/command_processor/command_processor_base/util:message_reader",
        "//modules/common_msgs/external_command_msgs:chassis_command_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_command_cc_proto",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
    ],
    alwayslink = True,
)

cpplint()
