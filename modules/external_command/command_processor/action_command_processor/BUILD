load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install", "install_files", "install_plugin", "install_src_files")
load("//tools:apollo.bzl", "cyber_plugin_description")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    data_dest = "external-command-action",
    data = [
        ":cyberfile.xml",
        ":external-command-action.BUILD",
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
    plugin = ":libaction_command_processor.so",
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
    dest = "external-command-action/src",
    filter = "*",
)

install_src_files(
    name = "install_hdrs",
    src_dir = ["."],
    dest = "external-command-action/include",
    filter = "*.h",
)

cyber_plugin_description(
    name = "plugin_action_command_processor_description",
    description = ":plugins.xml",
    plugin = ":libaction_command_processor.so",
)

cc_binary(
    name = "libaction_command_processor.so",
    linkshared = True,
    linkstatic = True,
    copts = ["-DMODULE_NAME=\\\"external_command_process\\\""],
    deps = [":action_command_processor_lib"],
)

cc_library(
    name = "action_command_processor_lib",
    srcs = ["action_command_processor.cc"],
    hdrs = ["action_command_processor.h"],
    copts = ["-DMODULE_NAME=\\\"external_command_process\\\""],
    deps = [
        "//cyber",
        "//modules/external_command/command_processor/command_processor_base:command_processor_base_lib",
        "//modules/external_command/command_processor/command_processor_base/proto:command_processor_config_cc_proto",
        "//modules/external_command/command_processor/action_command_processor/proto:action_command_config_cc_proto",
        "//modules/external_command/command_processor/command_processor_base/util:message_reader",
        "//modules/common_msgs/external_command_msgs:action_command_cc_proto",
        "//modules/common_msgs/planning_msgs:pad_msg_cc_proto",
        "//modules/common_msgs/control_msgs:pad_msg_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common/util:message_util",   
    ],
    alwayslink = True,
)

cpplint()
