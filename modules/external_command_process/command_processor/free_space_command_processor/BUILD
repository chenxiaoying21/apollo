load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install", "install_files", "install_plugin", "install_src_files")
load("//tools:apollo.bzl", "cyber_plugin_description")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    data_dest = "external-command-free-space",
    data = [
        ":cyberfile.xml",
        ":external-command-free-space.BUILD",
    ],
    deps = [
        "install_plugin_free_space_command_processor",
    ],
)

install_plugin(
    name = "install_plugin_free_space_command_processor",
    data = [
    ] + glob(["conf/**"]),
    description = ":plugins.xml",
    plugin = ":libfree_space_command_processor.so",
)

cyber_plugin_description(
    name = "plugin_free_space_command_processor_description",
    description = ":plugins.xml",
    plugin = ":libfree_space_command_processor.so",
)

cc_binary(
    name = "libfree_space_command_processor.so",
    linkshared = True,
    linkstatic = True,
    copts = ["-DMODULE_NAME=\\\"external_command_process\\\""],
    deps = [":free_space_command_processor_lib"],
)

cc_library(
    name = "free_space_command_processor_lib",
    srcs = ["free_space_command_processor.cc"],
    hdrs = ["free_space_command_processor.h"],
    copts = ["-DMODULE_NAME=\\\"external_command_process\\\""],
    deps = [
        "//cyber",
        "//modules/external_command_process/command_processor/command_processor_base:motion_command_processor_base_lib",
        "//modules/common_msgs/external_command_msgs:free_space_command_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_command_cc_proto",
    ],
    alwayslink = True,
)

cpplint()
