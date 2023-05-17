load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install", "install_files")
load("//tools/install:install.bzl", "install", "install_files", "install_plugin", "install_src_files")
load("//tools:apollo.bzl", "cyber_plugin_description")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    data_dest = "planning-task-open-space-trajectory-partition",
    data = [
        ":cyberfile.xml",
        ":planning-task-open-space-trajectory-partition.BUILD",
    ],
    deps = [
        "install_plugin_open_space_trajectory_partition",
    ],
)

install_plugin(
    name = "install_plugin_open_space_trajectory_partition",
    data = [
    ] + glob(["conf/**"]),
    description = ":plugins.xml",
    plugin = ":libopen_space_trajectory_partition.so",
)

cyber_plugin_description(
    name = "plugin_open_space_trajectory_partition_description",
    description = ":plugins.xml",
    plugin = ":libopen_space_trajectory_partition.so",
)

cc_binary(
    name = "libopen_space_trajectory_partition.so",
    linkshared = True,
    linkstatic = True,
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [":open_space_trajectory_partition_lib"],
)

cc_library(
    name = "open_space_trajectory_partition_lib",
    srcs = ["open_space_trajectory_partition.cc"],
    hdrs = ["open_space_trajectory_partition.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        "//modules/common/status",
        "//modules/planning/proto:planning_config_cc_proto",
        "//modules/planning/tasks:task",
        "//modules/planning/tasks/common:trajectory_optimizer",
        "//modules/planning/tasks/open_space_trajectory_partition/proto:open_space_trajectory_partition_cc_proto"
    ],
    alwayslink = True,
)

cc_test(
    name = "open_space_trajectory_partition_test",
    size = "small",
    srcs = ["open_space_trajectory_partition_test.cc"],
    deps = [
        ":open_space_trajectory_partition_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
