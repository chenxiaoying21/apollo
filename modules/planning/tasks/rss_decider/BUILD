load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install", "install_files", "install_plugin", "install_src_files")
load("//tools:apollo.bzl", "cyber_plugin_description")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    data_dest = "planning-task-rss-decider",
    data = [
        ":cyberfile.xml",
        ":planning-task-rss-decider.BUILD",
    ],
    deps = [
        "install_plugin_rss_decider",
    ],
)

install_plugin(
    name = "install_plugin_rss_decider",
    data = [
    ] + glob(["conf/**"]),
    description = ":plugins.xml",
    plugin = ":librss_decider.so",
)

cyber_plugin_description(
    name = "plugin_rss_decider_description",
    description = ":plugins.xml",
    plugin = ":librss_decider.so",
)

cc_binary(
    name = "librss_decider.so",
    linkshared = True,
    linkstatic = True,
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [":rss_decider_lib"],
)

cc_library(
    name = "rss_decider_lib",
    srcs = ["rss_decider.cc"],
    hdrs = ["rss_decider.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/status",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/tasks:task",
        "@ad_rss_lib//:ad_rss",
        "//modules/planning/tasks/rss_decider/proto:rss_decider_cc_proto",
    ],
    alwayslink = True,
)

cc_test(
    name = "rss_decider_test",
    size = "small",
    srcs = ["rss_decider_test.cc"],
    tags = ["exclude"],
    deps = [
        ":rss_decider_lib",
        "@ad_rss_lib//:ad_rss",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
