load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install", "install_files", "install_plugin", "install_src_files")
load("//tools:apollo.bzl", "cyber_plugin_description")

package(default_visibility = ["//visibility:public"])
PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

install(
    name = "install",
    data_dest = "planning-task-open-space-fallback-decider",
    data = [
        ":cyberfile.xml",
        ":planning-task-open-space-fallback-decider.BUILD",
    ],
    deps = [
        "install_plugin_open_space_fallback_decider",
    ],
)

install_plugin(
    name = "install_plugin_open_space_fallback_decider",
    data = [
    ] + glob(["conf/**"]),
    description = ":plugins.xml",
    plugin = ":libopen_space_fallback_decider.so",
)

cyber_plugin_description(
    name = "plugin_open_space_fallback_decider_description",
    description = ":plugins.xml",
    plugin = ":libopen_space_fallback_decider.so",
)

cc_binary(
    name = "libopen_space_fallback_decider.so",
    linkshared = True,
    linkstatic = True,
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [":open_space_fallback_decider_lib"],
)

cc_library(
    name = "open_space_fallback_decider_lib",
    srcs = ["open_space_fallback_decider.cc"],
    hdrs = ["open_space_fallback_decider.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning/planning_base/common:dependency_injector",
        "//modules/planning/planning_base/common:planning_context",
        "//modules/planning/planning_base/common:planning_gflags",
        "//modules/planning/planning_base/task_base/common:decider_base",
        "//modules/planning/tasks/open_space_fallback_decider/proto:open_space_fallback_decider_cc_proto"
    ],
    alwayslink = True,
)

cc_test(
    name = "open_space_fallback_decider_test",
    size = "small",
    srcs = ["open_space_fallback_decider_test.cc"],
    deps = [
        ":open_space_fallback_decider_lib",
        "//modules/planning/planning_base/common:dependency_injector",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
