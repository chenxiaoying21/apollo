load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install", "install_files", "install_plugin", "install_src_files")
load("//tools:apollo.bzl", "cyber_plugin_description")

package(default_visibility = ["//visibility:public"])
PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

install(
    name = "install",
    data_dest = "planning-task-open-space-pre-stop-decider",
    data = [
        ":cyberfile.xml",
        ":planning-task-open-space-pre-stop-decider.BUILD",
    ],
    deps = [
        "install_plugin_open_space_pre_stop_decider",
    ],
)

install_plugin(
    name = "install_plugin_open_space_pre_stop_decider",
    data = [
    ] + glob(["conf/**"]),
    description = ":plugins.xml",
    plugin = ":libopen_space_pre_stop_decider.so",
)

cyber_plugin_description(
    name = "plugin_open_space_pre_stop_decider_description",
    description = ":plugins.xml",
    plugin = ":libopen_space_pre_stop_decider.so",
)

cc_binary(
    name = "libopen_space_pre_stop_decider.so",
    linkshared = True,
    linkstatic = True,
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [":open_space_pre_stop_decider_lib"],
)

cc_library(
    name = "open_space_pre_stop_decider_lib",
    srcs = ["open_space_pre_stop_decider.cc"],
    hdrs = ["open_space_pre_stop_decider.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning/common:reference_line_info",
        "//modules/planning/common/util:common_lib",
        "//modules/planning/tasks/common:decider_base",
        "//modules/planning/tasks/open_space_pre_stop_decider/proto:open_space_pre_stop_decider_cc_proto"
    ],
    alwayslink = True,
)

cpplint()
