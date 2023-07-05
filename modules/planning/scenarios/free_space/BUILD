load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install", "install_files", "install_plugin", "install_src_files")
load("//tools:apollo.bzl", "cyber_plugin_description")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    data_dest = "planning-scenario-free-space",
    data = [
        ":cyberfile.xml",
        ":planning-scenario-free-space.BUILD",
    ],
    deps = [
        "install_plugin_free_space_scenario",
    ],
)

install_plugin(
    name = "install_plugin_free_space_scenario",
    data = [
    ] + glob(["conf/**"]),
    description = ":plugins.xml",
    plugin = ":libfree_space_scenario.so",
)

cyber_plugin_description(
    name = "plugin_free_space_description",
    description = ":plugins.xml",
    plugin = ":libfree_space_scenario.so",
)

cc_binary(
    name = "libfree_space_scenario.so",
    linkshared = True,
    linkstatic = True,
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [":free_space_scenario_lib"],
)

cc_library(
    name = "free_space_scenario_lib",
    srcs = [
        "stage_free_space.cc",
        "free_space_scenario.cc",
    ],
    hdrs = [
        "stage_free_space.h",
        "free_space_scenario.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber",
        "//modules/planning/planning_base/open_space/utils:open_space_roi_utils",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/common_msgs/external_command_msgs:free_space_command_cc_proto",
        "//modules/planning/planning_base/scenario_base:scenario",
        "//modules/planning/scenarios/free_space/proto:free_space_cc_proto",
    ],
    alwayslink = True,
)

cpplint()
