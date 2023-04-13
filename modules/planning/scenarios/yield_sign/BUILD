load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install", "install_files", "install_plugin", "install_src_files")
load("//tools:apollo.bzl", "cyber_plugin_description")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    data_dest = "planning-scenario-yield-sign",
    data = [
        ":cyberfile.xml",
        ":planning-scenario-yield-sign.BUILD",
    ],
    deps = [
        "install_plugin_yield_sign_scenario",
    ],
)

install_plugin(
    name = "install_plugin_yield_sign_scenario",
    data = [
    ] + glob(["conf/**"]),
    description = ":plugins.xml",
    plugin = ":libyield_sign_scenario.so",
)

cyber_plugin_description(
    name = "plugin_yield_sign_scenario_description",
    description = ":plugins.xml",
    plugin = ":libyield_sign_scenario.so",
)

cc_binary(
    name = "libyield_sign_scenario.so",
    linkshared = True,
    linkstatic = True,
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [":yield_sign_scenario_lib"],
)

cc_library(
    name = "yield_sign_scenario_lib",
    srcs = [
        "stage_approach.cc",
        "stage_creep.cc",
        "yield_sign_scenario.cc",
    ],
    hdrs = [
        "stage_approach.h",
        "stage_creep.h",
        "yield_sign_scenario.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/planning/common:planning_common",
        "//modules/planning/common/util:common_lib",
        "//modules/planning/common/util:util_lib",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/scenarios:scenario",
        "//modules/planning/scenarios/yield_sign/proto:yield_sign_cc_proto",
        "//modules/planning/scenarios/common/creep_util:creep_decider",
        "@com_github_gflags_gflags//:gflags",
        "@eigen",
    ],
    alwayslink = True,
)

cc_test(
    name = "yield_sign_scenario_test",
    size = "small",
    srcs = ["yield_sign_scenario_test.cc"],
    data = [
        "//modules/planning:planning_conf",
    ],
    linkopts = ["-lgomp"],
    deps = [
        ":yield_sign_scenario_lib",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

cc_test(
    name = "stage_approach_test",
    size = "small",
    srcs = ["stage_approach_test.cc"],
    data = [
        "//modules/planning:planning_conf",
    ],
    linkopts = ["-lgomp"],
    deps = [
        ":yield_sign_scenario_lib",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

cc_test(
    name = "stage_creep_test",
    size = "small",
    srcs = ["stage_creep_test.cc"],
    data = [
        "//modules/planning:planning_conf",
    ],
    linkopts = ["-lgomp"],
    deps = [
        ":yield_sign_scenario_lib",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

cpplint()
