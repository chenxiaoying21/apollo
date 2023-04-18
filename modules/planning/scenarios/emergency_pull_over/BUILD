load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install", "install_files", "install_plugin", "install_src_files")
load("//tools:apollo.bzl", "cyber_plugin_description")

package(default_visibility = ["//visibility:public"])
PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

install(
    name = "install",
    data_dest = "planning-scenario-emergency-pull-over",
    data = [
        ":cyberfile.xml",
        ":planning-scenario-emergency-pull-over.BUILD",
    ],
    deps = [
        "install_plugin_emergency_pull_over_scenario",
    ],
)

install_plugin(
    name = "install_plugin_emergency_pull_over_scenario",
    data = [
    ] + glob(["conf/**"]),
    description = ":plugins.xml",
    plugin = ":libemergency_pull_over_scenario.so",
)

cyber_plugin_description(
    name = "plugin_emergency_pull_over_scenario_description",
    description = ":plugins.xml",
    plugin = ":libemergency_pull_over_scenario.so",
)

cc_binary(
    name = "libemergency_pull_over_scenario.so",
    linkshared = True,
    linkstatic = True,
    copts = PLANNING_COPTS,
    deps = [":emergency_pull_over_scenario_lib"],
)

cc_library(
    name = "emergency_pull_over_scenario_lib",
    srcs = [
        "emergency_pull_over_scenario.cc",
        "stage_approach.cc",
        "stage_slow_down.cc",
        "stage_standby.cc",
    ],
    hdrs = [
        "emergency_pull_over_scenario.h",
        "stage_approach.h",
        "stage_slow_down.h",
        "stage_standby.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/planning/common:planning_common",
        "//modules/planning/common:dependency_injector",
        "//modules/planning/common/util:common_lib",
        "//modules/planning/common/util:util_lib",
        "//modules/planning/scenarios:scenario",
        "//modules/planning/scenarios/emergency_pull_over/proto:emergency_pull_over_cc_proto",
        "@com_github_gflags_gflags//:gflags",
        "@eigen",
    ],
    alwayslink = True,
)

cc_test(
    name = "emergency_pull_over_scenario_test",
    size = "small",
    srcs = ["emergency_pull_over_scenario_test.cc"],
    data = [
        "//modules/planning:planning_conf",
    ],
    linkopts = ["-lgomp"],
    deps = [
        ":emergency_pull_over_scenario_lib",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

cc_test(
    name = "stage_slow_down_test",
    size = "small",
    srcs = ["stage_slow_down_test.cc"],
    data = [
        "//modules/planning:planning_conf",
    ],
    linkopts = ["-lgomp"],
    deps = [
        ":emergency_pull_over_scenario_lib",
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
        ":emergency_pull_over_scenario_lib",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

cc_test(
    name = "stage_standby_test",
    size = "small",
    srcs = ["stage_standby_test.cc"],
    data = [
        "//modules/planning:planning_conf",
    ],
    linkopts = ["-lgomp"],
    deps = [
        ":emergency_pull_over_scenario_lib",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

cpplint()
