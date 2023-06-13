load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install", "install_files", "install_plugin", "install_src_files")
load("//tools:apollo.bzl", "cyber_plugin_description")

package(default_visibility = ["//visibility:public"])
PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

install(
    name = "install",
    data_dest = "planning-scenario-lane-follow",
    data = [
        ":cyberfile.xml",
        ":planning-scenario-lane-follow.BUILD",
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
    plugin = ":liblane_follow_scenario.so",
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
    dest = "planning-scenario-lane-follow/src",
    filter = "*",
)

install_src_files(
    name = "install_hdrs",
    src_dir = ["."],
    dest = "planning-scenario-lane-follow/include",
    filter = "*.h",
)

cyber_plugin_description(
    name = "plugin_lane_follow_scenario_description",
    description = ":plugins.xml",
    plugin = ":liblane_follow_scenario.so",
)


cc_binary(
    name = "liblane_follow_scenario.so",
    linkshared = True,
    linkstatic = True,
    copts = PLANNING_COPTS,
    deps = [":lane_follow_scenario_lib"],
)

cc_library(
    name = "lane_follow_scenario_lib",
    srcs = ["lane_follow_scenario.cc"],
    hdrs = ["lane_follow_scenario.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":lane_follow_stage",
        "//cyber",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common/status",
        "//modules/common/util",
        "//modules/common/util:util_tool",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/map/hdmap",
        "//modules/planning/planning_base/common:planning_common",
        "//modules/planning/planning_base/common:speed_profile_generator",
        "//modules/planning/planning_base/constraint_checker",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/planning_base/scenario_base:scenario",
        "@com_github_gflags_gflags//:gflags",
        "@eigen",
    ],
    alwayslink = True,
)

cc_library(
    name = "lane_follow_stage",
    srcs = ["lane_follow_stage.cc"],
    hdrs = ["lane_follow_stage.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//cyber",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common/status",
        "//modules/common/util",
        "//modules/common/util:util_tool",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/map/hdmap",
        "//modules/planning/planning_base/common:planning_common",
        "//modules/planning/planning_base/common:speed_profile_generator",
        "//modules/planning/planning_base/constraint_checker",
        "//modules/planning/planning_base/math/curve1d:quartic_polynomial_curve1d",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/planning_base/reference_line",
        "//modules/planning/planning_base/reference_line:qp_spline_reference_line_smoother",
        "//modules/planning/planning_base/scenario_base:scenario",
        "@com_github_gflags_gflags//:gflags",
        "@eigen",
    ],
    alwayslink = True,
)

cc_test(
    name = "lane_follow_scenario_test",
    size = "small",
    srcs = ["lane_follow_scenario_test.cc"],
    data = [
        "//modules/planning/planning_base:planning_conf",
    ],
    linkopts = ["-lgomp"],
    deps = [
        ":lane_follow_scenario_lib",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

cpplint()
