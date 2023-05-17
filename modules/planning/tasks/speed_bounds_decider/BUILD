load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install", "install_files", "install_plugin", "install_src_files")
load("//tools:apollo.bzl", "cyber_plugin_description")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    data_dest = "planning-task-speed-bounds-decider",
    data = [
        ":cyberfile.xml",
        ":planning-task-speed-bounds-decider.BUILD",
    ],
    deps = [
        "install_plugin_speed_bounds_decider",
    ],
)

install_plugin(
    name = "install_plugin_speed_bounds_decider",
    data = [
    ] + glob(["conf/**"]),
    description = ":plugins.xml",
    plugin = ":libspeed_bounds_decider.so",
)

cyber_plugin_description(
    name = "plugin_speed_bounds_decider_description",
    description = ":plugins.xml",
    plugin = ":libspeed_bounds_decider.so",
)

cc_binary(
    name = "libspeed_bounds_decider.so",
    linkshared = True,
    linkstatic = True,
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [":speed_bounds_decider_lib"],
)

cc_library(
    name = "speed_bounds_decider_lib",
    srcs = ["speed_bounds_decider.cc"],
    hdrs = ["speed_bounds_decider.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        ":st_boundary_mapper",
        "//modules/common/status",
        "//modules/planning/common:frame",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:st_graph_data",
        "//modules/planning/common/util:common_lib",
        "//modules/planning/tasks:task",
        "//modules/planning/tasks/common:decider_base",
        "//modules/planning/tasks/speed_bounds_decider/proto:speed_bounds_decider_cc_proto",
    ],
    alwayslink = True,
)

cc_library(
    name = "st_boundary_mapper",
    srcs = [
        "speed_limit_decider.cc",
        "st_boundary_mapper.cc",
    ],
    hdrs = [
        "speed_limit_decider.h",
        "st_boundary_mapper.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common_msgs/config_msgs:vehicle_config_cc_proto",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common/status",
        "//modules/map/pnc_map",
        "//modules/planning/common:dependency_injector",
        "//modules/planning/common:frame",
        "//modules/planning/common:obstacle",
        "//modules/planning/common:path_decision",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:speed_limit",
        "//modules/planning/common/path:discretized_path",
        "//modules/planning/common/path:frenet_frame_path",
        "//modules/planning/common/path:path_data",
        "//modules/planning/common/speed:st_boundary",
        "//modules/planning/common/trajectory:discretized_trajectory",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/proto:planning_config_cc_proto",
        "//modules/planning/reference_line",
        "//modules/planning/tasks/speed_bounds_decider/proto:speed_bounds_decider_cc_proto",
    ],
)

cc_test(
    name = "st_boundary_mapper_test",
    size = "small",
    srcs = ["st_boundary_mapper_test.cc"],
    data = [
        "//modules/planning:planning_testdata",
    ],
    deps = [
        ":st_boundary_mapper",
        "//cyber",
        "//modules/common/util",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
