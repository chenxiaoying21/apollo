load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

cc_library(
    name = "path_generation",
    srcs = ["path_generation.cc"],
    hdrs = ["path_generation.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/status",
        "//modules/planning/common:frame",
        "//modules/planning/tasks:task",
        "//modules/planning/tasks/common/path_util:path_util"
    ],
)

cc_library(
    name = "decider_base",
    srcs = ["decider.cc"],
    hdrs = ["decider.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/status",
        "//modules/planning/common:frame",
        "//modules/planning/tasks:task",
    ],
)

cc_library(
    name = "speed_optimizer",
    srcs = ["speed_optimizer.cc"],
    hdrs = ["speed_optimizer.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/status",
        "//modules/planning/common:path_decision",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/common:speed_limit",
        "//modules/planning/common:st_graph_data",
        "//modules/planning/common/path:path_data",
        "//modules/planning/common/speed:speed_data",
        "//modules/planning/tasks:task",
        "@eigen",
    ],
)

cc_library(
    name = "trajectory_optimizer",
    srcs = ["trajectory_optimizer.cc"],
    hdrs = ["trajectory_optimizer.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/status",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common/trajectory:discretized_trajectory",
        "//modules/planning/tasks:task",
    ],
)

cpplint()
