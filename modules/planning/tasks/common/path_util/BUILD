load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "path_util",
    srcs = ["path_assessment_decider_util.cc",
                  "path_bounds_decider_util.cc",
                  "path_optimizer_util.cc"],
    hdrs = ["path_assessment_decider_util.h",
                  "path_bounds_decider_util.h",
                  "path_optimizer_util.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/planning/common:frame",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common/path:discretized_path",
        "//modules/planning/common/path:frenet_frame_path",
        "//modules/planning/common/path:path_data",
        "//modules/planning/proto:piecewise_jerk_path_config_cc_proto",
         "//modules/planning/math/piecewise_jerk:piecewise_jerk_path_problem",
         "//modules/planning/common/trajectory1d:piecewise_jerk_trajectory1d"
    ],
)

cpplint()
