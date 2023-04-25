load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install", "install_files", "install_plugin", "install_src_files")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    deps = [
        "//modules/planning/tasks/lane_change_decider:install",
        "//modules/planning/tasks/open_space_fallback_decider:install",
        "//modules/planning/tasks/open_space_pre_stop_decider:install",
        "//modules/planning/tasks/open_space_roi_decider:install",
        "//modules/planning/tasks/open_space_trajectory_partition:install",
        "//modules/planning/tasks/open_space_trajectory_provider:install",
        "//modules/planning/tasks/path_assessment_decider:install",
        "//modules/planning/tasks/path_bounds_decider:install",
        "//modules/planning/tasks/path_decider:install",
        "//modules/planning/tasks/path_lane_borrow_decider:install",
        "//modules/planning/tasks/path_reference_decider:install",
        "//modules/planning/tasks/path_reuse_decider:install",
        "//modules/planning/tasks/path_time_heuristic:install",
        "//modules/planning/tasks/piecewise_jerk_path_optimizer:install",
        "//modules/planning/tasks/piecewise_jerk_speed:install",
        "//modules/planning/tasks/piecewise_jerk_speed_nonlinear:install",
        "//modules/planning/tasks/rss_decider:install",
        "//modules/planning/tasks/rule_based_stop_decider:install",
        "//modules/planning/tasks/speed_bounds_decider:install",
        "//modules/planning/tasks/speed_decider:install",
        "//modules/planning/tasks/st_bounds_decider:install",
    ],
)

cc_library(
    name = "task",
    srcs = ["task.cc"],
    hdrs = ["task.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/status",
        "//modules/planning/common:dependency_injector",
        "//modules/planning/common:frame",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/common:config_util",
        "//modules/planning/common/trajectory:discretized_trajectory",
        "//modules/planning/common/trajectory:publishable_trajectory",
    ],
)

cpplint()
