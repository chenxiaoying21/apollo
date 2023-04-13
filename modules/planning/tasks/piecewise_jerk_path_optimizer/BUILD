load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install", "install_files", "install_plugin", "install_src_files")
load("//tools:apollo.bzl", "cyber_plugin_description")

package(default_visibility = ["//visibility:public"])
PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

install(
    name = "install",
    data_dest = "planning-task-piecewise-jerk-path-optimizer",
    data = [
        ":cyberfile.xml",
        ":planning-task-piecewise-jerk-path-optimizer.BUILD",
    ],
    deps = [
        "install_plugin_piecewise_jerk_path_optimizer",
    ],
)

install_plugin(
    name = "install_plugin_piecewise_jerk_path_optimizer",
    data = [
    ] + glob(["conf/**"]),
    description = ":plugins.xml",
    plugin = ":libpiecewise_jerk_path_optimizer.so",
)

cyber_plugin_description(
    name = "plugin_piecewise_jerk_path_optimizer_description",
    description = ":plugins.xml",
    plugin = ":libpiecewise_jerk_path_optimizer.so",
)

cc_binary(
    name = "libpiecewise_jerk_path_optimizer.so",
    linkshared = True,
    linkstatic = True,
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [":piecewise_jerk_path_optimizer_lib"],
)

cc_library(
    name = "piecewise_jerk_path_optimizer_lib",
    srcs = ["piecewise_jerk_path_optimizer.cc"],
    hdrs = ["piecewise_jerk_path_optimizer.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common/util",
        "//modules/planning/common:obstacle",
        "//modules/planning/common:path_boundary",
        "//modules/planning/common:path_decision",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common/path:discretized_path",
        "//modules/planning/common/path:frenet_frame_path",
        "//modules/planning/common/path:path_data",
        "//modules/planning/common/speed:speed_data",
        "//modules/planning/lattice/trajectory_generation:trajectory1d_generator",
        "//modules/planning/math:polynomial_xd",
        "//modules/planning/math/curve1d:polynomial_curve1d",
        "//modules/planning/math/curve1d:quintic_polynomial_curve1d",
        "//modules/planning/math/piecewise_jerk:piecewise_jerk_path_problem",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/reference_line",
        "//modules/planning/tasks/optimizers:path_optimizer",
        "//modules/planning/tasks/piecewise_jerk_path_optimizer/proto:piecewise_jerk_path_optimizer_cc_proto",
        "@eigen",
    ],
    alwayslink = True,
)

cc_library(
    name = "piecewise_jerk_path_ipopt_solver",
    srcs = ["piecewise_jerk_path_ipopt_solver.cc"],
    hdrs = ["piecewise_jerk_path_ipopt_solver.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//cyber",
        "@eigen",
        "@ipopt",
    ],
)

cpplint()
