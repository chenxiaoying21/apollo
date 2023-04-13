load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])
PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

cc_library(
    name = "scenario",
    srcs = ["scenario.cc"],
    hdrs = ["scenario.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":stage",
        "//modules/planning/common:planning_common",
        "//modules/planning/common:config_util",
        "//modules/planning/common/util:util_lib",
        "//modules/planning/tasks:task",
        "//cyber/plugin_manager:plugin_manager",
        "//modules/planning/proto:scenario_pipeline_cc_proto",
    ],
)

cc_library(
    name = "stage",
    srcs = [
        "stage.cc",
        "stage_intersection_cruise.cc",
    ],
    hdrs = [
        "stage.h",
        "stage_intersection_cruise.h",
    ],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning/common:planning_common",
        "//modules/planning/common:config_util",
        "//modules/planning/common/util:util_lib",
        "//modules/planning/tasks:task",
        "//cyber/plugin_manager:plugin_manager",
        "//modules/planning/proto:scenario_pipeline_cc_proto",
        "//modules/planning/common:speed_profile_generator",
    ],
)

cc_library(
    name = "scenario_manager",
    srcs = ["scenario_manager.cc"],
    hdrs = ["scenario_manager.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/planning/common:planning_common",
        "//modules/planning/common:planning_context",
        "//modules/planning/common/util:util_lib",
        "//cyber/plugin_manager:plugin_manager",
        ":scenario",
    ],
)

cpplint()
