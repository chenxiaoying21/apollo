load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install", "install_files")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "creep_decider",
    srcs = ["creep_decider.cc"],
    hdrs = ["creep_decider.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/planning/common:planning_context",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/common/util:common_lib",
        "//modules/planning/common/util:util_lib",
        "//modules/planning/tasks/deciders:decider_base",
        "//modules/planning/proto:creep_decider_cc_proto",
    ],
    alwayslink = True,
)

cpplint()
