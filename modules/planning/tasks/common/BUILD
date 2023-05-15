load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "path_generation",
    srcs = ["path_generation.cc"],
    hdrs = ["path_generation.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/status",
        "//modules/planning/common:frame",
        "//modules/planning/tasks:task",
        "//modules/planning/tasks/common/path_util:path_util"
    ],
)

cpplint()
