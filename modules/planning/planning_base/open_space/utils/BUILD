load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("@local_config_cuda//cuda:build_defs.bzl", "cuda_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_gpu")

package(default_visibility = ["//visibility:public"])
PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

cc_library(
    name = "open_space_roi_utils",
    srcs = ["open_space_roi_util.cc"],
    hdrs = ["open_space_roi_util.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning/planning_base/common:frame"
    ],
)

cpplint()
