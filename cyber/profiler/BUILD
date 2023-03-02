load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

config_setting(
    name = "enable_profiler",
    define_values = {
        "ENABLE_PROFILER": "true",
    },
)

cc_library(
    name = "block_manager",
    srcs = [
        "block_manager.cc",
        "block.cc",
        "frame.cc",
    ],
    hdrs = [
        "block_manager.h",
        "block.h",
        "frame.h",
    ],
    deps = [
        "//cyber/common",
        "//cyber/croutine",
    ],
)

cc_library(
    name = "profiler",
    hdrs = ["profiler.h"],
    # Todo(daohu527): Output profiler logs to a new file using below trick
    copts = [
        "-DMODULE_NAME=\\\"perf\\\"",
    ],
    deps = [
        ":block_manager",
    ],
)

cc_test(
    name = "profiler_test",
    size = "small",
    srcs = ["profiler_test.cc"],
    deps = [
        ":profiler",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

cpplint()
