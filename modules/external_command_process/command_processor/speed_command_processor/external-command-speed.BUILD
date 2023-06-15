load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "external-command-speed",
    includes = [
        "include",
    ],
    srcs = glob(["lib/*.so*"]),
    hdrs = glob(["include/**/*"]),
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
)

cpplint()
