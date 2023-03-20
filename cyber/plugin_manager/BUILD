load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "plugin_manager",
    srcs = [
        "plugin_manager.cc",
    ],
    hdrs = [
        "plugin_manager.h",
    ],
    deps = [
        "//cyber/class_loader:class_loader_manager",
        "//cyber/common",
        "@tinyxml2",
    ],
    alwayslink = True,
)

cpplint()
