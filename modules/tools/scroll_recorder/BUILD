load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

SCROLL_RECORDER_COPTS = ['-DMODULE_NAME=\\"scroll_recorder\\"']

cc_library(
    name = "scroll_recorder_component_lib",
    srcs = ["scroll_recorder_component.cc"],
    hdrs = ["scroll_recorder_component.h"],
    copts = SCROLL_RECORDER_COPTS,
    deps = [
        "//cyber",
        "//cyber/tools/cyber_recorder:recorder",
        "//modules/tools/scroll_recorder/proto:scroll_recorder_conf_cc_proto",
    ],
)

cc_binary(
    name = "libscroll_recorder_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [
        ":scroll_recorder_component_lib",
    ],
)

install(
    name = "install",
    data = [
        ":runtime_data",
    ],
    targets = [
        ":libscroll_recorder_component.so",
    ],
    deps = [
        "//cyber:install",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

cpplint()
