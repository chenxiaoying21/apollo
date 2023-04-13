load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("//tools/install:install.bzl", "install", "install_files")
load("//tools:python_rules.bzl", "py_proto_library")

package(default_visibility = ["//visibility:public"])

install_files(
    name = "py_pb_speed_bounds_decider",
    dest = "planning-task-speed-bounds-decider/python/proto", 
    files = [
        ":speed_bounds_decider_py_pb2",
    ]
)

install_files(
    name = "pb_hdrs_speed_bounds_decider",
    dest = "planning-task-speed-bounds-decider/include/proto", 
    files = [
        ":speed_bounds_decider_cc_proto",
    ]
)

cc_proto_library(
    name = "speed_bounds_decider_cc_proto",
    deps = [
        ":speed_bounds_decider_proto",
    ],
)

proto_library(
    name = "speed_bounds_decider_proto",
    srcs = ["speed_bounds_decider.proto"],
)

py_proto_library(
    name = "speed_bounds_decider_py_pb2",
    deps = [
        ":speed_bounds_decider_proto",
    ],
)