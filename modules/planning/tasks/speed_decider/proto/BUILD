load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("//tools/install:install.bzl", "install", "install_files")
load("//tools:python_rules.bzl", "py_proto_library")

package(default_visibility = ["//visibility:public"])

install_files(
    name = "py_pb_speed_decider",
    dest = "planning-task-speed-decider/python/proto", 
    files = [
        ":speed_decider_py_pb2",
    ]
)

install_files(
    name = "pb_hdrs_speed_decider",
    dest = "planning-task-speed-decider/include/proto", 
    files = [
        ":speed_decider_cc_proto",
    ]
)

cc_proto_library(
    name = "speed_decider_cc_proto",
    deps = [
        ":speed_decider_proto",
    ],
)

proto_library(
    name = "speed_decider_proto",
    srcs = ["speed_decider.proto"],
)

py_proto_library(
    name = "speed_decider_py_pb2",
    deps = [
        ":speed_decider_proto",
    ],
)