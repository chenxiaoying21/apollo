load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("//tools/install:install.bzl", "install", "install_files")
load("//tools:python_rules.bzl", "py_proto_library")

package(default_visibility = ["//visibility:public"])

install_files(
    name = "py_pb_path_reference_decider",
    dest = "planning-task-path_reference_decider/python/proto",
    files = [
        ":path_reference_decider_py_pb2",
    ]
)

install_files(
    name = "pb_hdrs_path_reference_decider",
    dest = "planning-task-path_reference_decider/include/proto",
    files = [
        ":path_reference_decider_cc_proto",
    ]
)

cc_proto_library(
    name = "path_reference_decider_cc_proto",
    deps = [
        ":path_reference_decider_proto",
    ],
)

proto_library(
    name = "path_reference_decider_proto",
    srcs = ["path_reference_decider.proto"],
)

py_proto_library(
    name = "path_reference_decider_py_pb2",
    deps = [
        ":path_reference_decider_proto",
    ],
)
