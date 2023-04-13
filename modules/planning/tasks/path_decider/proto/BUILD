load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("//tools/install:install.bzl", "install", "install_files")
load("//tools:python_rules.bzl", "py_proto_library")

package(default_visibility = ["//visibility:public"])

install_files(
    name = "py_pb_path_decider",
    dest = "planning-task-path-decider/python/proto", 
    files = [
        ":path_decider_py_pb2",
    ]
)

install_files(
    name = "pb_hdrs_path_decider",
    dest = "planning-task-path-decider/include/proto", 
    files = [
        ":path_decider_cc_proto",
    ]
)

cc_proto_library(
    name = "path_decider_cc_proto",
    deps = [
        ":path_decider_proto",
    ],
)

proto_library(
    name = "path_decider_proto",
    srcs = ["path_decider.proto"],
)

py_proto_library(
    name = "path_decider_py_pb2",
    deps = [
        ":path_decider_proto",
    ],
)