load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("//tools:python_rules.bzl", "py_proto_library")
load("//tools/install:install.bzl", "install", "install_files")

package(default_visibility = ["//visibility:public"])

install_files(
    name = "py_pb_free_space",
    dest = "planning-scenario-free-space/python/proto", 
    files = [
        ":free_space_py_pb2",
    ]
)

install_files(
    name = "pb_hdrs_free_space",
    dest = "planning-scenario-free-space/include/proto", 
    files = [
        ":free_space_cc_proto",
    ]
)

cc_proto_library(
    name = "free_space_cc_proto",
    deps = [
        ":free_space_proto",
    ],
)

proto_library(
    name = "free_space_proto",
    srcs = ["free_space_scenario.proto"],
)

py_proto_library(
    name = "free_space_py_pb2",
    deps = [
        ":free_space_proto",
    ],
)