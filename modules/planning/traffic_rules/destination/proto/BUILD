load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("//tools/install:install.bzl", "install", "install_files")
load("//tools:python_rules.bzl", "py_proto_library")

package(default_visibility = ["//visibility:public"])

install_files(
    name = "py_pb_destination",
    dest = "planning-traffic-rules-destination/python/proto", 
    files = [
        ":destination_py_pb2",
    ]
)

install_files(
    name = "pb_hdrs_destination",
    dest = "planning-traffic-rules-destination/include/proto", 
    files = [
        ":destination_cc_proto",
    ]
)

cc_proto_library(
    name = "destination_cc_proto",
    deps = [
        ":destination_proto",
    ],
)

proto_library(
    name = "destination_proto",
    srcs = ["destination.proto"],
)

py_proto_library(
    name = "destination_py_pb2",
    deps = [
        ":destination_proto",
    ],
)