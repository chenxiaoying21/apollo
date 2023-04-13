load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("//tools:python_rules.bzl", "py_proto_library")
load("//tools/install:install.bzl", "install", "install_files")

package(default_visibility = ["//visibility:public"])

install_files(
    name = "py_pb_valet_parking",
    dest = "planning-scenario-valet-parking/python/proto", 
    files = [
        ":valet_parking_py_pb2",
    ]
)

install_files(
    name = "pb_hdrs_valet_parking",
    dest = "planning-scenario-valet-parking/include/proto", 
    files = [
        ":valet_parking_cc_proto",
    ]
)

cc_proto_library(
    name = "valet_parking_cc_proto",
    deps = [
        ":valet_parking_proto",
    ],
)

proto_library(
    name = "valet_parking_proto",
    srcs = ["valet_parking.proto"],
)

py_proto_library(
    name = "valet_parking_py_pb2",
    deps = [
        ":valet_parking_proto",
    ],
)