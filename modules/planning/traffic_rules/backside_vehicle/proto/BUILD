load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("//tools/install:install.bzl", "install", "install_files")
load("//tools:python_rules.bzl", "py_proto_library")

package(default_visibility = ["//visibility:public"])

install_files(
    name = "py_pb_backside_vehicle",
    dest = "planning-traffic-rules-backside-vehicle/python/proto", 
    files = [
        ":backside_vehicle_py_pb2",
    ]
)

install_files(
    name = "pb_hdrs_backside_vehicle",
    dest = "planning-traffic-rules-backside-vehicle/include/proto", 
    files = [
        ":backside_vehicle_cc_proto",
    ]
)

cc_proto_library(
    name = "backside_vehicle_cc_proto",
    deps = [
        ":backside_vehicle_proto",
    ],
)

proto_library(
    name = "backside_vehicle_proto",
    srcs = ["backside_vehicle.proto"],
)

py_proto_library(
    name = "backside_vehicle_py_pb2",
    deps = [
        ":backside_vehicle_proto",
    ],
)