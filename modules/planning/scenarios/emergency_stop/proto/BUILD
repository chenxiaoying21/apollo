load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("//tools:python_rules.bzl", "py_proto_library")
load("//tools/install:install.bzl", "install", "install_files")

package(default_visibility = ["//visibility:public"])

install_files(
    name = "py_pb_emergency_stop",
    dest = "planning-scenario-emergency-stop/python/proto", 
    files = [
        ":emergency_stop_py_pb2",
    ]
)

install_files(
    name = "pb_hdrs_emergency_stop",
    dest = "planning-scenario-emergency-stop/include/proto", 
    files = [
        ":emergency_stop_cc_proto",
    ]
)

cc_proto_library(
    name = "emergency_stop_cc_proto",
    deps = [
        ":emergency_stop_proto",
    ],
)

proto_library(
    name = "emergency_stop_proto",
    srcs = ["emergency_stop.proto"],
)

py_proto_library(
    name = "emergency_stop_py_pb2",
    deps = [
        ":emergency_stop_proto",
    ],
)