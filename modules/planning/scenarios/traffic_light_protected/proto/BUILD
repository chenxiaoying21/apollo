load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("//tools:python_rules.bzl", "py_proto_library")
load("//tools/install:install.bzl", "install", "install_files")

package(default_visibility = ["//visibility:public"])

install_files(
    name = "py_pb_traffic_light_protected",
    dest = "traffic-light-protected/python/proto",
    files = [
        ":traffic_light_protected_py_pb2",
    ]
)

install_files(
    name = "pb_hdrs_traffic_light_protected",
    dest = "traffic-light-unprotected-left-turn/include/proto",
    files = [
        ":traffic_light_protected_cc_proto",
    ]
)

cc_proto_library(
    name = "traffic_light_protected_cc_proto",
    deps = [
        ":traffic_light_protected_proto",
    ],
)

proto_library(
    name = "traffic_light_protected_proto",
    srcs = ["traffic_light_protected.proto"],
)

py_proto_library(
    name = "traffic_light_protected_py_pb2",
    deps = [
        ":traffic_light_protected_proto",
    ],
)
