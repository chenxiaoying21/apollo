load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("//tools/install:install.bzl", "install", "install_files")
load("//tools:python_rules.bzl", "py_proto_library")

package(default_visibility = ["//visibility:public"])

install_files(
    name = "py_pb_rerouting",
    dest = "planning-traffic-rules-rerouting/python/proto", 
    files = [
        ":rerouting_py_pb2",
    ]
)

install_files(
    name = "pb_hdrs_rerouting",
    dest = "planning-traffic-rules-rerouting/include/proto", 
    files = [
        ":rerouting_cc_proto",
    ]
)

cc_proto_library(
    name = "rerouting_cc_proto",
    deps = [
        ":rerouting_proto",
    ],
)

proto_library(
    name = "rerouting_proto",
    srcs = ["rerouting.proto"],
)

py_proto_library(
    name = "rerouting_py_pb2",
    deps = [
        ":rerouting_proto",
    ],
)