load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("//tools/install:install.bzl", "install", "install_files")
load("//tools:python_rules.bzl", "py_proto_library")

package(default_visibility = ["//visibility:public"])

install_files(
    name = "py_pb_crosswalk",
    dest = "planning-traffic-rules-crosswalk/python/proto", 
    files = [
        ":crosswalk_py_pb2",
    ]
)

install_files(
    name = "pb_hdrs_crosswalk",
    dest = "planning-traffic-rules-crosswalk/include/proto", 
    files = [
        ":crosswalk_cc_proto",
    ]
)

cc_proto_library(
    name = "crosswalk_cc_proto",
    deps = [
        ":crosswalk_proto",
    ],
)

proto_library(
    name = "crosswalk_proto",
    srcs = ["crosswalk.proto"],
)

py_proto_library(
    name = "crosswalk_py_pb2",
    deps = [
        ":crosswalk_proto",
    ],
)