load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("//tools/install:install.bzl", "install", "install_files")
load("//tools:python_rules.bzl", "py_proto_library")

package(default_visibility = ["//visibility:public"])

install_files(
    name = "py_pb_reference_line_end",
    dest = "planning-traffic-rules-reference-line-end/python/proto", 
    files = [
        ":reference_line_end_py_pb2",
    ]
)

install_files(
    name = "pb_hdrs_reference_line_end",
    dest = "planning-traffic-rules-reference-line-end/include/proto", 
    files = [
        ":reference_line_end_cc_proto",
    ]
)

cc_proto_library(
    name = "reference_line_end_cc_proto",
    deps = [
        ":reference_line_end_proto",
    ],
)

proto_library(
    name = "reference_line_end_proto",
    srcs = ["reference_line_end.proto"],
)

py_proto_library(
    name = "reference_line_end_py_pb2",
    deps = [
        ":reference_line_end_proto",
    ],
)