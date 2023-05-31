load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("//tools/install:install.bzl", "install", "install_files")
load("//tools:python_rules.bzl", "py_proto_library")

package(default_visibility = ["//visibility:public"])

install_files(
    name = "py_pb_keepclear",
    dest = "planning-traffic-rules-keepclear/python/proto", 
    files = [
        ":keepclear_py_pb2",
    ]
)

install_files(
    name = "pb_hdrs_keepclear",
    dest = "planning-traffic-rules-keepclear/include/proto", 
    files = [
        ":keepclear_cc_proto",
    ]
)

cc_proto_library(
    name = "keepclear_cc_proto",
    deps = [
        ":keepclear_proto",
    ],
)

proto_library(
    name = "keepclear_proto",
    srcs = ["keepclear.proto"],
)

py_proto_library(
    name = "keepclear_py_pb2",
    deps = [
        ":keepclear_proto",
    ],
)