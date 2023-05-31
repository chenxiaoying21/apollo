load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("//tools/install:install.bzl", "install", "install_files")
load("//tools:python_rules.bzl", "py_proto_library")

package(default_visibility = ["//visibility:public"])

install_files(
    name = "py_pb_stop_sign",
    dest = "planning-traffic-rules-stop-sign/python/proto", 
    files = [
        ":stop_sign_py_pb2",
    ]
)

install_files(
    name = "pb_hdrs_stop_sign",
    dest = "planning-traffic-rules-stop-sign/include/proto", 
    files = [
        ":stop_sign_cc_proto",
    ]
)

cc_proto_library(
    name = "stop_sign_cc_proto",
    deps = [
        ":stop_sign_proto",
    ],
)

proto_library(
    name = "stop_sign_proto",
    srcs = ["stop_sign.proto"],
)

py_proto_library(
    name = "stop_sign_py_pb2",
    deps = [
        ":stop_sign_proto",
    ],
)