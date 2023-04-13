load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("//tools:python_rules.bzl", "py_proto_library")
load("//tools/install:install.bzl", "install", "install_files")

package(default_visibility = ["//visibility:public"])

install_files(
    name = "py_pb_park_and_go",
    dest = "planning-scenario-park-and-go/python/proto", 
    files = [
        ":park_and_go_py_pb2",
    ]
)

install_files(
    name = "pb_hdrs_park_and_go",
    dest = "planning-scenario-park-and-go/include/proto", 
    files = [
        ":park_and_go_cc_proto",
    ]
)

cc_proto_library(
    name = "park_and_go_cc_proto",
    deps = [
        ":park_and_go_proto",
    ],
)

proto_library(
    name = "park_and_go_proto",
    srcs = ["park_and_go.proto"],
)

py_proto_library(
    name = "park_and_go_py_pb2",
    deps = [
        ":park_and_go_proto",
    ],
)