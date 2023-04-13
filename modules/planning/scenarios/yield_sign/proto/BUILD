load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("//tools:python_rules.bzl", "py_proto_library")
load("//tools/install:install.bzl", "install", "install_files")

package(default_visibility = ["//visibility:public"])

install_files(
    name = "py_pb_yield_sign",
    dest = "planning-scenario-yield-sign/python/proto", 
    files = [
        ":yield_sign_py_pb2",
    ]
)

install_files(
    name = "pb_hdrs_yield_sign",
    dest = "planning-scenario-yield-sign/include/proto", 
    files = [
        ":yield_sign_cc_proto",
    ]
)

cc_proto_library(
    name = "yield_sign_cc_proto",
    deps = [
        ":yield_sign_proto",
    ],
)

proto_library(
    name = "yield_sign_proto",
    srcs = ["yield_sign.proto"],
    deps = [
        "//modules/planning/proto:creep_decider_proto",
    ],
)

py_proto_library(
    name = "yield_sign_py_pb2",
    deps = [
        ":yield_sign_proto",
    ],
)