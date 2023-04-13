load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("//tools/install:install.bzl", "install", "install_files")
load("//tools:python_rules.bzl", "py_proto_library")

package(default_visibility = ["//visibility:public"])

install_files(
    name = "py_pb_rss_decider",
    dest = "planning-rss-decider/python/proto", 
    files = [
        ":rss_decider_py_pb2",
    ]
)

install_files(
    name = "pb_hdrs_rss_decider",
    dest = "planning-rss-decider/include/proto", 
    files = [
        ":rss_decider_cc_proto",
    ]
)

cc_proto_library(
    name = "rss_decider_cc_proto",
    deps = [
        ":rss_decider_proto",
    ],
)

proto_library(
    name = "rss_decider_proto",
    srcs = ["rss_decider.proto"],
)

py_proto_library(
    name = "rss_decider_py_pb2",
    deps = [
        ":rss_decider_proto",
    ],
)