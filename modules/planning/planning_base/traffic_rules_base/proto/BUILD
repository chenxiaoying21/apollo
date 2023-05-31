load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("//tools/install:install.bzl", "install", "install_files")
load("//tools:python_rules.bzl", "py_proto_library")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "traffic_rules_cc_proto",
    deps = [
        ":traffic_rules_proto",
    ],
)

proto_library(
    name = "traffic_rules_proto",
    srcs = ["traffic_rules_pipeline.proto"],
    deps = [
        "//modules/planning/planning_base/proto:plugin_declare_info_proto",
    ]
)

py_proto_library(
    name = "traffic_rules_py_pb2",
    deps = [
        ":traffic_rules_proto",
    ],
)