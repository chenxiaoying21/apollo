load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("//tools/platform:build_defs.bzl", "if_gpu")
load("//tools/install:install.bzl", "install", "install_files")
load("//tools:python_rules.bzl", "py_proto_library")

package(default_visibility = ["//visibility:public"])

install_files(
    name = "py_pb_planning",
    dest = if_gpu(
        "planning-gpu/python/modules/planning/planning_base/traffic_rules_base/proto",
        "planning/python/modules/planning/planning_base/traffic_rules_base/proto",
    ),
    files = [
        ":traffic_rules_py_pb2",
    ]
)

install_files(
    name = "pb_hdrs_planning",
    dest = if_gpu(
        "planning-gpu/include/traffic_rules_base/proto",
        "planning/include/traffic_rules_base/proto",
    ),
    files = [
        ":traffic_rules_cc_proto",
    ]
)

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
