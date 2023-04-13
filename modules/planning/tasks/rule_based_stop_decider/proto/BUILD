load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("//tools/install:install.bzl", "install", "install_files")
load("//tools:python_rules.bzl", "py_proto_library")

package(default_visibility = ["//visibility:public"])

install_files(
    name = "py_pb_rule_based_stop_decider",
    dest = "planning-task-rule-based-stop-decider/python/proto", 
    files = [
        ":rule_based_stop_decider_py_pb2",
    ]
)

install_files(
    name = "pb_hdrs_rule_based_stop_decider",
    dest = "planning-task-rule-based-stop-decider/include/proto", 
    files = [
        ":rule_based_stop_decider_cc_proto",
    ]
)

cc_proto_library(
    name = "rule_based_stop_decider_cc_proto",
    deps = [
        ":rule_based_stop_decider_proto",
    ],
)

proto_library(
    name = "rule_based_stop_decider_proto",
    srcs = ["rule_based_stop_decider.proto"],
)

py_proto_library(
    name = "rule_based_stop_decider_py_pb2",
    deps = [
        ":rule_based_stop_decider_proto",
    ],
)