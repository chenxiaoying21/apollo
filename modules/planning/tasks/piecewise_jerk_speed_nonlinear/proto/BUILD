load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("//tools/install:install.bzl", "install", "install_files")
load("//tools:python_rules.bzl", "py_proto_library")

package(default_visibility = ["//visibility:public"])

install_files(
    name = "py_pb_piecewise_jerk_speed_nonlinear",
    dest = "planning-task-piecewise-jerk-speed-nonlinear/python/proto", 
    files = [
        ":piecewise_jerk_speed_nonlinear_py_pb2",
    ]
)

install_files(
    name = "pb_hdrs_piecewise_jerk_speed_nonlinear",
    dest = "planning-task-piecewise-jerk-speed-nonlinear/include/proto", 
    files = [
        ":piecewise_jerk_speed_nonlinear_cc_proto",
    ]
)

cc_proto_library(
    name = "piecewise_jerk_speed_nonlinear_cc_proto",
    deps = [
        ":piecewise_jerk_speed_nonlinear_proto",
    ],
)

proto_library(
    name = "piecewise_jerk_speed_nonlinear_proto",
    srcs = ["piecewise_jerk_speed_nonlinear.proto"],
)

py_proto_library(
    name = "piecewise_jerk_speed_nonlinear_py_pb2",
    deps = [
        ":piecewise_jerk_speed_nonlinear_proto",
    ],
)