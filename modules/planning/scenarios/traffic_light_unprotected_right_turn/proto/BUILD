load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("//tools:python_rules.bzl", "py_proto_library")
load("//tools/install:install.bzl", "install", "install_files")

package(default_visibility = ["//visibility:public"])

install_files(
    name = "py_pb_traffic_light_unprotected_right_turn",
    dest = "traffic-light-unprotected-right-turn/python/proto", 
    files = [
        ":traffic_light_unprotected_right_turn_py_pb2",
    ]
)

install_files(
    name = "pb_hdrs_traffic_light_unprotected_right_turn",
    dest = "traffic-light-unprotected-right-turn/include/proto", 
    files = [
        ":traffic_light_unprotected_right_turn_cc_proto",
    ]
)

cc_proto_library(
    name = "traffic_light_unprotected_right_turn_cc_proto",
    deps = [
        ":traffic_light_unprotected_right_turn_proto",
    ],
)

proto_library(
    name = "traffic_light_unprotected_right_turn_proto",
    srcs = ["traffic_light_unprotected_right_turn.proto"],
    deps = [
        "//modules/planning/planning_base/scenario_base/proto:creep_stage_proto",
    ],

)

py_proto_library(
    name = "traffic_light_unprotected_right_turn_py_pb2",
    deps = [
        ":traffic_light_unprotected_right_turn_proto",
    ],
)