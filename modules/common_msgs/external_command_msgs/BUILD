load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("//tools:python_rules.bzl", "py_proto_library")
load("//tools/install:install.bzl", "install", "install_files")

package(default_visibility = ["//visibility:public"])

# command status
cc_proto_library(
    name = "command_status_cc_proto",
    deps = [
        ":command_status_proto",
    ],
)

proto_library(
    name = "command_status_proto",
    srcs = ["command_status.proto"],
    deps = [
        "//modules/common_msgs/basic_msgs:header_proto",
    ],
)

py_proto_library(
    name = "command_status_py_pb2",
    deps = [
        ":command_status_proto",
    ],
)

# lane follow command
cc_proto_library(
    name = "lane_follow_command_cc_proto",
    deps = [
        ":lane_follow_command_proto",
    ],
)

proto_library(
    name = "lane_follow_command_proto",
    srcs = ["lane_follow_command.proto"],
    deps = [
        "//modules/common_msgs/basic_msgs:header_proto",
        ":geometry_proto",
        ":lane_segment_proto",
    ],
)

py_proto_library(
    name = "lane_follow_command_py_pb2",
    deps = [
        ":lane_follow_command_proto",
    ],
)

# parking command
cc_proto_library(
    name = "valet_parking_command_cc_proto",
    deps = [
        ":valet_parking_command_proto",
    ],
)

proto_library(
    name = "valet_parking_command_proto",
    srcs = ["valet_parking_command.proto"],
    deps = [
        "//modules/common_msgs/basic_msgs:header_proto",
        ":geometry_proto",
        ":lane_segment_proto",
    ],
)

py_proto_library(
    name = "valet_parking_command_py_pb2",
    deps = [
        ":valet_parking_command_proto",
    ],
)

# geometry
cc_proto_library(
    name = "geometry_cc_proto",
    deps = [
        ":geometry_proto",
    ],
)

proto_library(
    name = "geometry_proto",
    srcs = ["geometry.proto"],
)

py_proto_library(
    name = "geometry_py_pb2",
    deps = [
        ":geometry_proto",
    ],
)

# lane_segment
cc_proto_library(
    name = "lane_segment_cc_proto",
    deps = [
        ":lane_segment_proto",
    ],
)

proto_library(
    name = "lane_segment_proto",
    srcs = ["lane_segment.proto"],
)

py_proto_library(
    name = "lane_segment_py_pb2",
    deps = [
        ":lane_segment_proto",
    ],
)

# action command
cc_proto_library(
    name = "action_command_cc_proto",
    deps = [
        ":action_command_proto",
    ],
)

proto_library(
    name = "action_command_proto",
    srcs = [
        "action_command.proto",
    ],
    deps = [
        "//modules/common_msgs/basic_msgs:header_proto",
    ]
)

py_proto_library(
    name = "action_command_py_pb2",
    deps = [
        ":action_command_proto",
    ],
)

########################################################################
############################Command for park############################
########################################################################
# path follow command 
cc_proto_library(
    name = "path_follow_command_cc_proto",
    deps = [
        ":path_follow_command_proto",
    ],
)

proto_library(
    name = "path_follow_command_proto",
    srcs = ["path_follow_command.proto"],
    deps = [
        "//modules/common_msgs/basic_msgs:header_proto",
        ":geometry_proto",
    ],
)

py_proto_library(
    name = "path_follow_command_py_pb2",
    deps = [
        ":path_follow_command_proto",
    ],
)

# free space command
cc_proto_library(
    name = "free_space_command_cc_proto",
    deps = [
        ":free_space_command_proto",
    ],
)

proto_library(
    name = "free_space_command_proto",
    srcs = ["free_space_command.proto"],
    deps = [
        "//modules/common_msgs/basic_msgs:header_proto",
        ":geometry_proto",
    ],
)

py_proto_library(
    name = "free_space_command_py_pb2",
    deps = [
        ":free_space_command_proto",
    ],
)

# chassis command 
cc_proto_library(
    name = "chassis_command_cc_proto",
    deps = [
        ":chassis_command_proto",
    ],
)

proto_library(
    name = "chassis_command_proto",
    srcs = ["chassis_command.proto"],
    deps = [
        "//modules/common_msgs/basic_msgs:header_proto",
        "//modules/common_msgs/basic_msgs:vehicle_signal_proto",
        "@com_google_protobuf//:any_proto",
    ],
)

py_proto_library(
    name = "chassis_command_py_pb2",
    deps = [
        ":chassis_command_proto",
    ],
)

# speed command
cc_proto_library(
    name = "speed_command_cc_proto",
    deps = [
        ":speed_command_proto",
    ],
)

proto_library(
    name = "speed_command_proto",
    srcs = [
        "speed_command.proto",
    ],
    deps = [
        "//modules/common_msgs/basic_msgs:header_proto",
    ]
)

py_proto_library(
    name = "speed_command_py_pb2",
    deps = [
        ":speed_command_proto",
    ],
)