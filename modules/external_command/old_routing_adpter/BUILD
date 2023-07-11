load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install",
     "install_files", "install_src_files")

package(default_visibility=["//visibility:public"])

cc_library(
    name="old_routing_adpter_lib",
    srcs=["old_routing_adpter.cc"],
    hdrs=["old_routing_adpter.h"],
    copts = [
        "-DMODULE_NAME=\\\"old_routing_adpter\\\"",
    ],
    deps=[
        "//cyber",
        "//modules/external_command/old_routing_adpter/proto:old_routing_adpter_cc_proto",
        "//modules/external_command/old_routing_adpter/proto:routing_cc_proto",
        "//modules/map/hdmap:hdmap_util",
        "//modules/common_msgs/external_command_msgs:lane_follow_command_cc_proto",
        "//modules/common_msgs/external_command_msgs:valet_parking_command_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_command_cc_proto",
        "//modules/common_msgs/external_command_msgs:command_status_cc_proto",
    ],
    alwayslink=True,
)

cc_binary(
    name="libold_routing_adpter.so",
    linkshared=True,
    linkstatic=True,
    deps=[":old_routing_adpter_lib"],
)

filegroup(
    name="runtime_data",
    srcs=glob([
        "conf/*.pb.txt",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

install(
    name="install",
    data_dest="old-routing-adpter",
    data=[
        ":runtime_data",
        "cyberfile.xml",
        "old-routing-adpter.BUILD",
    ],
    targets=[
        ":libold_routing_adpter.so",
    ],
    deps=[
        ":pb_old_routing_adpter",
        ":pb_hdrs",
    ],
)

install(
    name="pb_hdrs",
    data_dest="old-routing-adpter/include",
    data=[
        "//modules/external_command/old_routing_adpter/proto:old_routing_adpter_cc_proto",
    ],
)

install_files(
    name="pb_old_routing_adpter",
    dest="old-routing-adpter",
    files=[
        "//modules/external_command/old_routing_adpter/proto:old_routing_adpter_py_pb2",
    ],
)

install_src_files(
    name="install_src",
    deps=[
        ":install_old_routing_adpter_src",
        ":install_old_routing_adpter_hdrs"
    ],
)

install_src_files(
    name="install_old_routing_adpter_src",
    src_dir=["."],
    dest="old-routing-adpter/src",
    filter="*",
)

install_src_files(
    name="install_old_routing_adpter_hdrs",
    src_dir=["."],
    dest="old-routing-adpter/include",
    filter="*.h",
)

cpplint()
