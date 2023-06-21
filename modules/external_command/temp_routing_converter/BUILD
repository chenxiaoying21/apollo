load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install",
     "install_files", "install_src_files")

package(default_visibility=["//visibility:public"])

cc_library(
    name="temp_routing_converter_lib",
    srcs=["temp_routing_converter.cc"],
    hdrs=["temp_routing_converter.h"],
    copts = [
        "-DMODULE_NAME=\\\"temp_routing_converter\\\"",
    ],
    deps=[
        "//cyber",
        "//modules/external_command/temp_routing_converter/proto:temp_routing_converter_cc_proto",
        "//modules/external_command/temp_routing_converter/proto:routing_cc_proto",
        "//modules/map/hdmap:hdmap_util",
        "//modules/common_msgs/external_command_msgs:lane_follow_command_cc_proto",
        "//modules/common_msgs/external_command_msgs:valet_parking_command_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_command_cc_proto",
        "//modules/common_msgs/external_command_msgs:command_status_cc_proto",
    ],
    alwayslink=True,
)

cc_binary(
    name="libtemp_routing_converter.so",
    linkshared=True,
    linkstatic=True,
    deps=[":temp_routing_converter_lib"],
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
    data_dest="temp-routing-converter",
    data=[
        ":runtime_data",
        "cyberfile.xml",
        "temp-routing-converter.BUILD",
    ],
    targets=[
        ":libtemp_routing_converter.so",
    ],
    deps=[
        ":pb_temp_routing_converter",
        ":pb_hdrs",
    ],
)

install(
    name="pb_hdrs",
    data_dest="temp-routing-converter/include",
    data=[
        "//modules/external_command/temp_routing_converter/proto:temp_routing_converter_cc_proto",
    ],
)

install_files(
    name="pb_temp_routing_converter",
    dest="temp-routing-converter",
    files=[
        "//modules/external_command/temp_routing_converter/proto:temp_routing_converter_py_pb2",
    ],
)

install_src_files(
    name="install_src",
    deps=[
        ":install_temp_routing_converter_src",
        ":install_temp_routing_converter_hdrs"
    ],
)

install_src_files(
    name="install_temp_routing_converter_src",
    src_dir=["."],
    dest="temp-routing-converter/src",
    filter="*",
)

install_src_files(
    name="install_temp_routing_converter_hdrs",
    src_dir=["."],
    dest="temp-routing-converter/include",
    filter="*.h",
)

cpplint()
