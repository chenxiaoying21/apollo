load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install",
     "install_files", "install_src_files")

package(default_visibility=["//visibility:public"])


cc_library(
    name="command_processor_base_lib",
    srcs=["command_processor_base.cc"],
    hdrs=["command_processor_base.h"],
    deps=[
        "//cyber",
        "//modules/common_msgs/external_command_msgs:command_status_cc_proto",
        "//modules/external_command_process/command_processor/command_processor_base/proto:command_processor_config_cc_proto",
    ],
    alwayslink=True
)

cc_binary(
    name="libcommand_processor_base.so",
    linkshared=True,
    linkstatic=True,
    deps=[":command_processor_base_lib"],
)

cc_library(
    name="motion_command_processor_base_lib",
    hdrs=["motion_command_processor_base.h"],
    deps=[
        "//cyber",
        "//modules/common_msgs/external_command_msgs:command_status_cc_proto",
        "//modules/external_command_process/command_processor/command_processor_base/proto:command_processor_config_cc_proto",
        "//modules/external_command_process/command_processor/command_processor_base/util:lane_way_tool",
        "//modules/external_command_process/command_processor/command_processor_base/util:message_reader",
        "//modules/routing:routing",
        "//modules/common_msgs/planning_msgs:planning_command_cc_proto",
        ":command_processor_base_lib",
    ],
    alwayslink=True
)

cc_binary(
    name="libmotion_command_processor_base.so",
    linkshared=True,
    linkstatic=True,
    deps=[":motion_command_processor_base_lib"],
)

install(
    name="install",
    data_dest="external-command-processor-base",
    data=[
        "cyberfile.xml",
        "external-command-processor-base.BUILD",
    ],
    targets=[
        ":libcommand_processor_base.so",
    ],
    deps=[
        ":pb_command_processor_base",
        ":pb_hdrs",
    ],
)

install(
    name="pb_hdrs",
    data_dest="external-command-processor-base/include",
    data=[
        "//modules/external_command_process/command_processor/command_processor_base/proto:command_processor_config_cc_proto",
    ],
)

install_files(
    name="pb_command_processor_base",
    dest="external-command-processor-base",
    files=[
        "//modules/external_command_process/command_processor/command_processor_base/proto:command_processor_config_py_pb2",
    ],
)

install_src_files(
    name="install_src",
    deps=[
        ":install_command_processor_base_src",
        ":install_command_processor_base_hdrs"
    ],
)

install_src_files(
    name="install_command_processor_base_src",
    src_dir=["."],
    dest="external-command-processor-base/src",
    filter="*",
)

install_src_files(
    name="install_command_processor_base_hdrs",
    src_dir=["."],
    dest="external-command-processor-base/include",
    filter="*.h",
)

cpplint()
