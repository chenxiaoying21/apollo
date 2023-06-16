load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install",
     "install_files", "install_src_files")

package(default_visibility=["//visibility:public"])

cc_library(
    name="external_command_process_component_lib",
    srcs=["external_command_process_component.cc"],
    hdrs=["external_command_process_component.h"],
    copts = ["-DMODULE_NAME=\\\"external_command_process\\\""],
    deps=[
        "//cyber",
        "//modules/external_command_process/process_component/proto:process_component_config_cc_proto",
        "//modules/common_msgs/external_command_msgs:command_status_cc_proto",
        "//modules/external_command_process/command_processor/command_processor_base:command_processor_base_lib",
        "//modules/external_command_process/command_processor/command_processor_base/proto:command_processor_config_cc_proto",
    ],
    alwayslink=True
)

cc_binary(
    name="libexternal_command_process_component.so",
    linkshared=True,
    linkstatic=True,
    deps=[":external_command_process_component_lib"],
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
    data_dest="external-command-process",
    data=[
        ":runtime_data",
        "cyberfile.xml",
        "external-command-process.BUILD",
    ],
    targets=[
        ":libexternal_command_process_component.so",
    ],
    deps=[
        ":pb_process_component",
        ":pb_hdrs",
    ],
)

install(
    name="pb_hdrs",
    data_dest="external-command-process/include",
    data=[
        "//modules/external_command_process/process_component/proto:process_component_config_cc_proto",
    ],
)

install_files(
    name="pb_process_component",
    dest="external-command-process",
    files=[
        "//modules/external_command_process/process_component/proto:process_component_config_py_pb2",
    ],
)

install_src_files(
    name="install_src",
    deps=[
        ":install_process_component_src",
        ":install_process_component_hdrs"
    ],
)

install_src_files(
    name="install_process_component_src",
    src_dir=["."],
    dest="external-command-process/src",
    filter="*",
)

install_src_files(
    name="install_process_component_hdrs",
    src_dir=["."],
    dest="external-command-process/include",
    filter="*.h",
)

cpplint()
