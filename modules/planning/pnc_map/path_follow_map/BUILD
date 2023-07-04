load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("@local_config_cuda//cuda:build_defs.bzl", "cuda_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install", "install_files", "install_plugin", "install_src_files")
load("//tools:apollo.bzl", "cyber_plugin_description")

package(default_visibility = ["//visibility:public"])

MAP_COPTS = ["-DMODULE_NAME=\\\"map\\\""]
PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

install(
    name = "install",
    library_dest = "map/lib",
    targets = [":libpath_follow_map.so"],
    visibility = ["//visibility:public"],
)

install_plugin(
    name = "install_plugin",
    description = ":plugins.xml",
    plugin = ":libpath_follow_map.so",
)

install_src_files(
    name = "install_src",
    deps = [
        ":install_all_src",
        ":install_hdrs"
    ],
)

install_src_files(
    name = "install_all_src",
    src_dir = ["."],
    dest = "planning-path-follow-map/src",
    filter = "*",
)

install_src_files(
    name = "install_hdrs",
    src_dir = ["."],
    dest = "planning-path-follow-map/include",
    filter = "*.h",
)

cyber_plugin_description(
    name = "plugin_path_follow_map_description",
    description = ":plugins.xml",
    plugin = ":libpath_follow_map.so",
)

cc_binary(
    name = "libpath_follow_map.so",
    srcs = [
        "path_follow_map.cc",
        "path_follow_map.h",
    ],
    linkshared = True,
    linkstatic = True,
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        "//modules/map/pnc_map:pnc_map",
        "//cyber",
        "//modules/common_msgs/map_msgs:map_id_cc_proto",
        "//modules/common_msgs/routing_msgs:routing_cc_proto",
        "//modules/common/vehicle_state/proto:vehicle_state_cc_proto",
        "//modules/common_msgs/external_command_msgs:path_follow_command_cc_proto",
        "//modules/common/configs:vehicle_config_helper",
        "//cyber/plugin_manager:plugin_manager",
        "@com_google_absl//:absl",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_library(
    name = "path_follow_map",
    srcs = ["libpath_follow_map.so"],
    hdrs = [
        "path_follow_map.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        "//cyber",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common_msgs/map_msgs:map_lane_cc_proto",
        "//modules/common_msgs/routing_msgs:routing_cc_proto",
        "//modules/common/vehicle_state/proto:vehicle_state_cc_proto",
        "//modules/common_msgs/external_command_msgs:path_follow_command_cc_proto",
        "//modules/common/configs:vehicle_config_helper",
        "//cyber/plugin_manager:plugin_manager",
        "@com_google_absl//:absl",
        "@com_google_protobuf//:protobuf",
    ],
)

cpplint()
