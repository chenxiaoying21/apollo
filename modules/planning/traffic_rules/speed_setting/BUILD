load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install", "install_files", "install_plugin", "install_src_files")
load("//tools:apollo.bzl", "cyber_plugin_description")

package(default_visibility = ["//visibility:public"])
PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

install(
    name = "install",
    data_dest = "planning-traffic-rules-speed-setting",
    data = [
        ":cyberfile.xml",
        ":planning-traffic-rules-speed-setting.BUILD",
    ],
    deps = [
        "install_plugin",
    ],
)

install_plugin(
    name = "install_plugin",
    data = [
    ] + glob(["conf/**"]),
    description = ":plugins.xml",
    plugin = ":libspeed_setting.so",
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
    dest = "planning-traffic-rules-speed-setting/src",
    filter = "*",
)

install_src_files(
    name = "install_hdrs",
    src_dir = ["."],
    dest = "planning-traffic-rules-speed-setting/include",
    filter = "*.h",
)

cyber_plugin_description(
    name = "plugin_libspeed_setting_lib_description",
    description = ":plugins.xml",
    plugin = ":libspeed_setting.so",
)

cc_binary(
    name = "libspeed_setting.so",
    linkshared = True,
    linkstatic = True,
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [":speed_setting_lib"],
)

cc_library(
    name = "speed_setting_lib",
    srcs = ["speed_setting.cc"],
    hdrs = ["speed_setting.h"],
    alwayslink = True,   
    copts = PLANNING_COPTS,
    deps = [
        "//cyber",
        "//modules/planning/planning_base/common:frame",
        "//modules/planning/planning_base/common:reference_line_info",
        "//modules/planning/planning_base/traffic_rules_base:traffic_rules",
        "//modules/common_msgs/external_command_msgs:speed_command_cc_proto",
    ],
)

cpplint()
