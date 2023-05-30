load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install", "install_files", "install_plugin", "install_src_files")
load("//tools:apollo.bzl", "cyber_plugin_description")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    data_dest = "planning-task-reuse-path",
    data = [
        ":cyberfile.xml",
        ":planning-task-reuse-path.BUILD",
    ],
    deps = [
        "install_plugin_reuse_path",
    ],
)

install_plugin(
    name = "install_plugin_reuse_path",
    data = [
    ] + glob(["conf/**"]),
    description = ":plugins.xml",
    plugin = ":libreuse_path.so",
)

cyber_plugin_description(
    name = "plugin_reuse_path_description",
    description = ":plugins.xml",
    plugin = ":libreuse_path.so",
)

cc_binary(
    name = "libreuse_path.so",
    linkshared = True,
    linkstatic = True,
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [":reuse_path_lib"],
)

cc_library(
    name = "reuse_path_lib",
    srcs = ["reuse_path.cc"],
    hdrs = ["reuse_path.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/map/pnc_map",
        "//modules/planning/planning_base/common:planning_context",
        "//modules/planning/planning_base/common:planning_gflags",
        "//modules/planning/planning_base/common:reference_line_info",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/planning_base/task_base/common:path_generation",
        "//modules/planning/tasks/reuse_path/proto:reuse_path_cc_proto",
        "//modules/planning/planning_base/task_base/common/path_util:path_util"
    ],
    alwayslink = True,
)

cpplint()
