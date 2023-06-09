load("//tools/install:install.bzl", "install", "install_files", "install_src_files")

install(
    name = "install",
    deps = [
        "//modules/planning/traffic_rules/yield_sign:install",
        "//modules/planning/traffic_rules/yield_sign:install_plugin_yield_sign",
    ],
)

install_src_files(
    name = "install_src",
    deps = [
        "//modules/planning/traffic_rules/yield_sign:install_src",
    ],
)


