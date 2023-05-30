load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("//tools:python_rules.bzl", "py_proto_library")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "learning_model_inference_cc_proto",
    deps = [
        ":learning_model_inference_proto",
    ],
)

proto_library(
    name = "learning_model_inference_proto",
    srcs = ["learning_model_inference.proto"],
)

py_proto_library(
    name = "learning_model_inference_py_pb2",
    deps = [
        ":learning_model_inference_proto",
    ],
)