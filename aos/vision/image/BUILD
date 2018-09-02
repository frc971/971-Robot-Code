package(default_visibility = ["//visibility:public"])

load("//tools/build_rules:protobuf.bzl", "proto_cc_library")

cc_library(
    name = "image_types",
    hdrs = ["image_types.h"],
    compatible_with = [
        "//tools:armhf-debian",
    ],
    deps = [
        "//aos/common/logging",
    ],
)

proto_cc_library(
    name = "camera_params",
    src = "camera_params.proto",
    compatible_with = [
        "//tools:armhf-debian",
    ],
)

cc_library(
    name = "reader",
    srcs = ["reader.cc"],
    hdrs = [
        "V4L2.h",
        "reader.h",
    ],
    compatible_with = [
        "//tools:armhf-debian",
    ],
    deps = [
        ":camera_params",
        ":image_types",
        "//aos/common:time",
        "//aos/common/logging",
    ],
)

cc_library(
    name = "jpeg_routines",
    srcs = ["jpeg_routines.cc"],
    hdrs = ["jpeg_routines.h"],
    compatible_with = [
        "//tools:armhf-debian",
    ],
    deps = [
        ":image_types",
        "//aos/common/logging",
        "//third_party/libjpeg",
    ],
)

cc_library(
    name = "image_stream",
    hdrs = ["image_stream.h"],
    compatible_with = [
        "//tools:armhf-debian",
    ],
    deps = [
        "//aos/vision/events:epoll_events",
        "//aos/vision/image:reader",
    ],
)
