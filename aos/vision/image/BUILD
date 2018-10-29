package(default_visibility = ["//visibility:public"])

load("@com_google_protobuf//:protobuf.bzl", "cc_proto_library")

cc_library(
    name = "image_types",
    hdrs = ["image_types.h"],
    compatible_with = [
        "//tools:armhf-debian",
    ],
    deps = [
        "//aos/logging",
    ],
)

cc_proto_library(
    name = "camera_params",
    srcs = ["camera_params.proto"],
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
        "//aos/logging",
        "//aos/time",
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
        "//aos/logging",
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
