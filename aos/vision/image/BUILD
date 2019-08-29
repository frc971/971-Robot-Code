package(default_visibility = ["//visibility:public"])

load("@com_google_protobuf//:protobuf.bzl", "cc_proto_library")

cc_library(
    name = "image_types",
    hdrs = ["image_types.h"],
    deps = [
        "@com_google_absl//absl/strings",
    ],
)

cc_proto_library(
    name = "camera_params",
    srcs = ["camera_params.proto"],
)

cc_library(
    name = "reader",
    srcs = ["reader.cc"],
    hdrs = [
        "V4L2.h",
        "reader.h",
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
    deps = [
        ":image_types",
        "//aos/logging",
        "//third_party/libjpeg",
    ],
)

cc_library(
    name = "image_dataset",
    srcs = ["image_dataset.cc"],
    hdrs = ["image_dataset.h"],
    deps = [
        ":image_types",
    ],
)

cc_library(
    name = "image_stream",
    srcs = ["image_stream.cc"],
    hdrs = ["image_stream.h"],
    deps = [
        "//aos/logging",
        "//aos/vision/events:epoll_events",
        "//aos/vision/image:reader",
    ],
)
