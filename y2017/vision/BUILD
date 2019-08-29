load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")
load("//tools/build_rules:gtk_dependent.bzl", "gtk_dependent_cc_binary", "gtk_dependent_cc_library")
load("@com_google_protobuf//:protobuf.bzl", "cc_proto_library")

package(default_visibility = ["//visibility:public"])

flatbuffer_cc_library(
    name = "vision_fbs",
    srcs = [
        "vision.fbs",
    ],
    gen_reflections = 1,
    visibility = ["//visibility:public"],
)

cc_proto_library(
    name = "vision_result",
    srcs = ["vision_result.proto"],
)

cc_proto_library(
    name = "vision_config",
    srcs = ["vision_config.proto"],
    deps = [
        "//aos/vision/image:camera_params",
    ],
)

cc_binary(
    name = "target_sender",
    srcs = [
        "target_sender.cc",
    ],
    deps = [
        ":target_finder",
        ":vision_config",
        ":vision_result",
        "//aos/logging",
        "//aos/logging:implementations",
        "//aos/time",
        "//aos/vision/blob:codec",
        "//aos/vision/blob:find_blob",
        "//aos/vision/blob:range_image",
        "//aos/vision/blob:threshold",
        "//aos/vision/events:epoll_events",
        "//aos/vision/events:socket_types",
        "//aos/vision/events:udp",
        "//aos/vision/image:image_stream",
        "//aos/vision/image:jpeg_routines",
        "//aos/vision/image:reader",
    ],
)

cc_binary(
    name = "target_receiver",
    srcs = [
        "target_receiver.cc",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":target_finder",
        ":vision_fbs",
        ":vision_result",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//aos/logging",
        "//aos/mutex",
        "//aos/time",
        "//aos/vision/events:udp",
    ],
)

cc_library(
    name = "target_finder",
    srcs = ["target_finder.cc"],
    hdrs = ["target_finder.h"],
    deps = [
        "//aos/vision/blob:threshold",
        "//aos/vision/blob:transpose",
        "//aos/vision/debug:overlay",
        "//aos/vision/math:vector",
    ],
)

gtk_dependent_cc_binary(
    name = "debug_viewer",
    srcs = ["debug_viewer.cc"],
    deps = [
        ":target_finder",
        "//aos/vision/blob:move_scale",
        "//aos/vision/blob:threshold",
        "//aos/vision/blob:transpose",
        "//aos/vision/debug:debug_framework",
        "//aos/vision/math:vector",
    ],
)
