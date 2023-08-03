load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")
load("//tools/build_rules:gtk_dependent.bzl", "gtk_dependent_cc_binary")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("@rules_proto//proto:defs.bzl", "proto_library")

package(default_visibility = ["//visibility:public"])

flatbuffer_cc_library(
    name = "vision_fbs",
    srcs = [
        "vision.fbs",
    ],
    gen_reflections = 1,
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

proto_library(
    name = "vision_result_proto",
    srcs = ["vision_result.proto"],
    target_compatible_with = ["@platforms//os:linux"],
)

proto_library(
    name = "vision_config_proto",
    srcs = ["vision_config.proto"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/vision/image:camera_params_proto",
    ],
)

cc_proto_library(
    name = "vision_result",
    target_compatible_with = ["@platforms//os:linux"],
    deps = ["vision_result_proto"],
)

cc_proto_library(
    name = "vision_config",
    target_compatible_with = ["@platforms//os:linux"],
    deps = ["vision_config_proto"],
)

cc_binary(
    name = "target_sender",
    srcs = [
        "target_sender.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":target_finder",
        ":vision_config",
        ":vision_result",
        "//aos/logging",
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
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":target_finder",
        ":vision_fbs",
        ":vision_result",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//aos/logging",
        "//aos/time",
        "//aos/vision/events:udp",
    ],
)

cc_library(
    name = "target_finder",
    srcs = ["target_finder.cc"],
    hdrs = ["target_finder.h"],
    target_compatible_with = ["@platforms//os:linux"],
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
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":target_finder",
        "//aos/vision/blob:move_scale",
        "//aos/vision/blob:threshold",
        "//aos/vision/blob:transpose",
        "//aos/vision/debug:debug_framework",
        "//aos/vision/math:vector",
    ],
)
