load("//aos/build:queues.bzl", "queue_library")
load("//tools/build_rules:gtk_dependent.bzl", "gtk_dependent_cc_binary", "gtk_dependent_cc_library")
load("@com_google_protobuf//:protobuf.bzl", "cc_proto_library")

package(default_visibility = ["//visibility:public"])

VISION_TARGETS = [
    "//tools:k8",
    "//tools:armhf-debian",
]

cc_library(
    name = "target_finder",
    srcs = [
        "target_finder.cc",
        "target_geometry.cc",
    ],
    hdrs = [
        "target_finder.h",
        "target_types.h",
    ],
    restricted_to = VISION_TARGETS,
    deps = [
        "//aos/vision/blob:contour",
        "//aos/vision/blob:hierarchical_contour_merge",
        "//aos/vision/blob:region_alloc",
        "//aos/vision/blob:threshold",
        "//aos/vision/blob:transpose",
        "//aos/vision/debug:overlay",
        "//aos/vision/math:vector",
        "@com_google_ceres_solver//:ceres",
    ],
)

gtk_dependent_cc_binary(
    name = "debug_viewer",
    srcs = ["debug_viewer.cc"],
    copts = ["-Wno-unused-variable"],
    restricted_to = VISION_TARGETS,
    deps = [
        ":target_finder",
        "//aos/vision/blob:move_scale",
        "//aos/vision/blob:threshold",
        "//aos/vision/blob:transpose",
        "//aos/vision/debug:debug_framework",
        "//aos/vision/math:vector",
    ],
)

cc_binary(
    name = "target_sender",
    srcs = ["target_sender.cc"],
    restricted_to = VISION_TARGETS,
    deps = [
        ":target_finder",
        "//aos/logging",
        "//aos/logging:implementations",
        "//aos/vision/blob:codec",
        "//aos/vision/blob:find_blob",
        "//aos/vision/events:epoll_events",
        "//aos/vision/events:socket_types",
        "//aos/vision/events:udp",
        "//y2019/jevois:serial",
        "//y2019/jevois:structures",
        "//y2019/jevois:uart",
        "//y2019/jevois/camera:reader",
        "//y2019/jevois/camera:image_stream",
        "@com_google_ceres_solver//:ceres",
    ],
)

cc_binary(
    name = "debug_serial",
    srcs = ["debug_serial.cc"],
    deps = [
        "//y2019/jevois:serial",
        "//y2019/jevois:structures",
        "//y2019/jevois:uart",
         "//aos/logging",
         "//aos/logging:implementations",
    ],
)

"""
cc_binary(
    name = "calibration",
    srcs = ["calibration.cc"],
    deps = [
         ":target_finder",
         "//aos/logging",
         "//aos/logging:implementations",
         "//aos/vision/blob:find_blob",
         "//aos/vision/blob:codec",
         "//aos/vision/events:epoll_events",
         "//aos/vision/events:socket_types",
         "//aos/vision/events:udp",
         "//aos/vision/image:image_stream",
         "//aos/vision/image:reader",
         "@com_google_ceres_solver//:ceres",
    ],
    restricted_to = VISION_TARGETS,
)
"""
