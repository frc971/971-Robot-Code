load("//tools/build_rules:gtk_dependent.bzl", "gtk_dependent_cc_binary", "gtk_dependent_cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "overlay",
    hdrs = ["overlay.h"],
    deps = [
        "//aos/vision/image:image_types",
        "//aos/vision/math:segment",
        "//aos/vision/math:vector",
    ],
)

gtk_dependent_cc_library(
    name = "debug_window",
    srcs = ["debug_window.cc"],
    hdrs = ["debug_window.h"],
    deps = [
        ":overlay",
        "//aos/vision/image:image_types",
        "@usr_repo//:gtk+-3.0",
    ],
)

gtk_dependent_cc_library(
    name = "debug_framework",
    srcs = [
        "aveugle-source.cc",
        "blob_log-source.cc",
        "camera-source.cc",
        "debug_framework.cc",
        "jpeg_list-source.cc",
        "tcp-source.cc",
    ],
    hdrs = ["debug_framework.h"],
    deps = [
        "//aos/logging",
        "//aos/logging:implementations",
        "//aos/util:global_factory",
        "//aos/vision/blob:codec",
        "//aos/vision/blob:find_blob",
        "//aos/vision/blob:range_image",
        "//aos/vision/blob:stream_view",
        "//aos/vision/events:epoll_events",
        "//aos/vision/events:gtk_event",
        "//aos/vision/events:tcp_client",
        "//aos/vision/image:image_dataset",
        "//aos/vision/image:image_stream",
        "//aos/vision/image:image_types",
        "//aos/vision/image:jpeg_routines",
        "@com_github_google_glog//:glog",
        "@usr_repo//:gtk+-3.0",
    ],
    alwayslink = 1,
)
