load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

cc_binary(
    name = "image_streamer",
    srcs = ["image_streamer.cc"],
    deps = [
        "//aos/logging",
        "//aos/logging:implementations",
        "//aos/vision/blob:codec",
        "//aos/vision/events:epoll_events",
        "//aos/vision/events:socket_types",
        "//aos/vision/events:udp",
        "//aos/vision/image:image_stream",
        "//aos/vision/image:reader",
        "//y2018:vision_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
)

flatbuffer_cc_library(
    name = "vision_fbs",
    srcs = [
        "vision.fbs",
    ],
    gen_reflections = 1,
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "vision_status",
    srcs = [
        "vision_status.cc",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":vision_fbs",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//aos/logging",
        "//aos/time",
        "//aos/vision/events:udp",
        "//y2018:vision_proto",
    ],
)
