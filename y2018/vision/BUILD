load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")

cc_binary(
    name = "image_streamer",
    srcs = ["image_streamer.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/logging",
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

static_flatbuffer(
    name = "vision_fbs",
    srcs = [
        "vision.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "vision_status",
    srcs = [
        "vision_status.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
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
