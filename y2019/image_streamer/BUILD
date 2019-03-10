package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "image_streamer",
    srcs = ["image_streamer.cc"],
    deps = [
        ":flip_image",
        "//aos/logging",
        "//aos/logging:implementations",
        "//aos/vision/blob:codec",
        "//aos/vision/events:epoll_events",
        "//aos/vision/events:socket_types",
        "//aos/vision/events:udp",
        "//aos/vision/image:image_stream",
        "//aos/vision/image:reader",
        "//y2019:vision_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "flip_image",
    srcs = ["flip_image.cc"],
    hdrs = ["flip_image.h"],
    copts = [
        "-Wno-format-nonliteral",
        "-Wno-cast-align",
        "-Wno-cast-qual",
        "-Wno-error=type-limits",
    ],
    deps = [
        "//third_party/cimg:CImg",
        "//third_party/libjpeg",
    ],
)
