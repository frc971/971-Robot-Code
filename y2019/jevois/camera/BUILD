package(default_visibility = ["//visibility:public"])

cc_library(
    name = "reader",
    srcs = ["reader.cc"],
    hdrs = ["reader.h"],
    deps = [
        "//aos/vision/image:reader",
        "//aos/vision/image:camera_params",
        "//aos/vision/image:image_types",
        "//aos/logging",
        "//aos/time",
    ],
)

cc_library(
    name = "image_stream",
    hdrs = ["image_stream.h"],
    deps = [
        "//aos/vision/events:epoll_events",
        ":reader",
    ],
)
