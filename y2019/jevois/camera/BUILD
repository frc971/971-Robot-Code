package(default_visibility = ["//visibility:public"])

cc_library(
    name = "reader",
    srcs = ["reader.cc"],
    hdrs = ["reader.h"],
    deps = [
        "//aos/time",
        "//aos/vision/image:camera_params",
        "//aos/vision/image:image_types",
        "//aos/vision/image:reader",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "image_stream",
    srcs = ["image_stream.cc"],
    hdrs = ["image_stream.h"],
    deps = [
        ":reader",
        "//aos/logging",
        "//aos/vision/events:epoll_events",
    ],
)
