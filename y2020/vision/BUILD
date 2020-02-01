load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")
load("//aos:config.bzl", "aos_config")

flatbuffer_cc_library(
    name = "vision_fbs",
    srcs = ["vision.fbs"],
    gen_reflections = 1,
    visibility = ["//y2020:__subpackages__"],
)

cc_library(
    name = "v4l2_reader",
    srcs = [
        "v4l2_reader.cc",
    ],
    hdrs = [
        "v4l2_reader.h",
    ],
    deps = [
        ":vision_fbs",
        "//aos/events:event_loop",
        "//aos/scoped:scoped_fd",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/base",
    ],
)

cc_binary(
    name = "camera_reader",
    srcs = [
        "camera_reader.cc",
    ],
    deps = [
        ":v4l2_reader",
        "//aos:init",
        "//aos/events:shm_event_loop",
    ],
)
