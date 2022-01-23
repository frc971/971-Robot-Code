load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library", "flatbuffer_ts_library")

flatbuffer_cc_library(
    name = "vision_fbs",
    srcs = ["vision.fbs"],
    gen_reflections = 1,
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

flatbuffer_ts_library(
    name = "vision_ts_fbs",
    srcs = ["vision.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "v4l2_reader",
    srcs = [
        "v4l2_reader.cc",
    ],
    hdrs = [
        "v4l2_reader.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":vision_fbs",
        "//aos/events:event_loop",
        "//aos/scoped:scoped_fd",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/base",
    ],
)
