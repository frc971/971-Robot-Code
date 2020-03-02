load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library", "flatbuffer_ts_library")
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
    visibility = ["//y2020:__subpackages__"],
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
    data = [
        "//y2020:config.json",
    ],
    restricted_to = [
        "//tools:k8",
        "//tools:armhf-debian",
    ],
    visibility = ["//y2020:__subpackages__"],
    deps = [
        ":v4l2_reader",
        ":vision_fbs",
        "//aos:flatbuffer_merge",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//aos/network:team_number",
        "//third_party:opencv",
        "//y2020/vision/sift:sift971",
        "//y2020/vision/sift:sift_fbs",
        "//y2020/vision/sift:sift_training_fbs",
        "//y2020/vision/tools/python_code:sift_training_data",
    ],
)

flatbuffer_ts_library(
    name = "vision_ts_fbs",
    srcs = ["vision.fbs"],
    visibility = ["//y2020:__subpackages__"],
)
