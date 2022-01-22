cc_binary(
    name = "camera_reader",
    srcs = [
        "camera_reader_main.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//y2022:__subpackages__"],
    deps = [
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//y2020/vision:camera_reader_lib",
    ],
)

cc_library(
    name = "blob_detector_lib",
    srcs = [
        "blob_detector.cc",
    ],
    hdrs = [
        "blob_detector.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//y2022:__subpackages__"],
    deps = [
        "//aos/network:team_number",
        "//third_party:opencv",
    ],
)

cc_binary(
    name = "viewer",
    srcs = [
        "viewer.cc",
    ],
    data = [
        "//y2020:config",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//y2022:__subpackages__"],
    deps = [
        ":blob_detector_lib",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//third_party:opencv",
        "//y2020/vision:vision_fbs",
        "//y2020/vision/sift:sift_fbs",
    ],
)
