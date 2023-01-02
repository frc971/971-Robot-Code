cc_binary(
    name = "camera_reader",
    srcs = [
        "camera_reader.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//y2023:__subpackages__"],
    deps = [
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//frc971/vision:media_device",
        "//frc971/vision:v4l2_reader",
    ],
)

cc_binary(
    name = "viewer",
    srcs = [
        "viewer.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//y2023:__subpackages__"],
    deps = [
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//frc971/vision:vision_fbs",
        "//third_party:opencv",
    ],
)
