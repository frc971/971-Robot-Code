cc_binary(
    name = "can_logger",
    srcs = [
        "can_logger_main.cc",
    ],
    target_compatible_with = ["@platforms//cpu:arm64"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//aos/time",
        "//frc971/can_logger:can_logger_lib",
        "@com_github_google_glog//:glog",
    ],
)
