load("//aos/seasocks:gen_embedded.bzl", "gen_embedded")
load("//frc971/downloader:downloader.bzl", "aos_downloader_dir")

gen_embedded(
    name = "gen_embedded",
    srcs = glob(
        ["www_defaults/**/*"],
        exclude = ["www/**/*"],
    ),
)

aos_downloader_dir(
    name = "www_files",
    srcs = glob([
        "www/**/*",
    ]),
    dir = "www",
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "dashboard",
    srcs = [
        "dashboard.cc",
        "dashboard.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":gen_embedded",
        "//aos:init",
        "//aos/events:event_loop",
        "//aos/events:shm_event_loop",
        "//aos/logging",
        "//aos/seasocks:seasocks_logger",
        "//aos/time",
        "//aos/util:phased_loop",
        "//frc971/autonomous:auto_mode_fbs",
        "//frc971/control_loops:control_loops_fbs",
        "//third_party/seasocks",
        "//y2016/control_loops/superstructure:superstructure_status_fbs",
        "//y2016/queues:ball_detector_fbs",
        "//y2016/vision:vision_fbs",
    ],
)
