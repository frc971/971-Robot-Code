load("//aos/seasocks:gen_embedded.bzl", "gen_embedded")
load("//aos/downloader:downloader.bzl", "aos_downloader_dir")

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
        "//aos/events:event-loop",
        "//aos/events:shm-event-loop",
        "//aos/logging",
        "//aos/seasocks:seasocks_logger",
        "//aos/time",
        "//aos/util:phased_loop",
        "//frc971/autonomous:auto_queue",
        "//third_party/seasocks",
        "//y2016/control_loops/superstructure:superstructure_queue",
        "//y2016/queues:ball_detector",
        "//y2016/vision:vision_queue",
    ],
)
