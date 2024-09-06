load("//frc971/downloader:downloader.bzl", "aos_downloader_dir")

aos_downloader_dir(
    name = "www_files",
    srcs = [
        "//frc971/control_loops/swerve/spline_ui/www:static_files",
    ],
    dir = "www",
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "server",
    srcs = [
        "server.cc",
    ],
    data = [
        "//frc971/control_loops/swerve/spline_ui/www:static_files",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:init",
        "//aos/containers:ring_buffer",
        "//aos/events:shm_event_loop",
        "//aos/logging",
        "//aos/network:gen_embedded",
        "//aos/seasocks:seasocks_logger",
        "//aos/time",
        "//third_party/seasocks",
        "@com_google_protobuf//:protobuf",
    ],
)
