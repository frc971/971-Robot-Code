load("//tools/build_rules:js.bzl", "rollup_bundle", "ts_project")
load("//frc971/downloader:downloader.bzl", "aos_downloader_dir")

ts_project(
    name = "camera_main",
    srcs = [
        "camera_main.ts",
        "image_handler.ts",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//y2020:__subpackages__"],
    deps = [
        "//aos:configuration_ts_fbs",
        "//aos/network:connect_ts_fbs",
        "//aos/network:web_proxy_ts_fbs",
        "//aos/network/www:proxy",
        "//frc971/vision:vision_ts_fbs",
        "//y2020/vision/sift:sift_ts_fbs",
        "@com_github_google_flatbuffers//ts:flatbuffers_ts",
    ],
)

ts_project(
    name = "field_main",
    srcs = [
        "constants.ts",
        "field_handler.ts",
        "field_main.ts",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos:configuration_ts_fbs",
        "//aos/network:connect_ts_fbs",
        "//aos/network:web_proxy_ts_fbs",
        "//aos/network/www:proxy",
        "//frc971/control_loops/drivetrain:drivetrain_status_ts_fbs",
        "//y2020/control_loops/drivetrain:localizer_debug_ts_fbs",
        "//y2020/control_loops/superstructure:superstructure_status_ts_fbs",
        "//y2020/vision/sift:sift_ts_fbs",
        "@com_github_google_flatbuffers//ts:flatbuffers_ts",
    ],
)

rollup_bundle(
    name = "camera_main_bundle",
    entry_point = "camera_main.ts",
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//y2020:__subpackages__"],
    deps = [
        "camera_main",
    ],
)

rollup_bundle(
    name = "field_main_bundle",
    entry_point = "field_main.ts",
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//y2020:__subpackages__"],
    deps = [
        "field_main",
    ],
)

filegroup(
    name = "files",
    srcs = glob([
        "**/*.html",
        "**/*.css",
    ]),
    visibility = ["//visibility:public"],
)

aos_downloader_dir(
    name = "www_files",
    srcs = [
        ":camera_main_bundle",
        ":field_main_bundle",
        ":files",
        "//frc971/analysis:plot_index_bundle.min.js",
    ],
    dir = "www",
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)
