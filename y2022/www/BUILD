load("//frc971/downloader:downloader.bzl", "aos_downloader_dir")
load("//tools/build_rules:js.bzl", "rollup_bundle", "ts_project")

filegroup(
    name = "files",
    srcs = glob([
        "**/*.html",
        "**/*.css",
        "**/*.png",
    ]),
    visibility = ["//visibility:public"],
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
        "//aos/network:connect_ts_fbs",
        "//aos/network:web_proxy_ts_fbs",
        "//aos/network/www:proxy",
        "//frc971/control_loops/drivetrain:drivetrain_status_ts_fbs",
        "//frc971/control_loops/drivetrain/localization:localizer_output_ts_fbs",
        "//y2022/control_loops/superstructure:superstructure_status_ts_fbs",
        "//y2022/localizer:localizer_status_ts_fbs",
        "//y2022/localizer:localizer_visualization_ts_fbs",
        "@com_github_google_flatbuffers//ts:flatbuffers_ts",
    ],
)

rollup_bundle(
    name = "field_main_bundle",
    entry_point = "field_main.ts",
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//y2022:__subpackages__"],
    deps = [
        ":field_main",
    ],
)

aos_downloader_dir(
    name = "www_files",
    srcs = [
        ":field_main_bundle.min.js",
        ":files",
        "//frc971/analysis:plot_index_bundle.min.js",
    ],
    dir = "www",
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)
