load("@build_bazel_rules_typescript//:defs.bzl", "ts_library")
load("@build_bazel_rules_nodejs//:defs.bzl", "rollup_bundle")
load("//frc971/downloader:downloader.bzl", "aos_downloader_dir")

ts_library(
    name = "camera_main",
    srcs = [
        "image_handler.ts",
        "camera_main.ts",
    ],
    visibility = ["//y2020:__subpackages__"],
    deps = [
        "//aos/network/www:proxy",
        "//y2020/vision:vision_ts_fbs",
        "//y2020/vision/sift:sift_ts_fbs",
    ],
)

ts_library(
    name = "field_main",
    srcs = [
        "field_main.ts",
        "field_handler.ts",
        "constants.ts",
    ],
    deps = [
        "//aos/network/www:proxy",
        "//y2020/vision/sift:sift_ts_fbs",
    ],
)

rollup_bundle(
    name = "camera_main_bundle",
    entry_point = "y2020/www/camera_main",
    visibility = ["//y2020:__subpackages__"],
    deps = [
        "camera_main",
    ],
)

rollup_bundle(
    name = "field_main_bundle",
    entry_point = "y2020/www/field_main",
    deps = [
        "field_main",
    ],
    visibility = ["//y2020:__subpackages__"],
)

filegroup(
    name = "files",
    srcs = glob([
        "**/*.html",
        "**/*.css",
    ]),
    visibility = ["//visibility:public"],
)

genrule(
    name = "flatbuffers",
    srcs = [
        "@com_github_google_flatbuffers//:flatjs",
    ],
    outs = [
        "flatbuffers.js",
    ],
    cmd = "cp $(location @com_github_google_flatbuffers//:flatjs) $@",
    visibility = ["//y2020:__subpackages__"],
)

aos_downloader_dir(
    name = "www_files",
    srcs = [
        ":camera_main_bundle",
        ":field_main_bundle",
        ":files",
        ":flatbuffers",
    ],
    dir = "www",
    visibility = ["//visibility:public"],
)
