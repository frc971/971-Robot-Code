load("@build_bazel_rules_typescript//:defs.bzl", "ts_library")
load("@build_bazel_rules_nodejs//:defs.bzl", "rollup_bundle")

ts_library(
    name = "main",
    srcs = [
        "main.ts",
        "image_handler.ts",
    ],
    deps = [
        "//aos/network/www:proxy",
        "//y2020/vision:vision_ts_fbs",
    ],
    visibility = ["//y2020:__subpackages__"],
)

rollup_bundle(
    name = "main_bundle",
    entry_point = "y2020/www/main",
    deps = [
        "main",
    ],
    visibility = ["//y2020:__subpackages__"],
)

filegroup(
    name = "files",
    srcs = glob([
        "**/*.html",
        "**/*.css",
    ]),
    visibility=["//visibility:public"],
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
    visibility=["//y2020:__subpackages__"],
)
