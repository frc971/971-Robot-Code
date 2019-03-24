load("//aos/seasocks:gen_embedded.bzl", "gen_embedded")
load("//aos/downloader:downloader.bzl", "aos_downloader_dir")
load("@build_bazel_rules_typescript//:defs.bzl", "ts_library")
load("@build_bazel_rules_nodejs//:defs.bzl", "rollup_bundle")

ts_library(
    name = "demo",
    srcs = [
        "demo.ts",
    ],
)

rollup_bundle(
    name = "bundle",
    entry_point = "y2019/vision/server/demo",
    deps = [":demo"],
)

gen_embedded(
    name = "gen_embedded",
    srcs = glob(
        include = ["www_defaults/**/*"],
        exclude = ["www/**/*"],
    ),
)

aos_downloader_dir(
    name = "www_files",
    srcs = glob([
        "www/**/*",
    ]) + [
        ":bundle",
    ],
    dir = "www",
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "server",
    srcs = [
        "server.cc",
    ],
    data = glob([
        "www/**/*",
    ]) + [
        ":bundle",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":gen_embedded",
        "//aos:init",
        "//aos/logging",
        "//aos/time",
        "//frc971/control_loops/drivetrain:drivetrain_queue",
        "//third_party/seasocks",
        "//y2019/control_loops/drivetrain:camera_queue",
    ],
)
