load("@build_bazel_rules_typescript//:defs.bzl", "ts_library")
load("@build_bazel_rules_nodejs//:defs.bzl", "rollup_bundle")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "files",
    srcs = glob([
        "**/*",
    ]),
)

ts_library(
    name = "visualizer",
    srcs = glob([
        "*.ts",
    ]),
)

rollup_bundle(
    name = "visualizer_bundle",
    entry_point = "y2019/vision/server/www/main",
    deps = [
        ":visualizer",
    ],
)
