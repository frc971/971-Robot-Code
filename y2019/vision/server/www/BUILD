load("@build_bazel_rules_typescript//:defs.bzl", "ts_library")
load("@build_bazel_rules_nodejs//:defs.bzl", "rollup_bundle")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "files",
    srcs = glob([
        "**/*.html",
    ]),
)

ts_library(
    name = "visualizer",
    srcs = glob([
        "*.ts",
    ]) + ["camera_constants.ts"],
)

cc_binary(
    name = "generate_camera",
    srcs = ["generate_camera.cc"],
    deps = ["//y2019:constants"],
)

genrule(
    name = "gen_cam_ts",
    outs = ["camera_constants.ts"],
    cmd = "$(location :generate_camera) $@",
    tools = [":generate_camera"],
)

rollup_bundle(
    name = "visualizer_bundle",
    entry_point = "y2019/vision/server/www/main",
    deps = [
        ":visualizer",
    ],
)
