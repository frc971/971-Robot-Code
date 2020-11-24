load("@npm_bazel_typescript//:defs.bzl", "ts_library")
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
    target_compatible_with = ["@platforms//os:linux"],
)

cc_binary(
    name = "generate_camera",
    srcs = ["generate_camera.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = ["//y2019:constants"],
)

genrule(
    name = "gen_cam_ts",
    outs = ["camera_constants.ts"],
    cmd = "$(location :generate_camera) $@",
    target_compatible_with = ["@platforms//os:linux"],
    tools = [":generate_camera"],
)

rollup_bundle(
    name = "visualizer_bundle",
    enable_code_splitting = False,
    entry_point = "main.ts",
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":visualizer",
    ],
)
