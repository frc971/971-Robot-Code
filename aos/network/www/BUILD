load("@build_bazel_rules_typescript//:defs.bzl", "ts_library")
load("@build_bazel_rules_nodejs//:defs.bzl", "rollup_bundle")

filegroup(
    name = "files",
    srcs = glob([
        "**/*.html",
    ]),
    visibility=["//visibility:public"],
)

ts_library(
    name = "proxy",
    srcs = glob([
        "*.ts",
    ]),
    deps = [
        "//aos/network:web_proxy_ts_fbs",
    ],
)

rollup_bundle(
    name = "proxy_bundle",
    entry_point = "aos/network/www/main",
    deps = [
        "proxy",
    ],
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
    visibility=["//visibility:public"],
)
