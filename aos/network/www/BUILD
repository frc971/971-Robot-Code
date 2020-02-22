load("@build_bazel_rules_typescript//:defs.bzl", "ts_library")
load("@build_bazel_rules_nodejs//:defs.bzl", "rollup_bundle")

filegroup(
    name = "files",
    srcs = glob([
        "**/*.html",
        "**/*.css",
    ]),
    visibility=["//visibility:public"],
)

ts_library(
    name = "proxy",
    srcs = [
        "config_handler.ts",
        "proxy.ts",
    ],
    deps = [
        "//aos/network:web_proxy_ts_fbs",
        "//aos/network:connect_ts_fbs",
        "//aos:configuration_ts_fbs",
    ],
    visibility=["//visibility:public"],
)

ts_library(
    name = "main",
    srcs = [
        "main.ts",
        "ping_handler.ts",
    ],
    deps = [
        ":proxy",
        "//aos/events:ping_ts_fbs",
    ],
)

rollup_bundle(
    name = "main_bundle",
    entry_point = "aos/network/www/main",
    deps = [
        "main",
    ],
    visibility=["//aos:__subpackages__"],
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
    visibility=["//aos:__subpackages__"],
)
