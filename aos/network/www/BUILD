load("@npm_bazel_typescript//:defs.bzl", "ts_library")
load("@build_bazel_rules_nodejs//:defs.bzl", "nodejs_binary", "rollup_bundle")
load("//aos:config.bzl", "aos_config")

filegroup(
    name = "files",
    srcs = glob([
        "**/*.html",
        "**/*.css",
    ]),
    visibility = ["//visibility:public"],
)

ts_library(
    name = "proxy",
    srcs = [
        "config_handler.ts",
        "proxy.ts",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:configuration_ts_fbs",
        "//aos/network:connect_ts_fbs",
        "//aos/network:web_proxy_ts_fbs",
        "@com_github_google_flatbuffers//ts:flatbuffers_ts",
    ],
)

ts_library(
    name = "main",
    srcs = [
        "main.ts",
        "ping_handler.ts",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":proxy",
        "//aos/events:ping_ts_fbs",
    ],
)

rollup_bundle(
    name = "main_bundle",
    enable_code_splitting = False,
    entry_point = "main.ts",
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//aos:__subpackages__"],
    deps = [
        ":main",
    ],
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
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//aos:__subpackages__"],
)

ts_library(
    name = "reflection_test_main",
    srcs = [
        "reflection_test_main.ts",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":reflection_ts",
        "//aos:configuration_ts_fbs",
        "//aos:json_to_flatbuffer_flatbuffer_ts",
        "//aos/network/www:proxy",
        "@com_github_google_flatbuffers//ts:flatbuffers_ts",
    ],
)

ts_library(
    name = "reflection_ts",
    srcs = ["reflection.ts"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:configuration_ts_fbs",
        "//aos:json_to_flatbuffer_flatbuffer_ts",
        "@com_github_google_flatbuffers//ts:flatbuffers_ts",
    ],
)

ts_library(
    name = "plotter",
    srcs = [
        "plotter.ts",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

ts_library(
    name = "graph_main",
    srcs = [
        "graph_main.ts",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":plotter",
        ":proxy",
        ":reflection_ts",
        "//aos:configuration_ts_fbs",
        "//aos/network:connect_ts_fbs",
        "@com_github_google_flatbuffers//ts:flatbuffers_ts",
    ],
)

rollup_bundle(
    name = "graph_main_bundle",
    enable_code_splitting = False,
    entry_point = "graph_main.ts",
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":graph_main",
    ],
)

aos_config(
    name = "test_config",
    src = "test_config_file.json",
    flatbuffers = [
        "//aos:configuration_fbs",
        "//aos:json_to_flatbuffer_flatbuffer",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/events:config",
    ],
)

rollup_bundle(
    name = "reflection_test_bundle",
    enable_code_splitting = False,
    entry_point = "reflection_test_main.ts",
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":reflection_test_main",
    ],
)

sh_binary(
    name = "web_proxy_demo",
    srcs = ["web_proxy_demo.sh"],
    data = [
        ":graph.html",
        ":graph_main_bundle.min.js",
        ":reflection_test.html",
        ":reflection_test_bundle.min.js",
        ":test_config",
        "//aos/network:web_proxy_main",
        "//y2020:config",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)
