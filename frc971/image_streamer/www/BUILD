load("//tools/build_rules:js.bzl", "rollup_bundle", "ts_project")
load("//frc971/downloader:downloader.bzl", "aos_downloader_dir")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "files",
    srcs = glob([
        "**/*.html",
        "**/*.css",
    ]),
)

ts_project(
    name = "proxy",
    srcs = [
        "proxy.ts",
    ],
    deps = [
        "//aos/network:web_proxy_ts_fbs",
        "@com_github_google_flatbuffers//ts:flatbuffers_ts",
    ],
)

ts_project(
    name = "main",
    srcs = [
        "main.ts",
    ],
    deps = [
        ":proxy",
    ],
)

rollup_bundle(
    name = "main_bundle",
    entry_point = "main.ts",
    deps = [
        "main",
    ],
)

aos_downloader_dir(
    name = "www_files",
    srcs = [
        ":files",
        ":main_bundle.min.js",
    ],
    dir = "image_streamer_www",
    visibility = ["//visibility:public"],
)
