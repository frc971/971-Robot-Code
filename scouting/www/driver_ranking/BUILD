load("@npm//@bazel/typescript:index.bzl", "ts_library")

ts_library(
    name = "driver_ranking",
    srcs = [
        "driver_ranking.component.ts",
        "driver_ranking.module.ts",
    ],
    angular_assets = [
        "driver_ranking.component.css",
        "driver_ranking.ng.html",
        "//scouting/www:common_css",
    ],
    compiler = "//tools:tsc_wrapped_with_angular",
    target_compatible_with = ["@platforms//cpu:x86_64"],
    use_angular_plugin = True,
    visibility = ["//visibility:public"],
    deps = [
        "//scouting/webserver/requests/messages:error_response_ts_fbs",
        "//scouting/webserver/requests/messages:submit_driver_ranking_ts_fbs",
        "@com_github_google_flatbuffers//ts:flatbuffers_ts",
        "@npm//@angular/common",
        "@npm//@angular/core",
        "@npm//@angular/forms",
    ],
)
