load("@npm//@bazel/typescript:index.bzl", "ts_library")

ts_library(
    name = "entry",
    srcs = [
        "entry.component.ts",
        "entry.module.ts",
    ],
    angular_assets = [
        "entry.component.css",
        "entry.ng.html",
        "//scouting/www:common_css",
    ],
    compiler = "//tools:tsc_wrapped_with_angular",
    target_compatible_with = ["@platforms//cpu:x86_64"],
    use_angular_plugin = True,
    visibility = ["//visibility:public"],
    deps = [
        "//scouting/webserver/requests/messages:error_response_ts_fbs",
        "//scouting/webserver/requests/messages:submit_data_scouting_response_ts_fbs",
        "//scouting/webserver/requests/messages:submit_data_scouting_ts_fbs",
        "//scouting/www/counter_button",
        "@com_github_google_flatbuffers//ts:flatbuffers_ts",
        "@npm//@angular/common",
        "@npm//@angular/core",
        "@npm//@angular/forms",
    ],
)
