load("@npm//@bazel/typescript:index.bzl", "ts_library")

ts_library(
    name = "view",
    srcs = [
        "view.component.ts",
        "view.module.ts",
    ],
    angular_assets = [
        "view.component.css",
        "view.ng.html",
        "//scouting/www:common_css",
    ],
    compiler = "//tools:tsc_wrapped_with_angular",
    target_compatible_with = ["@platforms//cpu:x86_64"],
    use_angular_plugin = True,
    visibility = ["//visibility:public"],
    deps = [
        "//scouting/webserver/requests/messages:error_response_ts_fbs",
        "//scouting/webserver/requests/messages:request_all_driver_rankings_response_ts_fbs",
        "//scouting/webserver/requests/messages:request_all_driver_rankings_ts_fbs",
        "//scouting/webserver/requests/messages:request_all_notes_response_ts_fbs",
        "//scouting/webserver/requests/messages:request_all_notes_ts_fbs",
        "//scouting/webserver/requests/messages:request_data_scouting_response_ts_fbs",
        "//scouting/webserver/requests/messages:request_data_scouting_ts_fbs",
        "//scouting/www/rpc",
        "@com_github_google_flatbuffers//ts:flatbuffers_ts",
        "@npm//@angular/common",
        "@npm//@angular/core",
        "@npm//@angular/forms",
    ],
)
