load("@npm//@bazel/typescript:index.bzl", "ts_library")

ts_library(
    name = "rpc",
    srcs = [
        "match_list_requestor.ts",
    ],
    compiler = "//tools:tsc_wrapped_with_angular",
    target_compatible_with = ["@platforms//cpu:x86_64"],
    use_angular_plugin = True,
    visibility = ["//visibility:public"],
    deps = [
        "//scouting/webserver/requests/messages:error_response_ts_fbs",
        "//scouting/webserver/requests/messages:request_all_matches_response_ts_fbs",
        "//scouting/webserver/requests/messages:request_all_matches_ts_fbs",
        "@com_github_google_flatbuffers//ts:flatbuffers_ts",
        "@npm//@angular/core",
    ],
)
