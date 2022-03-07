load("@npm//@bazel/typescript:index.bzl", "ts_library")

ts_library(
    name = "import_match_list",
    srcs = [
        "import_match_list.component.ts",
        "import_match_list.module.ts",
    ],
    angular_assets = [
        "import_match_list.component.css",
        "import_match_list.ng.html",
        "//scouting/www:common_css",
    ],
    compiler = "//tools:tsc_wrapped_with_angular",
    target_compatible_with = ["@platforms//cpu:x86_64"],
    use_angular_plugin = True,
    visibility = ["//visibility:public"],
    deps = [
        "//scouting/webserver/requests/messages:error_response_ts_fbs",
        "//scouting/webserver/requests/messages:refresh_match_list_response_ts_fbs",
        "//scouting/webserver/requests/messages:refresh_match_list_ts_fbs",
        "@com_github_google_flatbuffers//ts:flatbuffers_ts",
        "@npm//@angular/common",
        "@npm//@angular/core",
        "@npm//@angular/forms",
    ],
)
