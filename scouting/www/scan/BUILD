load("@npm//:defs.bzl", "npm_link_all_packages")
load("//tools/build_rules:js.bzl", "ng_pkg")

npm_link_all_packages(name = "node_modules")

ng_pkg(
    name = "scan",
    extra_srcs = [
        "//scouting/www:app_common_css",
    ],
    deps = [
        ":node_modules/@angular/forms",
        ":node_modules/@types/pako",
        ":node_modules/pako",
        "//scouting/webserver/requests/messages:error_response_ts_fbs",
        "@com_github_google_flatbuffers//ts:flatbuffers_ts",
    ],
)
