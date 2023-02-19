load("@npm//:defs.bzl", "npm_link_all_packages")
load("//tools/build_rules:js.bzl", "ng_pkg")

npm_link_all_packages(name = "node_modules")

ng_pkg(
    name = "view",
    extra_srcs = [
        "//scouting/www:app_common_css",
    ],
    deps = [
        ":node_modules/@angular/forms",
        "//scouting/webserver/requests/messages:error_response_ts_fbs",
        "//scouting/webserver/requests/messages:request_all_driver_rankings_response_ts_fbs",
        "//scouting/webserver/requests/messages:request_all_driver_rankings_ts_fbs",
        "//scouting/webserver/requests/messages:request_all_notes_response_ts_fbs",
        "//scouting/webserver/requests/messages:request_all_notes_ts_fbs",
        "//scouting/webserver/requests/messages:request_data_scouting_response_ts_fbs",
        "//scouting/webserver/requests/messages:request_data_scouting_ts_fbs",
        "//scouting/www/rpc:_lib",
        "@com_github_google_flatbuffers//ts:flatbuffers_ts",
    ],
)
