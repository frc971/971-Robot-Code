load("@npm//:defs.bzl", "npm_link_all_packages")
load("//tools/build_rules:js.bzl", "ng_pkg")

npm_link_all_packages(name = "node_modules")

ng_pkg(
    name = "rpc",
    extra_srcs = [
        "public-api.ts",
    ],
    generate_public_api = False,
    deps = [
        "//scouting/webserver/requests/messages:error_response_ts_fbs",
        "//scouting/webserver/requests/messages:request_all_driver_rankings_response_ts_fbs",
        "//scouting/webserver/requests/messages:request_all_driver_rankings_ts_fbs",
        "//scouting/webserver/requests/messages:request_all_matches_response_ts_fbs",
        "//scouting/webserver/requests/messages:request_all_matches_ts_fbs",
        "//scouting/webserver/requests/messages:request_all_notes_response_ts_fbs",
        "//scouting/webserver/requests/messages:request_all_notes_ts_fbs",
        "//scouting/webserver/requests/messages:request_data_scouting_response_ts_fbs",
        "//scouting/webserver/requests/messages:request_data_scouting_ts_fbs",
        "@com_github_google_flatbuffers//ts:flatbuffers_ts",
    ],
)
