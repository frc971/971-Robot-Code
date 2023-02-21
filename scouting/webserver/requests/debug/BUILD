load("@io_bazel_rules_go//go:def.bzl", "go_library")

go_library(
    name = "debug",
    srcs = ["debug.go"],
    importpath = "github.com/frc971/971-Robot-Code/scouting/webserver/requests/debug",
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = ["//visibility:public"],
    deps = [
        "//scouting/webserver/requests/messages:error_response_go_fbs",
        "//scouting/webserver/requests/messages:request_2023_data_scouting_response_go_fbs",
        "//scouting/webserver/requests/messages:request_all_driver_rankings_response_go_fbs",
        "//scouting/webserver/requests/messages:request_all_matches_response_go_fbs",
        "//scouting/webserver/requests/messages:request_all_notes_response_go_fbs",
        "//scouting/webserver/requests/messages:request_notes_for_team_response_go_fbs",
        "//scouting/webserver/requests/messages:request_shift_schedule_response_go_fbs",
        "//scouting/webserver/requests/messages:submit_actions_response_go_fbs",
        "//scouting/webserver/requests/messages:submit_driver_ranking_response_go_fbs",
        "//scouting/webserver/requests/messages:submit_notes_response_go_fbs",
        "//scouting/webserver/requests/messages:submit_shift_schedule_response_go_fbs",
        "@com_github_google_flatbuffers//go:go_default_library",
    ],
)
