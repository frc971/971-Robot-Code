load("@aspect_rules_js//npm:defs.bzl", "npm_package")
load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_go_library")
load("@com_github_google_flatbuffers//:typescript.bzl", "flatbuffer_ts_library")

FILE_NAMES = (
    "error_response",
    "request_all_driver_rankings",
    "request_all_driver_rankings_response",
    "request_all_matches",
    "request_all_matches_response",
    "request_all_notes",
    "request_all_notes_response",
    "request_current_scouting",
    "request_current_scouting_response",
    "request_2024_data_scouting",
    "request_2024_data_scouting_response",
    "request_2025_data_scouting",
    "request_2025_data_scouting_response",
    "request_averaged_driver_rankings_2025",
    "request_averaged_driver_rankings_2025_response",
    "request_averaged_human_rankings_2025",
    "request_averaged_human_rankings_2025_response",
    "submit_notes",
    "submit_notes_response",
    "request_notes_for_team",
    "request_notes_for_team_response",
    "submit_pit_image",
    "submit_pit_image_response",
    "request_pit_images",
    "request_pit_images_response",
    "request_all_pit_images",
    "request_all_pit_images_response",
    "request_shift_schedule",
    "request_shift_schedule_response",
    "submit_shift_schedule",
    "submit_shift_schedule_response",
    "submit_driver_ranking",
    "submit_driver_ranking_response",
    "submit_2024_actions",
    "submit_2024_actions_response",
    "submit_2025_actions",
    "submit_2025_actions_response",
    "submit_driver_ranking_2025",
    "submit_driver_ranking_2025_response",
    "submit_human_ranking_2025",
    "submit_human_ranking_2025_response",
    "delete_2024_data_scouting",
    "delete_2024_data_scouting_response",
)

filegroup(
    name = "fbs_files",
    srcs = ["%s.fbs" % name for name in FILE_NAMES],
    visibility = ["//visibility:public"],
)

[(
    flatbuffer_go_library(
        name = name + "_go_fbs",
        srcs = [name + ".fbs"],
        importpath = "github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/" + name,
        target_compatible_with = ["@platforms//cpu:x86_64"],
        visibility = ["//visibility:public"],
    ),
    flatbuffer_ts_library(
        name = name + "_ts_fbs",
        srcs = [name + ".fbs"],
        target_compatible_with = ["@platforms//cpu:x86_64"],
        visibility = ["//visibility:public"],
    ),
) for name in FILE_NAMES]

npm_package(
    name = "messages",
    srcs = [
        ":package.json",
    ] + [
        ":{}_ts_fbs_ts".format(lib)
        for lib in FILE_NAMES
    ],
    visibility = ["//visibility:public"],
)
