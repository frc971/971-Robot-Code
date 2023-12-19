load("@bazel_gazelle//:def.bzl", "gazelle")
load("@aspect_rules_ts//ts:defs.bzl", "ts_config")
load("@npm//:defs.bzl", "npm_link_all_packages")
load("@aspect_rules_js//npm:defs.bzl", "npm_link_package")
load("@hedron_compile_commands//:refresh_compile_commands.bzl", "refresh_compile_commands")

# Link npm packages
npm_link_all_packages(name = "node_modules")

exports_files([
    "tsconfig.json",
    "tsconfig.node.json",
    "rollup.config.js",
])

# The root repo tsconfig
ts_config(
    name = "tsconfig",
    src = "tsconfig.json",
    visibility = ["//visibility:public"],
)

ts_config(
    name = "tsconfig.node",
    src = "tsconfig.node.json",
    visibility = ["//visibility:public"],
    deps = [":tsconfig"],
)

npm_link_package(
    name = "node_modules/flatbuffers",
    src = "@com_github_google_flatbuffers//ts:flatbuffers",
)

npm_link_package(
    name = "node_modules/flatbuffers_reflection",
    src = "@com_github_google_flatbuffers//reflection:flatbuffers_reflection",
)

# gazelle:prefix github.com/frc971/971-Robot-Code
# gazelle:build_file_name BUILD
# gazelle:proto disable
# gazelle:go_generate_proto false
# gazelle:exclude third_party
# gazelle:exclude external
# gazelle:resolve go github.com/google/flatbuffers/go @com_github_google_flatbuffers//go:go_default_library
# gazelle:resolve go github.com/frc971/971-Robot-Code/build_tests/fbs //build_tests:test_go_fbs
# gazelle:resolve go github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/error_response //scouting/webserver/requests/messages:error_response_go_fbs
# gazelle:resolve go github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_notes //scouting/webserver/requests/messages:submit_notes_go_fbs
# gazelle:resolve go github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_notes_response //scouting/webserver/requests/messages:submit_notes_response_go_fbs
# gazelle:resolve go github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_2023_data_scouting_response //scouting/webserver/requests/messages:request_2023_data_scouting_response_go_fbs
# gazelle:resolve go github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_2023_data_scouting //scouting/webserver/requests/messages:request_2023_data_scouting_go_fbs
# gazelle:resolve go github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_matches_for_team_response //scouting/webserver/requests/messages:request_matches_for_team_response_go_fbs
# gazelle:resolve go github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_matches_for_team //scouting/webserver/requests/messages:request_matches_for_team_go_fbs
# gazelle:resolve go github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_notes_for_team_response //scouting/webserver/requests/messages:request_notes_for_team_response_go_fbs
# gazelle:resolve go github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_notes_for_team //scouting/webserver/requests/messages:request_notes_for_team_go_fbs
# gazelle:resolve go github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_all_matches_response //scouting/webserver/requests/messages:request_all_matches_response_go_fbs
# gazelle:resolve go github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_all_matches //scouting/webserver/requests/messages:request_all_matches_go_fbs
# gazelle:resolve go github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_all_notes_response //scouting/webserver/requests/messages:request_all_notes_response_go_fbs
# gazelle:resolve go github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_all_notes //scouting/webserver/requests/messages:request_all_notes_go_fbs
# gazelle:resolve go github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_all_driver_rankings_response //scouting/webserver/requests/messages:request_all_driver_rankings_response_go_fbs
# gazelle:resolve go github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_all_driver_rankings //scouting/webserver/requests/messages:request_all_driver_rankings_go_fbs
# gazelle:resolve go github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/refresh_match_list //scouting/webserver/requests/messages:refresh_match_list_go_fbs
# gazelle:resolve go github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/refresh_match_list_response //scouting/webserver/requests/messages:refresh_match_list_response_go_fbs
# gazelle:resolve go github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_shift_schedule //scouting/webserver/requests/messages:request_shift_schedule_go_fbs
# gazelle:resolve go github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_shift_schedule_response //scouting/webserver/requests/messages:request_shift_schedule_response_go_fbs
# gazelle:resolve go github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_actions //scouting/webserver/requests/messages:submit_actions_go_fbs
# gazelle:resolve go github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_actions_response //scouting/webserver/requests/messages:submit_actions_response_go_fbs
# gazelle:resolve go github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_shift_schedule //scouting/webserver/requests/messages:submit_shift_schedule_go_fbs
# gazelle:resolve go github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_shift_schedule_response //scouting/webserver/requests/messages:submit_shift_schedule_response_go_fbs
# gazelle:resolve go github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_driver_ranking //scouting/webserver/requests/messages:submit_driver_ranking_go_fbs
# gazelle:resolve go github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_driver_ranking_response //scouting/webserver/requests/messages:submit_driver_ranking_response_go_fbs
# gazelle:resolve go github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/delete_2023_data_scouting //scouting/webserver/requests/messages:delete_2023_data_scouting_go_fbs
# gazelle:resolve go github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/delete_2023_data_scouting_response //scouting/webserver/requests/messages:delete_2023_data_scouting_response_go_fbs
# gazelle:resolve go github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_pit_image //scouting/webserver/requests/messages:submit_pit_image_go_fbs
# gazelle:resolve go github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_pit_image_response //scouting/webserver/requests/messages:submit_pit_image_response_go_fbs
# gazelle:resolve go github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_pit_images //scouting/webserver/requests/messages:request_pit_images_go_fbs
# gazelle:resolve go github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_pit_images_response //scouting/webserver/requests/messages:request_pit_images_response_go_fbs

gazelle(
    name = "gazelle",
    visibility = ["//tools/lint:__subpackages__"],
)

refresh_compile_commands(
    name = "refresh_compile_commands",

    # Specify the targets of interest.
    # For example, specify a dict of targets and any flags required to build.
    targets = {
        "//aos/...": "-c opt",
        "//frc971/...": "-c opt",
    },
    # No need to add flags already in .bazelrc. They're automatically picked up.
    # If you don't need flags, a list of targets is also okay, as is a single target string.
    # Wildcard patterns, like //... for everything, *are* allowed here, just like a build.
    # As are additional targets (+) and subtractions (-), like in bazel query https://docs.bazel.build/versions/main/query.html#expressions
    # And if you're working on a header-only library, specify a test or binary target that compiles it.
)
