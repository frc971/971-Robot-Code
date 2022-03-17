load("@bazel_gazelle//:def.bzl", "gazelle")

exports_files([
    "tsconfig.json",
    "rollup.config.js",
])

# gazelle:prefix github.com/frc971/971-Robot-Code
# gazelle:build_file_name BUILD
# gazelle:proto disable
# gazelle:go_generate_proto false
# gazelle:exclude third_party
# gazelle:exclude external
# gazelle:resolve go github.com/phst/runfiles @com_github_phst_runfiles//:go_default_library
# gazelle:resolve go github.com/frc971/971-Robot-Code/build_tests/fbs //build_tests:test_go_fbs
# gazelle:resolve go github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/error_response //scouting/webserver/requests/messages:error_response_go_fbs
# gazelle:resolve go github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_data_scouting //scouting/webserver/requests/messages:submit_data_scouting_go_fbs
# gazelle:resolve go github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_data_scouting_response //scouting/webserver/requests/messages:submit_data_scouting_response_go_fbs
# gazelle:resolve go github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_notes //scouting/webserver/requests/messages:submit_notes_go_fbs
# gazelle:resolve go github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/submit_notes_response //scouting/webserver/requests/messages:submit_notes_response_go_fbs
# gazelle:resolve go github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_data_scouting_response //scouting/webserver/requests/messages:request_data_scouting_response_go_fbs
# gazelle:resolve go github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_data_scouting //scouting/webserver/requests/messages:request_data_scouting_go_fbs
# gazelle:resolve go github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_matches_for_team_response //scouting/webserver/requests/messages:request_matches_for_team_response_go_fbs
# gazelle:resolve go github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_matches_for_team //scouting/webserver/requests/messages:request_matches_for_team_go_fbs
# gazelle:resolve go github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_notes_for_team_response //scouting/webserver/requests/messages:request_notes_for_team_response_go_fbs
# gazelle:resolve go github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_notes_for_team //scouting/webserver/requests/messages:request_notes_for_team_go_fbs
# gazelle:resolve go github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_all_matches_response //scouting/webserver/requests/messages:request_all_matches_response_go_fbs
# gazelle:resolve go github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/request_all_matches //scouting/webserver/requests/messages:request_all_matches_go_fbs
# gazelle:resolve go github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/refresh_match_list //scouting/webserver/requests/messages:refresh_match_list_go_fbs
# gazelle:resolve go github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/refresh_match_list_response //scouting/webserver/requests/messages:refresh_match_list_response_go_fbs

gazelle(
    name = "gazelle",
    visibility = ["//tools/lint:__subpackages__"],
)
