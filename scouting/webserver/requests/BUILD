load("@io_bazel_rules_go//go:def.bzl", "go_library", "go_test")

go_library(
    name = "requests",
    srcs = ["requests.go"],
    importpath = "github.com/frc971/971-Robot-Code/scouting/webserver/requests",
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = ["//visibility:public"],
    deps = [
        "//scouting/db",
        "//scouting/webserver/requests/messages:error_response_go_fbs",
        "//scouting/webserver/requests/messages:submit_data_scouting_go_fbs",
        "//scouting/webserver/requests/messages:submit_data_scouting_response_go_fbs",
        "//scouting/webserver/server",
        "@com_github_google_flatbuffers//go:go_default_library",
    ],
)

go_test(
    name = "requests_test",
    srcs = ["requests_test.go"],
    embed = [":requests"],
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        "//scouting/db",
        "//scouting/webserver/requests/messages:error_response_go_fbs",
        "//scouting/webserver/requests/messages:submit_data_scouting_go_fbs",
        "//scouting/webserver/requests/messages:submit_data_scouting_response_go_fbs",
        "//scouting/webserver/server",
        "@com_github_google_flatbuffers//go:go_default_library",
    ],
)
