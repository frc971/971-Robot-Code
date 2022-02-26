load("@io_bazel_rules_go//go:def.bzl", "go_library")

go_library(
    name = "debug",
    srcs = ["debug.go"],
    importpath = "github.com/frc971/971-Robot-Code/scouting/webserver/requests/debug",
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = ["//visibility:public"],
    deps = [
        "//scouting/webserver/requests/messages:error_response_go_fbs",
        "//scouting/webserver/requests/messages:submit_data_scouting_response_go_fbs",
    ],
)
