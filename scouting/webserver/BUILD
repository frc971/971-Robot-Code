load("@io_bazel_rules_go//go:def.bzl", "go_binary", "go_library")

go_library(
    name = "webserver_lib",
    srcs = ["main.go"],
    importpath = "github.com/frc971/971-Robot-Code/scouting/webserver",
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = ["//visibility:private"],
    deps = [
        "//scouting/webserver/server",
        "//scouting/webserver/static",
    ],
)

go_binary(
    name = "webserver",
    embed = [":webserver_lib"],
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = ["//visibility:public"],
)
