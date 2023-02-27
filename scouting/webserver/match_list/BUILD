load("@io_bazel_rules_go//go:def.bzl", "go_library")

go_library(
    name = "match_list",
    srcs = ["match_list.go"],
    importpath = "github.com/frc971/971-Robot-Code/scouting/webserver/match_list",
    visibility = ["//visibility:public"],
    deps = [
        "//scouting/db",
        "//scouting/scraping",
    ],
)
