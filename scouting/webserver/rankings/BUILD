load("@io_bazel_rules_go//go:def.bzl", "go_library", "go_test")

go_library(
    name = "rankings",
    srcs = ["rankings.go"],
    importpath = "github.com/frc971/971-Robot-Code/scouting/webserver/rankings",
    visibility = ["//visibility:public"],
    deps = [
        "//scouting/db",
        "//scouting/scraping",
    ],
)

go_test(
    name = "rankings_test",
    srcs = ["rankings_test.go"],
    data = [
        "scouting_test_config.json",
        "//scouting/scraping:test_data",
    ],
    embed = [":rankings"],
    deps = [
        "//scouting/db",
        "//scouting/webserver/server",
    ],
)
