load("@io_bazel_rules_go//go:def.bzl", "go_binary", "go_library")

go_binary(
    name = "sql_demo",
    embed = [":scouting_lib"],
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = ["//visibility:public"],
)

go_library(
    name = "scouting_lib",
    srcs = ["sql_demo.go"],
    importpath = "github.com/frc971/971-Robot-Code/scouting",
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = ["//visibility:private"],
    deps = ["@com_github_mattn_go_sqlite3//:go-sqlite3"],
)
