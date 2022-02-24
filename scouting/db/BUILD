load("@io_bazel_rules_go//go:def.bzl", "go_library", "go_test")

go_library(
    name = "db",
    srcs = ["db.go"],
    importpath = "github.com/frc971/971-Robot-Code/scouting/db",
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = ["//visibility:public"],
    deps = ["@com_github_mattn_go_sqlite3//:go-sqlite3"],
)

go_test(
    name = "db_test",
    srcs = ["db_test.go"],
    embed = [":db"],
    target_compatible_with = ["@platforms//cpu:x86_64"],
)
