load("@io_bazel_rules_go//go:def.bzl", "go_library", "go_test")

go_library(
    name = "db",
    srcs = ["db.go"],
    importpath = "github.com/frc971/971-Robot-Code/scouting/db",
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = ["//visibility:public"],
    deps = ["@com_github_jackc_pgx//stdlib"],
)

go_test(
    name = "db_test",
    size = "small",
    srcs = ["db_test.go"],
    data = [
        "//scouting/db/testdb_server",
    ],
    embed = [":db"],
    target_compatible_with = ["@platforms//cpu:x86_64"],
)
