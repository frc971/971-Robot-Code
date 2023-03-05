load("@io_bazel_rules_go//go:def.bzl", "go_binary", "go_library")

go_library(
    name = "testdb_server_lib",
    srcs = ["main.go"],
    importpath = "github.com/frc971/971-Robot-Code/scouting/db/testdb_server",
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = ["//visibility:private"],
    deps = ["@io_bazel_rules_go//go/runfiles:go_default_library"],
)

go_binary(
    name = "testdb_server",
    data = [
        "postgres_test.conf",
        "@postgresql_amd64//:initdb",
        "@postgresql_amd64//:postgres",
    ],
    embed = [":testdb_server_lib"],
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = ["//visibility:public"],
)
