load("@io_bazel_rules_go//go:def.bzl", "go_library", "go_test")

go_library(
    name = "db",
    srcs = ["db.go"],
    importpath = "github.com/frc971/971-Robot-Code/scouting/db",
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = ["//visibility:public"],
    deps = [
        "@io_gorm_driver_postgres//:postgres",
        "@io_gorm_gorm//:gorm",
        "@io_gorm_gorm//clause",
        "@io_gorm_gorm//logger",
    ],
)

go_test(
    name = "db_test",
    size = "small",
    srcs = ["db_test.go"],
    data = ["//scouting/db/testdb_server"],
    embed = [":db"],
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = ["@com_github_davecgh_go_spew//spew"],
)
