load("@io_bazel_rules_go//go:def.bzl", "go_library")

go_library(
    name = "scraping",
    srcs = [
        "scrape.go",
        "types.go",
    ],
    importpath = "github.com/frc971/971-Robot-Code/scouting/scraping",
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = ["//visibility:public"],
)
