load("@io_bazel_rules_go//go:def.bzl", "go_binary", "go_library")

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

go_binary(
    name = "scraping_demo",
    srcs = ["scraping_demo.go"],
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = ["//visibility:public"],
    deps = [
        "scraping",
        "@com_github_davecgh_go_spew//spew:go_default_library",
    ],
)
