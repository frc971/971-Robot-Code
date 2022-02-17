load("@io_bazel_rules_go//go:def.bzl", "go_binary", "go_library")

go_library(
    name = "go_flatbuffer_lib",
    srcs = ["go_fbs.go"],
    importpath = "github.com/frc971/971-Robot-Code/build_tests/go_flatbuffer",
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = ["//visibility:private"],
    deps = [
        "//build_tests:test_go_fbs",
        "@com_github_google_flatbuffers//go:go_default_library",
    ],
)

go_binary(
    name = "go_flatbuffer",
    embed = [":go_flatbuffer_lib"],
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = ["//visibility:public"],
)
