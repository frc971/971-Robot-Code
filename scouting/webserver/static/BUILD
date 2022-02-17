load("@io_bazel_rules_go//go:def.bzl", "go_library", "go_test")

go_library(
    name = "static",
    srcs = ["static.go"],
    importpath = "github.com/frc971/971-Robot-Code/scouting/webserver/static",
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = ["//visibility:public"],
    deps = ["//scouting/webserver/server"],
)

go_test(
    name = "static_test",
    srcs = ["static_test.go"],
    data = [
        "test_pages/page.txt",
        "test_pages/root.txt",
    ],
    embed = [":static"],
    deps = ["//scouting/webserver/server"],
)
