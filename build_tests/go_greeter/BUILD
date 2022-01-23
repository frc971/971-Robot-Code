load("@io_bazel_rules_go//go:def.bzl", "go_library", "go_test")

go_library(
    name = "go_greeter",
    srcs = ["greeter.go"],
    importpath = "github.com/frc971/971-Robot-Code/build_tests/go_greeter",
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = ["//visibility:public"],
)

go_test(
    name = "go_greeter_test",
    srcs = ["greeter_test.go"],
    embed = [":go_greeter"],
    target_compatible_with = ["@platforms//cpu:x86_64"],
)
