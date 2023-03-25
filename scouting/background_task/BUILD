load("@io_bazel_rules_go//go:def.bzl", "go_library", "go_test")

go_library(
    name = "background_task",
    srcs = ["background_task.go"],
    importpath = "github.com/frc971/971-Robot-Code/scouting/background_task",
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = ["//visibility:public"],
)

go_test(
    name = "background_task_test",
    srcs = ["background_task_test.go"],
    embed = [":background_task"],
)
