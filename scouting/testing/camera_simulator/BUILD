load("@io_bazel_rules_go//go:def.bzl", "go_binary", "go_library")

go_library(
    name = "camera_simulator_lib",
    srcs = ["camera_simulator.go"],
    importpath = "github.com/frc971/971-Robot-Code/scouting/testing/camera_simulator",
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = ["//visibility:private"],
)

go_binary(
    name = "camera_simulator",
    embed = [":camera_simulator_lib"],
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = ["//visibility:public"],
)
