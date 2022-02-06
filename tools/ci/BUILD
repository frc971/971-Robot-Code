load("@io_bazel_rules_go//go:def.bzl", "go_binary", "go_library")

go_library(
    name = "buildkite_gerrit_trigger_lib",
    srcs = ["buildkite_gerrit_trigger.go"],
    importpath = "github.com/frc971/971-Robot-Code/tools/ci",
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = ["//visibility:private"],
    deps = ["@com_github_buildkite_go_buildkite//buildkite"],
)

go_binary(
    name = "buildkite_gerrit_trigger",
    embed = [":buildkite_gerrit_trigger_lib"],
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = ["//visibility:public"],
)
