load("@io_bazel_rules_go//go:def.bzl", "go_library", "go_test")

go_library(
    name = "driver_ranking",
    srcs = ["driver_ranking.go"],
    data = [
        "//scouting/DriverRank:driver_rank_script",
    ],
    importpath = "github.com/frc971/971-Robot-Code/scouting/webserver/driver_ranking",
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = ["//visibility:public"],
    deps = [
        "//scouting/db",
        "@io_bazel_rules_go//go/runfiles:go_default_library",
    ],
)

go_test(
    name = "driver_ranking_test",
    srcs = ["driver_ranking_test.go"],
    data = [
        ":fake_driver_rank_script",
    ],
    embed = [":driver_ranking"],
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        "//scouting/db",
        "@com_github_davecgh_go_spew//spew",
    ],
)

py_binary(
    name = "fake_driver_rank_script",
    testonly = True,
    srcs = [
        "fake_driver_rank_script.py",
    ],
)
