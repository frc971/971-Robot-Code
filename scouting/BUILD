load("@io_bazel_rules_go//go:def.bzl", "go_binary", "go_library")
load("//tools/build_rules:js.bzl", "protractor_ts_test", "turn_files_into_runfiles")

go_binary(
    name = "sql_demo",
    embed = [":scouting_lib"],
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = ["//visibility:public"],
)

go_library(
    name = "scouting_lib",
    srcs = ["sql_demo.go"],
    importpath = "github.com/frc971/971-Robot-Code/scouting",
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = ["//visibility:private"],
    deps = ["@com_github_mattn_go_sqlite3//:go-sqlite3"],
)

turn_files_into_runfiles(
    name = "main_bundle_compiled_runfiles",
    files = "//scouting/www:main_bundle_compiled",
)

sh_binary(
    name = "scouting",
    srcs = [
        "scouting.sh",
    ],
    data = [
        ":main_bundle_compiled_runfiles",
        "//scouting/webserver",
        "//scouting/www:index.html",
        "//scouting/www:zonejs_copy",
    ],
    visibility = ["//scouting/deploy:__pkg__"],
)

protractor_ts_test(
    name = "scouting_test",
    srcs = [
        ":scouting_test.ts",
    ],
    on_prepare = ":scouting_test.protractor.on-prepare.js",
    server = ":scouting",
)
