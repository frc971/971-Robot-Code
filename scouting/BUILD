load("//tools/build_rules:apache.bzl", "apache_wrapper")
load("//tools/build_rules:js.bzl", "protractor_ts_test")

sh_binary(
    name = "scouting",
    srcs = [
        "scouting.sh",
    ],
    data = [
        "//scouting/webserver",
        "//scouting/www:static_files",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@bazel_tools//tools/bash/runfiles",
    ],
)

protractor_ts_test(
    name = "scouting_test",
    srcs = [
        ":scouting_test.ts",
    ],
    on_prepare = ":scouting_test.protractor.on-prepare.js",
    server = "//scouting/testing:scouting_test_servers",
)

apache_wrapper(
    name = "https",
    binary = ":scouting",
)
