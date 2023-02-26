load("//tools/build_rules:apache.bzl", "apache_wrapper")
load("//tools/build_rules:js.bzl", "cypress_test")

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

cypress_test(
    name = "scouting_test",
    data = [
        "scouting_test.cy.js",
        "//scouting/testing:scouting_test_servers",
    ],
    runner = "scouting_test_runner.js",
)

apache_wrapper(
    name = "https",
    binary = ":scouting",
)
