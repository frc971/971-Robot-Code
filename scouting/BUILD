load("//tools/build_rules:apache.bzl", "apache_wrapper")
load("//tools/build_rules:js.bzl", "protractor_ts_test", "turn_files_into_runfiles")

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
        "//scouting/www:static_files",
    ],
    visibility = ["//visibility:public"],
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
