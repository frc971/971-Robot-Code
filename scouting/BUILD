load("@aspect_rules_cypress//cypress:defs.bzl", "cypress_module_test")
load("//tools/build_rules:apache.bzl", "apache_wrapper")

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

cypress_module_test(
    name = "scouting_test",
    args = [
        "run",
        "--config-file=cypress.config.js",
        "--browser=../../chrome_linux/chrome",
    ],
    browsers = ["@chrome_linux//:all"],
    copy_data_to_bin = False,
    cypress = "//:node_modules/cypress",
    data = [
        "cypress.config.js",
        "scouting_test.cy.js",
        "//scouting/testing:scouting_test_servers",
        "@xvfb_amd64//:wrapped_bin/Xvfb",
    ],
    runner = "cypress_runner.js",
)

apache_wrapper(
    name = "https",
    binary = ":scouting",
)
