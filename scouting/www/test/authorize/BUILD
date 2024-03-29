load("@aspect_rules_js//js:defs.bzl", "js_test")
load("@io_bazel_rules_go//go:def.bzl", "go_binary", "go_library")
load("@npm//:defs.bzl", "npm_link_all_packages")
load("//scouting/www:defs.bzl", "assemble_static_files")
load("//tools/build_rules:js.bzl", "ng_application")

npm_link_all_packages(name = "node_modules")

go_library(
    name = "authorize_lib",
    srcs = ["server.go"],
    importpath = "github.com/frc971/971-Robot-Code/scouting/www/test/authorize",
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = ["//visibility:private"],
)

go_binary(
    name = "authorize",
    data = [
        ":static_files",
    ],
    embed = [":authorize_lib"],
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = ["//visibility:public"],
)

ng_application(
    name = "app",
    html_assets = [],
    deps = [
        ":node_modules",
    ],
)

assemble_static_files(
    name = "static_files",
    srcs = [
        "//scouting/www:_static_files/authorize/index.html",
    ],
    app_files = ":app",
    replace_prefixes = {
        "prod": "",
        "dev": "",
        "scouting/www/_static_files": "",
    },
)

js_test(
    name = "authorize_test",
    chdir = package_name(),
    copy_data_to_bin = False,
    data = [
        ":authorize",
        ":node_modules/selenium-webdriver",
        "@chrome_linux//:all",
        "@chromedriver_linux//:chromedriver",
    ],
    entry_point = "authorize_test.js",
    tags = [
        "no-remote-cache",
    ],
)
