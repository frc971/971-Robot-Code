load("@aspect_bazel_lib//lib:copy_file.bzl", "copy_file")
load("@npm//:defs.bzl", "npm_link_all_packages")
load("//tools/build_rules:js.bzl", "ng_application")
load("//tools/build_rules/js:static.bzl", "assemble_static_files")

npm_link_all_packages(name = "node_modules")

ng_application(
    name = "app",
    assets = [
        # Explicitly blank to avoid the default.
    ],
    extra_srcs = [
        "app/common.css",
    ],
    html_assets = [
        "favicon.ico",
    ],
    deps = [
        ":node_modules",
    ],
)

assemble_static_files(
    name = "static_files",
    srcs = [
        "//third_party/y2024/field:pictures",
    ],
    app_files = ":app",
    replace_prefixes = {
        "prod": "",
        "dev": "",
        "third_party/y2024": "pictures",
    },
    tags = [
        "no-remote-cache",
    ],
    visibility = ["//visibility:public"],
)

copy_file(
    name = "app_common_css",
    src = "common.css",
    out = "app/common.css",
    visibility = [":__subpackages__"],
)
