load("@npm//@bazel/typescript:index.bzl", "ts_library")
load("//tools/build_rules:js.bzl", "rollup_bundle")
load("@npm//@bazel/concatjs:index.bzl", "concatjs_devserver")
load("@npm//@babel/cli:index.bzl", "babel")

exports_files([
    "index.html",
])

ts_library(
    name = "app",
    srcs = glob([
        "*.ts",
    ]),
    angular_assets = glob([
        "*.ng.html",
        "*.css",
    ]),
    compiler = "//tools:tsc_wrapped_with_angular",
    target_compatible_with = ["@platforms//cpu:x86_64"],
    use_angular_plugin = True,
    visibility = ["//visibility:public"],
    deps = [
        "//scouting/www/entry",
        "//scouting/www/import_match_list",
        "@npm//@angular/animations",
        "@npm//@angular/common",
        "@npm//@angular/core",
        "@npm//@angular/platform-browser",
    ],
)

rollup_bundle(
    name = "main_bundle",
    entry_point = "main.ts",
    deps = [
        "app",
    ],
)

babel(
    name = "main_bundle_compiled",
    args = [
        "$(execpath :main_bundle)",
        "--no-babelrc",
        "--source-maps",
        "--plugins=@angular/compiler-cli/linker/babel",
        "--out-dir",
        "$(@D)",
    ],
    data = [
        ":main_bundle",
        "@npm//@angular/compiler-cli",
    ],
    output_dir = True,
    visibility = ["//visibility:public"],
)

# Create a copy of zone.js here so that we can have a predictable path to
# source it from on the webserver.
genrule(
    name = "zonejs_copy",
    srcs = [
        "@npm//:node_modules/zone.js/dist/zone.min.js",
    ],
    outs = [
        "npm/node_modules/zone.js/dist/zone.min.js",
    ],
    cmd = "cp $(SRCS) $(OUTS)",
    visibility = ["//visibility:public"],
)

concatjs_devserver(
    name = "devserver",
    serving_path = "/main_bundle.js",
    static_files = [
        "index.html",
        ":zonejs_copy",
    ],
    deps = [":main_bundle_compiled"],
)

filegroup(
    name = "common_css",
    srcs = ["common.css"],
    visibility = ["//scouting/www:__subpackages__"],
)
