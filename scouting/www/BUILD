load("@npm//@bazel/typescript:index.bzl", "ts_project")
load("//tools/build_rules:js.bzl", "rollup_bundle")
load("@npm//@bazel/concatjs:index.bzl", "concatjs_devserver")
load("@npm//@babel/cli:index.bzl", "babel")

ts_project(
    name = "app",
    srcs = glob([
        "*.ts",
        "*.ng.html",
    ]),
    tsc = "@npm//@angular/compiler-cli/bin:ngc",
    tsconfig = "//:tsconfig.json",
    visibility = ["//visibility:public"],
    deps = [
        "@npm//@angular/animations",
        "@npm//@angular/common",
        "@npm//@angular/compiler",
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
)

concatjs_devserver(
    name = "devserver",
    serving_path = "/main_bundle.js",
    static_files = [
        ":index.html",
        "@npm//:node_modules/zone.js/dist/zone.min.js",
    ],
    deps = [":main_bundle_compiled"],
)
