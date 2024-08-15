load("@aspect_bazel_lib//lib:copy_file.bzl", "copy_file")
load("@aspect_rules_js//js:defs.bzl", "js_binary", "js_run_binary")
load("@npm//:@angular/service-worker/package_json.bzl", angular_service_worker = "bin")
load("@npm//:defs.bzl", "npm_link_all_packages")
load("//tools/build_rules:js.bzl", "ng_application")
load("//tools/build_rules/js:static.bzl", "assemble_static_files")
load(":defs.bzl", "assemble_service_worker_files")

npm_link_all_packages(name = "node_modules")

exports_files([
    "_static_files/authorize/index.html",
])

assemble_service_worker_files(
    name = "service_worker_files",
    outs = [
        "ngsw-worker.js",
    ],
)

OPENCV_VERSION = "4.9.0"

copy_file(
    name = "opencv.js",
    src = "@opencv_wasm//file",
    out = "assets/opencv_{}/opencv.js".format(OPENCV_VERSION),
)

ng_application(
    name = "app",
    assets = [
        "manifest.json",
        ":opencv.js",
    ],
    extra_srcs = [
        "app/common.css",
    ],
    html_assets = [
        "favicon.ico",
        "assets/971_144.png",
    ],
    deps = [
        ":node_modules",
    ],
)

assemble_static_files(
    name = "static_files",
    srcs = [
        "_static_files/authorize/index.html",
        ":ngsw.json",
        ":ngsw-worker.js",
        "//third_party/y2024/field:pictures",
    ],
    app_files = ":app",
    replace_prefixes = {
        "_static_files": "",
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
    visibility = ["//scouting/www:__subpackages__"],
)

angular_service_worker.ngsw_config_binary(
    name = "ngsw_config_binary",
)

js_binary(
    name = "ngsw_config_wrapper",
    data = [
        ":ngsw_config_binary",
    ],
    entry_point = "ngsw_config_wrapper.js",
)

js_run_binary(
    name = "ngsw_config",
    srcs = [
        "manifest.json",
        "ngsw-config.json",
        ":app",
        ":ngsw_config_binary",
    ],
    outs = [
        "ngsw.json",
    ],
    args = [
        "$(rootpath :ngsw_config_binary)",
        "$(rootpath :ngsw.json)",
        "$(rootpath :prod)",
        "$(rootpath ngsw-config.json)",
    ],
    tool = ":ngsw_config_wrapper",
)
