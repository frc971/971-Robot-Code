load("@aspect_bazel_lib//lib:copy_file.bzl", "copy_file")
load("@npm//:defs.bzl", "npm_link_all_packages")
load("//tools/build_rules:js.bzl", "ng_application")
load(":defs.bzl", "assemble_static_files")

npm_link_all_packages(name = "node_modules")

ng_application(
    name = "app",
    extra_srcs = [
        "app/common.css",
    ],
    html_assets = [
        "favicon.ico",
    ],
    tags = [
        "no-remote-cache",
    ],
    deps = [
        "//:node_modules/@angular/animations",
        "//scouting/www/driver_ranking",
        "//scouting/www/entry",
        "//scouting/www/match_list",
        "//scouting/www/notes",
        "//scouting/www/shift_schedule",
        "//scouting/www/view",
    ],
)

assemble_static_files(
    name = "static_files",
    app_files = ":app",
    pictures = [
        "//third_party/y2023/field:pictures",
    ],
    replace_prefixes = {
        "prod": "",
        "dev": "",
        "third_party/y2023": "pictures",
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
