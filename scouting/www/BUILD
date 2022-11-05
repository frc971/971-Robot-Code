load("@npm//@bazel/typescript:index.bzl", "ts_library")
load("//tools/build_rules:js.bzl", "rollup_bundle")
load("@npm//@babel/cli:index.bzl", "babel")

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
        "//scouting/www/match_list",
        "//scouting/www/notes",
        "//scouting/www/shift_schedule",
        "//scouting/www/view",
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
        "$(execpath :main_bundle.min.js)",
        "--no-babelrc",
        "--source-maps",
        "--minified",
        "--no-comments",
        "--plugins=@angular/compiler-cli/linker/babel",
        "--out-dir",
        "$(@D)",
    ],
    data = [
        ":main_bundle.min.js",
        "@npm//@angular/compiler-cli",
    ],
    output_dir = True,
)

# The babel() rule above puts everything into a directory without telling bazel
# what's in the directory. That makes it annoying to work with from other
# rules. This genrule() here copies the one file in the directory we care about
# so that other rules have an easier time using the file.
genrule(
    name = "main_bundle_file",
    srcs = [":main_bundle_compiled"],
    outs = ["main_bundle_file.js"],
    cmd = "cp $(location :main_bundle_compiled)/main_bundle.min.js $(OUTS)",
)

py_binary(
    name = "index_html_generator",
    srcs = ["index_html_generator.py"],
)

genrule(
    name = "generate_index_html",
    srcs = [
        "index.template.html",
        "main_bundle_file.js",
    ],
    outs = ["index.html"],
    cmd = " ".join([
        "$(location :index_html_generator)",
        "--template $(location index.template.html)",
        "--bundle $(location main_bundle_file.js)",
        "--output $(location index.html)",
    ]),
    tools = [
        ":index_html_generator",
    ],
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
)

genrule(
    name = "field_pictures_copy",
    srcs = ["//third_party/y2022/field:pictures"],
    outs = [
        "pictures/field/balls.jpeg",
        "pictures/field/quadrants.jpeg",
        "pictures/field/reversed_quadrants.jpeg",
        "pictures/field/reversed_balls.jpeg",
    ],
    cmd = "cp $(SRCS) $(@D)/pictures/field/",
)

filegroup(
    name = "static_files",
    srcs = [
        "index.html",
        ":field_pictures_copy",
        ":main_bundle_file.js",
        ":zonejs_copy",
    ],
    visibility = ["//visibility:public"],
)

filegroup(
    name = "common_css",
    srcs = ["common.css"],
    visibility = ["//scouting/www:__subpackages__"],
)
