load("@npm//@bazel/typescript:index.bzl", "ts_library")

ts_library(
    name = "entry",
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
        "@npm//@angular/common",
        "@npm//@angular/core",
        "@npm//@angular/forms",
    ],
)
