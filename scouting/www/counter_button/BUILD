load("@npm//@bazel/typescript:index.bzl", "ts_library")

ts_library(
    name = "counter_button",
    srcs = [
        "counter_button.component.ts",
        "counter_button.module.ts",
    ],
    angular_assets = [
        "counter_button.component.css",
        "counter_button.ng.html",
    ],
    compiler = "//tools:tsc_wrapped_with_angular",
    target_compatible_with = ["@platforms//cpu:x86_64"],
    use_angular_plugin = True,
    visibility = ["//visibility:public"],
    deps = [
        "@npm//@angular/common",
        "@npm//@angular/core",
    ],
)
