load("@npm//@bazel/typescript:index.bzl", "ts_library")
load("//tools/build_rules:js.bzl", "rollup_bundle")

package(default_visibility = ["//visibility:public"])

ts_library(
    name = "cpp_plot",
    srcs = ["cpp_plot.ts"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos:configuration_ts_fbs",
        "//aos/network/www:proxy",
        "//frc971/analysis:plot_data_utils",
    ],
)

rollup_bundle(
    name = "cpp_plot_bundle",
    entry_point = "cpp_plot.ts",
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":cpp_plot",
    ],
)

genrule(
    name = "copy_css",
    srcs = [
        "//aos/network/www:styles.css",
    ],
    outs = ["styles.css"],
    cmd = "cp $< $@",
)

filegroup(
    name = "cpp_plot_files",
    srcs = [
        "cpp_plot_bundle.js",
        "cpp_plot_bundle.min.js",
        "index.html",
        "styles.css",
    ],
)
