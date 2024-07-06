load("//tools/build_rules:js.bzl", "ts_project")

ts_project(
    name = "swerve_plotter",
    srcs = ["swerve_plotter.ts"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/network/www:aos_plotter",
        "//aos/network/www:colors",
        "//aos/network/www:proxy",
    ],
)
