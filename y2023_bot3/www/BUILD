load("//frc971/downloader:downloader.bzl", "aos_downloader_dir")

filegroup(
    name = "files",
    srcs = glob([
        "**/*.html",
        "**/*.css",
        "**/*.png",
    ]),
    visibility = ["//visibility:public"],
)

aos_downloader_dir(
    name = "www_files",
    srcs = [
        ":files",
        "//frc971/analysis:plot_index_bundle.min.js",
    ],
    dir = "www",
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)
