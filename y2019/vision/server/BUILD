load("//aos/seasocks:gen_embedded.bzl", "gen_embedded")
load("//aos/downloader:downloader.bzl", "aos_downloader_dir")

gen_embedded(
    name = "gen_embedded",
    srcs = glob(
        include = ["www_defaults/**/*"],
        exclude = ["www/**/*"],
    ),
)

aos_downloader_dir(
    name = "www_files",
    srcs = glob([
        "www/**/*",
    ]),
    dir = "www",
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "server",
    srcs = [
        "server.cc",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":gen_embedded",
        "//aos:init",
        "//aos/logging",
        "//aos/time",
        "//third_party/seasocks",
    ],
)
