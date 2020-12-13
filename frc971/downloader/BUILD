py_binary(
    name = "downloader",
    srcs = [
        "downloader.py",
    ],
    data = [
        "@rsync",
        "@ssh",
        "@ssh//:scp",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)
