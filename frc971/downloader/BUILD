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
    visibility = ["//visibility:public"],
)
