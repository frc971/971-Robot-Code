sh_binary(
    name = "gofmt",
    srcs = ["gofmt.sh"],
    data = [
        "@go_sdk//:bin/gofmt",
    ],
    deps = [
        "@bazel_tools//tools/bash/runfiles",
    ],
)

sh_binary(
    name = "run-ci",
    srcs = [
        "run-ci.sh",
    ],
    data = [
        ":gofmt",
    ],
    deps = [
        "@bazel_tools//tools/bash/runfiles",
    ],
)
