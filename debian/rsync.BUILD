genrule(
    name = "copy_rsync_wrapper",
    srcs = ["@//debian:rsync_wrapper.sh"],
    outs = ["rsync_wrapper.sh"],
    cmd = "cp $< $@",
)

sh_binary(
    name = "rsync",
    srcs = [
        "rsync_wrapper.sh",
    ],
    data = [
        "usr/bin/rsync",
        ":libs",
        "@bazel_tools//tools/bash/runfiles",
    ],
    visibility = ["//visibility:public"],
)

filegroup(
    name = "libs",
    srcs = glob([
        "usr/lib/x86_64-linux-gnu/**",
    ]),
)
