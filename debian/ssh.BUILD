_tools = [
    "ssh",
    "scp",
]

[genrule(
    name = "copy_%s_wrapper" % tool,
    srcs = ["@//debian:ssh_wrapper.sh"],
    outs = ["%s_wrapper.sh" % tool],
    cmd = "cat $< | sed 's,%%(TOOL),usr/bin/%s,g' > $@" % tool,
) for tool in _tools]

[sh_binary(
    name = tool,
    srcs = [
        "%s_wrapper.sh" % tool,
    ],
    data = [
        "usr/bin/%s" % tool,
        ":libs",
        "@bazel_tools//tools/bash/runfiles",
    ],
    visibility = ["//visibility:public"],
) for tool in _tools]

filegroup(
    name = "libs",
    srcs = glob([
        "usr/lib/x86_64-linux-gnu/**",
    ]),
)
