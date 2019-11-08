package(default_visibility = ["//tools/cpp:__pkg__"])

filegroup(
    name = "ar",
    srcs = [
        "x86_64-linux-gnu-ar",
        "@clang_6p0_repo//:ar",
    ],
)

filegroup(
    name = "ld",
    srcs = [
        "x86_64-linux-gnu-ld",
        "@clang_6p0_repo//:ld",
    ],
)

filegroup(
    name = "nm",
    srcs = [
        "x86_64-linux-gnu-nm",
        "@clang_6p0_repo//:nm",
    ],
)

filegroup(
    name = "objcopy",
    srcs = [
        "x86_64-linux-gnu-objcopy",
        "@clang_6p0_repo//:objcopy",
    ],
)

filegroup(
    name = "objdump",
    srcs = [
        "x86_64-linux-gnu-objdump",
        "@clang_6p0_repo//:objdump",
    ],
)

filegroup(
    name = "strip",
    srcs = [
        "x86_64-linux-gnu-strip",
        "@clang_6p0_repo//:strip",
    ],
)

filegroup(
    name = "as",
    srcs = [
        "x86_64-linux-gnu-as",
        "@clang_6p0_repo//:as",
    ],
)

filegroup(
    name = "clang",
    srcs = [
        "x86_64-linux-gnu-clang-6.0",
        "@clang_6p0_repo//:clang",
    ],
)

filegroup(
    name = "tool-wrappers",
    srcs = [
        ":ar",
        ":as",
        ":clang",
        ":ld",
        ":nm",
        ":objcopy",
        ":objdump",
        ":strip",
    ],
)

filegroup(
    name = "clang-symlinks",
    srcs = glob([
        "clang_more_libs/**",
    ]),
)
