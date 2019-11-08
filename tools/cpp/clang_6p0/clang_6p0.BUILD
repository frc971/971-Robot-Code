package(default_visibility = ["//visibility:public"])

filegroup(
    name = "clang-format",
    srcs = [
        "usr/bin/clang-6.0",
        ":compiler_pieces",
    ],
)

filegroup(
    name = "clang",
    srcs = [
        "usr/bin/clang-6.0",
    ],
)

filegroup(
    name = "ar",
    srcs = [
        "usr/bin/ar",
    ],
)

filegroup(
    name = "ld",
    srcs = [
        "usr/bin/ld",
    ],
)

filegroup(
    name = "nm",
    srcs = [
        "usr/bin/nm",
    ],
)

filegroup(
    name = "objcopy",
    srcs = [
        "usr/bin/objcopy",
    ],
)

filegroup(
    name = "objdump",
    srcs = [
        "usr/bin/objdump",
    ],
)

filegroup(
    name = "strip",
    srcs = [
        "usr/bin/strip",
    ],
)

filegroup(
    name = "as",
    srcs = [
        "usr/bin/as",
    ],
)

cc_library(
    name = "librt",
    srcs = [
        "usr/lib/x86_64-linux-gnu/librt.so",
    ],
)

cc_library(
    name = "libdl",
    srcs = [
        "usr/lib/x86_64-linux-gnu/libdl.so",
    ],
)

cc_library(
    name = "libm",
    srcs = [
        "usr/lib/x86_64-linux-gnu/libm.so",
    ],
)

filegroup(
    name = "compiler_pieces",
    srcs = glob(
        [
            "**",
        ],
        exclude = [
            "usr/share/**",
        ],
    ),
)

filegroup(
    name = "compiler_components",
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
