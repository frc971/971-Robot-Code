package(default_visibility = ["//visibility:public"])

prefix = "arm-frc2020-linux-gnueabi"

filegroup(
    name = "gcc",
    srcs = [
        "bin/" + prefix + "-gcc",
    ],
)

filegroup(
    name = "ar",
    srcs = [
        "bin/" + prefix + "-ar",
    ],
)

filegroup(
    name = "as",
    srcs = [
        "bin/" + prefix + "-as",
    ],
)

filegroup(
    name = "ld",
    srcs = [
        "bin/" + prefix + "-ld",
    ],
)

filegroup(
    name = "nm",
    srcs = [
        "bin/" + prefix + "-nm",
    ],
)

filegroup(
    name = "objcopy",
    srcs = [
        "bin/" + prefix + "-objcopy",
    ],
)

filegroup(
    name = "objdump",
    srcs = [
        "bin/" + prefix + "-objdump",
    ],
)

filegroup(
    name = "strip",
    srcs = [
        "bin/" + prefix + "-strip",
    ],
)

filegroup(
    name = "compiler_pieces",
    srcs = glob([prefix + "/" + s for s in [
        "usr/include/**",
        "usr/lib/**",
        "lib/**",
        "bin/**",
        "**",
    ]] + [
        "libexec/gcc/" + prefix + "/7.3.0/**",
        "bin/**",
    ]),
)

filegroup(
    name = "compiler_components",
    srcs = [
        ":ar",
        ":as",
        ":gcc",
        ":ld",
        ":nm",
        ":objcopy",
        ":objdump",
        ":strip",
    ],
)
