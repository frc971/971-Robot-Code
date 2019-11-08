package(default_visibility = ["//tools/cpp:__pkg__"])

cc_library(
    name = "libpthread",
    srcs = [
        "clang_more_libs/libpthread.so",
    ],
    visibility = ["//visibility:public"],
)

filegroup(
    name = "gcc",
    srcs = [
        "arm-linux-gnueabihf-gcc",
        "@linaro_linux_gcc_repo//:gcc",
    ],
)

filegroup(
    name = "ar",
    srcs = [
        "arm-linux-gnueabihf-ar",
        "@linaro_linux_gcc_repo//:ar",
    ],
)

filegroup(
    name = "ld",
    srcs = [
        "arm-linux-gnueabihf-ld",
        "@linaro_linux_gcc_repo//:ld",
    ],
)

filegroup(
    name = "nm",
    srcs = [
        "arm-linux-gnueabihf-nm",
        "@linaro_linux_gcc_repo//:nm",
    ],
)

filegroup(
    name = "objcopy",
    srcs = [
        "arm-linux-gnueabihf-objcopy",
        "@linaro_linux_gcc_repo//:objcopy",
    ],
)

filegroup(
    name = "objdump",
    srcs = [
        "arm-linux-gnueabihf-objdump",
        "@linaro_linux_gcc_repo//:objdump",
    ],
)

filegroup(
    name = "strip",
    srcs = [
        "arm-linux-gnueabihf-strip",
        "@linaro_linux_gcc_repo//:strip",
    ],
)

filegroup(
    name = "as",
    srcs = [
        "arm-linux-gnueabihf-as",
        "@linaro_linux_gcc_repo//:as",
    ],
)

filegroup(
    name = "clang",
    srcs = [
        "clang_bin/clang",
    ],
)

filegroup(
    name = "clang-ld",
    srcs = [
        "clang_bin/ld",
        ":ld",
    ],
)

filegroup(
    name = "tool-wrappers",
    srcs = [
        "clang_bin/as",
        ":ar",
        ":as",
        ":clang",
        ":clang-ld",
        ":gcc",
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
        "clang_syroot/**",
    ]),
)
