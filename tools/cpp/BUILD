load(":toolchain_config.bzl", "cc_toolchain_config")

package(default_visibility = ["//visibility:public"])

[cc_toolchain_config(
    name = "{}_toolchain_config".format(cpu),
    cpu = cpu,
) for cpu in
    ["armeabi-v7a", "armhf-debian", "cortex-m4f", "cortex-m4f-k22", "k8", "roborio"]
]

cc_library(
    name = "empty_main",
    srcs = ["empty_main.c"],
)

cc_library(
    name = "malloc",
    deps = select({
        "//tools:has_asan": [],
        "//tools:has_tsan": [],
        "//tools:cpu_cortex_m4f": [],
        "//tools:cpu_cortex_m4f_k22": [],
        "//conditions:default": ["//third_party/gperftools:tcmalloc"],
    }),
)

cc_library(
    name = "stl",
)

filegroup(
    name = "empty",
    srcs = [],
)

# This is the entry point for --crosstool_top.
cc_toolchain_suite(
    name = "toolchain",
    toolchains = {
        "k8": ":cc-compiler-k8",
        "roborio": ":cc-compiler-roborio",
        "armhf-debian": "cc-compiler-armhf-debian",
        "cortex-m4f": "cc-compiler-cortex-m4f",
        "cortex-m4f-k22": "cc-compiler-cortex-m4f-k22",
    },
)

# Compiler inputs given by --copt etc in //tools:bazel.rc.
filegroup(
    name = "flags_compiler_inputs",
    srcs = select({
        "//tools:has_asan": [
            "asan-blacklist",
        ],
        "//tools:has_ubsan": [
            "ubsan-blacklist",
        ],
        "//conditions:default": [],
    }),
)

filegroup(
    name = "clang_6p0_all_files",
    srcs = [
        ":flags_compiler_inputs",
        "//tools/cpp/clang_6p0:as",
        "//tools/cpp/clang_6p0:tool-wrappers",
        "@clang_6p0_repo//:compiler_pieces",
    ],
)

filegroup(
    name = "clang_6p0_ar_files",
    srcs = [
        "//tools/cpp/clang_6p0:ar",
        "@clang_6p0_repo//:compiler_pieces",
    ],
)

filegroup(
    name = "clang_6p0_linker_files",
    srcs = [
        "//tools/cpp/clang_6p0:ar",
        "//tools/cpp/clang_6p0:clang",
        "//tools/cpp/clang_6p0:clang-symlinks",
        "//tools/cpp/clang_6p0:ld",
        "@clang_6p0_repo//:compiler_pieces",
    ],
)

filegroup(
    name = "clang_6p0_strip_files",
    srcs = [
        "//tools/cpp/clang_6p0:strip",
        "@clang_6p0_repo//:compiler_pieces",
    ],
)

filegroup(
    name = "clang_6p0_compiler_files",
    srcs = [
        "flags_compiler_inputs",
        "//tools/cpp/clang_6p0:clang",
        "//tools/cpp/clang_6p0:ld",
        "@clang_6p0_repo//:compiler_components",
        "@clang_6p0_repo//:compiler_pieces",
    ],
)

cc_toolchain(
    name = "cc-compiler-k8",
    all_files = ":clang_6p0_all_files",
    compiler_files = ":clang_6p0_compiler_files",
    dwp_files = ":empty",
    linker_files = ":clang_6p0_linker_files",
    ar_files = ":clang_6p0_ar_files",
    as_files = ":clang_6p0_compiler_files",
    objcopy_files = "//tools/cpp/clang_6p0:objcopy",
    strip_files = ":clang_6p0_strip_files",
    supports_param_files = 1,
    toolchain_identifier = "k8_linux",
    toolchain_config = ":k8_toolchain_config",
)

filegroup(
    name = "roborio-compiler-files",
    srcs = [
        ":flags_compiler_inputs",
        "//tools/cpp/arm-frc-linux-gnueabi:as",
        "//tools/cpp/arm-frc-linux-gnueabi:tool-wrappers",
        "@arm_frc_linux_gnueabi_repo//:compiler_pieces",
    ],
)

filegroup(
    name = "roborio_linker_files",
    srcs = [
        "//tools/cpp/arm-frc-linux-gnueabi:ar",
        "//tools/cpp/arm-frc-linux-gnueabi:gcc",
        "//tools/cpp/arm-frc-linux-gnueabi:ld",
        "//tools/cpp/arm-frc-linux-gnueabi:libs",
        "@arm_frc_linux_gnueabi_repo//:compiler_pieces",
    ],
)

filegroup(
    name = "roborio_ar_files",
    srcs = [
        "//tools/cpp/arm-frc-linux-gnueabi:ar",
        "@arm_frc_linux_gnueabi_repo//:compiler_pieces",
    ],
)

filegroup(
    name = "roborio_compiler_files",
    srcs = [
        "//tools/cpp/arm-frc-linux-gnueabi:gcc",
        "//tools/cpp/arm-frc-linux-gnueabi:ld",
        "@arm_frc_linux_gnueabi_repo//:compiler_components",
        "@arm_frc_linux_gnueabi_repo//:compiler_pieces",
    ],
)

filegroup(
    name = "roborio_strip_files",
    srcs = [
        "//tools/cpp/arm-frc-linux-gnueabi:strip",
        "@arm_frc_linux_gnueabi_repo//:compiler_pieces",
    ],
)

cc_toolchain(
    name = "cc-compiler-roborio",
    all_files = ":roborio-compiler-files",
    compiler_files = ":roborio_compiler_files",
    dwp_files = ":empty",
    linker_files = ":roborio_linker_files",
    ar_files = ":roborio_ar_files",
    as_files = ":roborio_compiler_files",
    objcopy_files = "//tools/cpp/arm-frc-linux-gnueabi:objcopy",
    strip_files = ":roborio_strip_files",
    supports_param_files = 1,
    toolchain_identifier = "roborio_linux",
    toolchain_config = ":roborio_toolchain_config",
)

filegroup(
    name = "linaro-gcc-files",
    srcs = [
        ":clang_6p0_all_files",
        "//tools/cpp/linaro_linux_gcc:clang-symlinks",
        "//tools/cpp/linaro_linux_gcc:tool-wrappers",
        "@linaro_linux_gcc_repo//:compiler_pieces",
    ],
)

filegroup(
    name = "linaro_linux_linker_files",
    srcs = [
        ":clang_6p0_linker_files",
        "//tools/cpp/linaro_linux_gcc:ar",
        "//tools/cpp/linaro_linux_gcc:clang",
        "//tools/cpp/linaro_linux_gcc:clang-ld",
        "//tools/cpp/linaro_linux_gcc:clang-symlinks",
        "//tools/cpp/linaro_linux_gcc:gcc",
        "//tools/cpp/linaro_linux_gcc:ld",
        "@linaro_linux_gcc_repo//:compiler_pieces",
    ],
)

filegroup(
    name = "linaro_linux_ar_files",
    srcs = [
        "//tools/cpp/linaro_linux_gcc:ar",
        "@linaro_linux_gcc_repo//:compiler_pieces",
    ],
)

filegroup(
    name = "linaro_linux_compiler_files",
    srcs = [
        ":clang_6p0_compiler_files",
        "//tools/cpp/linaro_linux_gcc:as",
        "//tools/cpp/linaro_linux_gcc:clang",
        "//tools/cpp/linaro_linux_gcc:gcc",
        "//tools/cpp/linaro_linux_gcc:ld",
        "@linaro_linux_gcc_repo//:compiler_pieces",
    ],
)

filegroup(
    name = "linaro_linux_strip_files",
    srcs = [
        "//tools/cpp/linaro_linux_gcc:strip",
        "@linaro_linux_gcc_repo//:compiler_pieces",
    ],
)

cc_toolchain(
    name = "cc-compiler-armhf-debian",
    all_files = ":linaro-gcc-files",
    compiler_files = ":linaro_linux_compiler_files",
    dwp_files = ":empty",
    linker_files = ":linaro_linux_linker_files",
    ar_files = "linaro_linux_ar_files",
    as_files = "linaro_linux_compiler_files",
    objcopy_files = "//tools/cpp/linaro_linux_gcc:objcopy",
    strip_files = ":linaro_linux_strip_files",
    supports_param_files = 1,
    toolchain_identifier = "clang_linux_armhf",
    toolchain_config = ":armhf-debian_toolchain_config",
)

filegroup(
    name = "gcc_arm_none_eabi_none_files",
    srcs = [
        "//tools/cpp/gcc_arm_none_eabi:tool-wrappers",
        "@gcc_arm_none_eabi//:compiler_pieces",
    ],
)

filegroup(
    name = "gcc_arm_none_eabi_compiler_files",
    srcs = [
        "//tools/cpp/gcc_arm_none_eabi:as",
        "//tools/cpp/gcc_arm_none_eabi:gcc",
        "//tools/cpp/gcc_arm_none_eabi:ld",
        "@gcc_arm_none_eabi//:compiler_pieces",
    ],
)

filegroup(
    name = "gcc_arm_none_eabi_linker_files",
    srcs = [
        "//motors/core:linkerscripts",
        "//tools/cpp/gcc_arm_none_eabi:ar",
        "//tools/cpp/gcc_arm_none_eabi:gcc",
        "//tools/cpp/gcc_arm_none_eabi:ld",
        "@gcc_arm_none_eabi//:compiler_pieces",
    ],
)

filegroup(
    name = "gcc_arm_none_eabi_ar_files",
    srcs = [
        "//tools/cpp/gcc_arm_none_eabi:ar",
        "@gcc_arm_none_eabi//:compiler_pieces",
    ],
)

cc_toolchain(
    name = "cc-compiler-cortex-m4f",
    all_files = ":gcc_arm_none_eabi_none_files",
    compiler_files = ":gcc_arm_none_eabi_compiler_files",
    dwp_files = ":empty",
    linker_files = ":gcc_arm_none_eabi_linker_files",
    ar_files = ":gcc_arm_none_eabi_ar_files",
    objcopy_files = "//tools/cpp/gcc_arm_none_eabi:objcopy",
    strip_files = "//tools/cpp/gcc_arm_none_eabi:strip",
    supports_param_files = 1,
    toolchain_identifier = "cortex-m4f",
    toolchain_config = ":cortex-m4f_toolchain_config",
)

cc_toolchain(
    name = "cc-compiler-cortex-m4f-k22",
    all_files = ":gcc_arm_none_eabi_none_files",
    compiler_files = ":gcc_arm_none_eabi_compiler_files",
    dwp_files = ":empty",
    linker_files = ":gcc_arm_none_eabi_linker_files",
    objcopy_files = ":empty",
    strip_files = ":empty",
    supports_param_files = 1,
    toolchain_identifier = "cortex-m4f-k22",
    toolchain_config = ":cortex-m4f-k22_toolchain_config",
)
