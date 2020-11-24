load(":toolchain_config.bzl", "cc_toolchain_config")

package(default_visibility = ["//visibility:public"])

[
    cc_toolchain_config(
        name = "{}_toolchain_config".format(cpu),
        cpu = cpu,
    )
    for cpu in [
        "armeabi-v7a",
        "armhf-debian",
        "cortex-m4f",
        "cortex-m4f-k22",
        "k8",
        "roborio",
    ]
]

cc_library(
    name = "empty_main",
    srcs = ["empty_main.c"],
    target_compatible_with = ["@platforms//os:linux"],
)

cc_library(
    name = "malloc",
    deps = select({
        "//tools:has_asan": [],
        "//tools:has_tsan": [],
        "//tools:cpu_cortex_m4f": [],
        # TODO(phil): Support this properly.
        #"//tools:cpu_cortex_m4f_k22": [],
        "//conditions:default": ["//third_party/gperftools:tcmalloc"],
    }),
)

cc_library(
    name = "stl",
    target_compatible_with = ["@platforms//os:linux"],
)

filegroup(
    name = "empty",
    srcs = [],
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
    ar_files = ":clang_6p0_ar_files",
    as_files = ":clang_6p0_compiler_files",
    compiler_files = ":clang_6p0_compiler_files",
    dwp_files = ":empty",
    linker_files = ":clang_6p0_linker_files",
    objcopy_files = "//tools/cpp/clang_6p0:objcopy",
    strip_files = ":clang_6p0_strip_files",
    supports_param_files = 1,
    toolchain_config = ":k8_toolchain_config",
    toolchain_identifier = "k8_linux",
)

toolchain(
    name = "cc-toolchain-k8",
    exec_compatible_with = [
        "@platforms//os:linux",
        "@platforms//cpu:x86_64",
    ],
    target_compatible_with = [
        "@platforms//os:linux",
        "@platforms//cpu:x86_64",
    ],
    toolchain = ":cc-compiler-k8",
    toolchain_type = "@bazel_tools//tools/cpp:toolchain_type",
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
    ar_files = ":roborio_ar_files",
    as_files = ":roborio_compiler_files",
    compiler_files = ":roborio_compiler_files",
    dwp_files = ":empty",
    linker_files = ":roborio_linker_files",
    objcopy_files = "//tools/cpp/arm-frc-linux-gnueabi:objcopy",
    strip_files = ":roborio_strip_files",
    supports_param_files = 1,
    toolchain_config = ":roborio_toolchain_config",
    toolchain_identifier = "roborio_linux",
)

toolchain(
    name = "cc-toolchain-roborio",
    exec_compatible_with = [
        "@platforms//os:linux",
        "@platforms//cpu:x86_64",
    ],
    target_compatible_with = [
        "@platforms//os:linux",
        "//tools/platforms/hardware:roborio",
    ],
    toolchain = ":cc-compiler-roborio",
    toolchain_type = "@bazel_tools//tools/cpp:toolchain_type",
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
    ar_files = "linaro_linux_ar_files",
    as_files = "linaro_linux_compiler_files",
    compiler_files = ":linaro_linux_compiler_files",
    dwp_files = ":empty",
    linker_files = ":linaro_linux_linker_files",
    objcopy_files = "//tools/cpp/linaro_linux_gcc:objcopy",
    strip_files = ":linaro_linux_strip_files",
    supports_param_files = 1,
    toolchain_config = ":armhf-debian_toolchain_config",
    toolchain_identifier = "clang_linux_armhf",
)

toolchain(
    name = "cc-toolchain-armhf-debian",
    exec_compatible_with = [
        "@platforms//os:linux",
        "@platforms//cpu:x86_64",
    ],
    target_compatible_with = [
        "@platforms//os:linux",
        "//tools/platforms/hardware:raspberry_pi",
    ],
    toolchain = ":cc-compiler-armhf-debian",
    toolchain_type = "@bazel_tools//tools/cpp:toolchain_type",
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
    ar_files = ":gcc_arm_none_eabi_ar_files",
    compiler_files = ":gcc_arm_none_eabi_compiler_files",
    dwp_files = ":empty",
    linker_files = ":gcc_arm_none_eabi_linker_files",
    objcopy_files = "//tools/cpp/gcc_arm_none_eabi:objcopy",
    strip_files = "//tools/cpp/gcc_arm_none_eabi:strip",
    supports_param_files = 1,
    toolchain_config = ":cortex-m4f_toolchain_config",
    toolchain_identifier = "cortex-m4f",
)

toolchain(
    name = "cc-toolchain-cortex-m4f",
    exec_compatible_with = [
        "@platforms//os:linux",
        "@platforms//cpu:x86_64",
    ],
    target_compatible_with = [
        "@platforms//os:none",
        "//tools/platforms/hardware:cortex_m4f",
    ],
    toolchain = ":cc-compiler-cortex-m4f",
    toolchain_type = "@bazel_tools//tools/cpp:toolchain_type",
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
    toolchain_config = ":cortex-m4f-k22_toolchain_config",
    toolchain_identifier = "cortex-m4f-k22",
)

toolchain(
    name = "cc-toolchain-cortex-m4f-k22",
    exec_compatible_with = [
        "@platforms//os:linux",
        "@platforms//cpu:x86_64",
    ],
    target_compatible_with = [
        "@platforms//os:none",
        "//tools/platforms/hardware:cortex_m4f",
    ],
    toolchain = ":cc-compiler-cortex-m4f-k22",
    toolchain_type = "@bazel_tools//tools/cpp:toolchain_type",
)
